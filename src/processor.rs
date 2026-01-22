use crate::{
    error::{XacroError, IMPLEMENTED_FEATURES, UNIMPLEMENTED_FEATURES},
    expander::{expand_node, XacroContext},
    extensions::ExtensionHandler,
    parse::xml::{extract_xacro_namespace, is_known_xacro_uri},
};
use xmltree::XMLNode;

use ::core::{cell::RefCell, str::FromStr};
use std::{collections::HashMap, path::PathBuf, rc::Rc};
use thiserror::Error;

/// Error type for invalid compatibility mode strings
#[derive(Debug, Error)]
pub enum CompatModeParseError {
    #[error("Compatibility mode cannot be empty (valid: all, namespace, duplicate_params)")]
    Empty,
    #[error("Unknown compatibility mode: '{0}' (valid: all, namespace, duplicate_params)")]
    Unknown(String),
}

/// Python xacro compatibility modes
#[derive(Debug, Clone, Copy, Default)]
pub struct CompatMode {
    /// Accept duplicate macro parameters (last declaration wins)
    pub duplicate_params: bool,
    /// Accept namespace URIs that don't match known xacro URIs
    pub namespace: bool,
}

impl CompatMode {
    /// No compatibility mode (strict validation)
    pub fn none() -> Self {
        Self {
            duplicate_params: false,
            namespace: false,
        }
    }

    /// All compatibility modes enabled
    pub fn all() -> Self {
        Self {
            duplicate_params: true,
            namespace: true,
        }
    }
}

impl FromStr for CompatMode {
    type Err = CompatModeParseError;

    /// Parse compatibility mode from string
    ///
    /// Supported formats:
    /// - "all" → all modes enabled
    /// - "namespace" → only namespace mode
    /// - "duplicate_params" → only duplicate params mode
    /// - "namespace,duplicate_params" → multiple modes (comma-separated)
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Reject empty or whitespace-only strings to prevent silent misconfigurations
        let s = s.trim();
        if s.is_empty() {
            return Err(CompatModeParseError::Empty);
        }

        let mut mode = Self::none();

        for part in s.split(',') {
            let part = part.trim();
            if part.is_empty() {
                continue;
            }
            match part {
                "all" => return Ok(Self::all()),
                "namespace" => mode.namespace = true,
                "duplicate_params" => mode.duplicate_params = true,
                _ => return Err(CompatModeParseError::Unknown(part.to_string())),
            }
        }

        Ok(mode)
    }
}

pub struct XacroProcessor {
    /// Maximum recursion depth for macro expansion and insert_block
    /// Default: 50 (set conservatively to prevent stack overflow)
    max_recursion_depth: usize,
    /// CLI arguments passed to the processor (for xacro:arg support)
    /// These take precedence over XML defaults
    args: HashMap<String, String>,
    /// Python xacro compatibility modes
    compat_mode: CompatMode,
    /// Extension handlers for $(command args...) resolution
    extensions: Rc<Vec<Box<dyn ExtensionHandler>>>,
    /// YAML tag handlers for custom tags (e.g., !degrees, !millimeters)
    #[cfg(feature = "yaml")]
    yaml_tag_handlers: Rc<crate::eval::yaml_tag_handler::YamlTagHandlerRegistry>,
}

/// Builder for configuring XacroProcessor with custom settings.
///
/// Provides a fluent API for setting optional parameters without constructor explosion.
///
/// # Example
/// ```
/// use xacro::XacroProcessor;
///
/// let processor = XacroProcessor::builder()
///     .with_arg("robot_name", "my_robot")
///     .with_arg("use_lidar", "true")
///     .with_max_depth(100)
///     .build();
/// ```
pub struct XacroBuilder {
    args: HashMap<String, String>,
    max_recursion_depth: usize,
    compat_mode: CompatMode,
    extensions: Option<Vec<Box<dyn ExtensionHandler>>>,
    #[cfg(feature = "yaml")]
    yaml_tag_handlers: Option<crate::eval::yaml_tag_handler::YamlTagHandlerRegistry>,
}

impl XacroBuilder {
    /// Create a new builder with default settings.
    fn new() -> Self {
        Self {
            args: HashMap::new(),
            max_recursion_depth: XacroContext::DEFAULT_MAX_DEPTH,
            compat_mode: CompatMode::none(),
            extensions: None, // None = use default extensions
            #[cfg(feature = "yaml")]
            yaml_tag_handlers: None, // None = empty registry (no default handlers)
        }
    }

    /// Add a single CLI argument.
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_arg("scale", "0.5")
    ///     .build();
    /// ```
    pub fn with_arg(
        mut self,
        name: impl Into<String>,
        value: impl Into<String>,
    ) -> Self {
        self.args.insert(name.into(), value.into());
        self
    }

    /// Add multiple CLI arguments from a HashMap.
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    /// use std::collections::HashMap;
    ///
    /// let mut args = HashMap::new();
    /// args.insert("scale".to_string(), "0.5".to_string());
    /// args.insert("prefix".to_string(), "robot_".to_string());
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_args(args)
    ///     .build();
    /// ```
    pub fn with_args(
        mut self,
        args: HashMap<String, String>,
    ) -> Self {
        self.args.extend(args);
        self
    }

    /// Set the maximum recursion depth.
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_max_depth(100)
    ///     .build();
    /// ```
    pub fn with_max_depth(
        mut self,
        max_depth: usize,
    ) -> Self {
        self.max_recursion_depth = max_depth;
        self
    }

    /// Enable all compatibility modes.
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_compat_all()
    ///     .build();
    /// ```
    pub fn with_compat_all(mut self) -> Self {
        self.compat_mode = CompatMode::all();
        self
    }

    /// Set a specific compatibility mode.
    ///
    /// # Example
    /// ```
    /// use xacro::{XacroProcessor, CompatMode};
    ///
    /// let compat = "namespace,duplicate_params".parse().unwrap();
    /// let processor = XacroProcessor::builder()
    ///     .with_compat_mode(compat)
    ///     .build();
    /// ```
    pub fn with_compat_mode(
        mut self,
        mode: CompatMode,
    ) -> Self {
        self.compat_mode = mode;
        self
    }

    /// Add a custom extension handler.
    ///
    /// By default, the processor includes CwdExtension and EnvExtension.
    /// Note: $(arg ...) is handled specially, not via an extension handler.
    /// This method allows adding additional custom extensions without replacing the defaults.
    ///
    /// # Example
    /// ```ignore
    /// use xacro::XacroProcessor;
    /// use xacro::extensions::ExtensionHandler;
    ///
    /// struct MyExtension;
    /// impl ExtensionHandler for MyExtension {
    ///     fn resolve(&self, command: &str, args_raw: &str)
    ///         -> Result<Option<String>, Box<dyn std::error::Error>> {
    ///         if command == "my_ext" {
    ///             Ok(Some(format!("Custom: {}", args_raw)))
    ///         } else {
    ///             Ok(None)
    ///         }
    ///     }
    /// }
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_extension(Box::new(MyExtension))
    ///     .build();
    /// ```
    pub fn with_extension(
        mut self,
        handler: Box<dyn ExtensionHandler>,
    ) -> Self {
        self.extensions
            .get_or_insert_with(Self::default_extensions)
            .push(handler);
        self
    }

    /// Clear all extensions (including defaults).
    ///
    /// Use this if you want to provide a completely custom extension list
    /// without any of the built-in extensions.
    ///
    /// # Example
    /// ```ignore
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::builder()
    ///     .clear_extensions()
    ///     .with_extension(Box::new(MyExtension))
    ///     .build();
    /// ```
    pub fn clear_extensions(mut self) -> Self {
        self.extensions = Some(Vec::new());
        self
    }

    /// Register a YAML tag handler
    ///
    /// Handlers are tried in registration order. Register more specific handlers
    /// before more general ones.
    ///
    /// # Example
    /// ```ignore
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_yaml_tag_handler(Box::new(MyHandler))
    ///     .build();
    /// ```
    #[cfg(feature = "yaml")]
    pub fn with_yaml_tag_handler(
        mut self,
        handler: crate::extensions::DynYamlTagHandler,
    ) -> Self {
        self.yaml_tag_handlers
            .get_or_insert_with(crate::eval::yaml_tag_handler::YamlTagHandlerRegistry::new)
            .register(handler);
        self
    }

    /// Enable ROS unit conversions (degrees, radians, millimeters, etc.)
    ///
    /// This is a convenience method for ROS users. Registers the RosUnitTagHandler
    /// which handles standard ROS unit tags like !degrees, !millimeters, etc.
    ///
    /// # Example
    /// ```ignore
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_ros_yaml_units()
    ///     .build();
    /// ```
    #[cfg(feature = "yaml")]
    pub fn with_ros_yaml_units(self) -> Self {
        self.with_yaml_tag_handler(Box::new(
            crate::extensions::ros_yaml_handlers::RosUnitTagHandler::new(),
        ))
    }

    /// Build the XacroProcessor with the configured settings.
    pub fn build(self) -> XacroProcessor {
        let extensions = Rc::new(self.extensions.unwrap_or_else(Self::default_extensions));

        XacroProcessor {
            max_recursion_depth: self.max_recursion_depth,
            args: self.args,
            compat_mode: self.compat_mode,
            extensions,
            #[cfg(feature = "yaml")]
            yaml_tag_handlers: Rc::new(self.yaml_tag_handlers.unwrap_or_default()),
        }
    }

    /// Create default extension handlers (CwdExtension, EnvExtension).
    ///
    /// This delegates to the centralized `extensions::core::default_extensions()`.
    /// ROS extensions (FindExtension, OptEnvExtension) are NOT included by default.
    /// Library users should explicitly add them via builder pattern if needed.
    /// The CLI binary adds ROS extensions automatically for user convenience.
    ///
    /// Note: $(arg ...) is handled specially in `EvalContext::resolve_extension()`
    /// to ensure correct interaction with the shared arguments map.
    fn default_extensions() -> Vec<Box<dyn ExtensionHandler>> {
        crate::extensions::core::default_extensions()
    }
}

impl Default for XacroProcessor {
    fn default() -> Self {
        Self::builder().build()
    }
}

impl XacroProcessor {
    /// Create a new builder for configuring the processor.
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::builder()
    ///     .with_arg("robot_name", "my_robot")
    ///     .with_max_depth(100)
    ///     .build();
    /// ```
    pub fn builder() -> XacroBuilder {
        XacroBuilder::new()
    }

    /// Create a new xacro processor with default settings.
    ///
    /// For custom configuration, use [`XacroProcessor::builder()`].
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    ///
    /// let processor = XacroProcessor::new();
    /// let input = r#"<?xml version="1.0"?>
    /// <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
    ///   <xacro:property name="value" value="42"/>
    ///   <link name="base"><inertial><mass value="${value}"/></inertial></link>
    /// </robot>"#;
    /// let output = processor.run_from_string(input)?;
    /// assert!(output.contains("mass value=\"42\""));
    /// # Ok::<(), xacro::XacroError>(())
    /// ```
    pub fn new() -> Self {
        Self::default()
    }

    /// Process xacro content from a file path
    pub fn run<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> Result<String, XacroError> {
        // Thin wrapper over run_with_deps that discards dependency list
        self.run_with_deps(path).map(|(output, _)| output)
    }

    /// Process xacro content from a file path and return included files
    ///
    /// Returns a tuple of (processed_output, included_files).
    /// The included_files list contains paths to all files that were included
    /// during processing via `<xacro:include>` directives.
    pub fn run_with_deps<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> Result<(String, Vec<PathBuf>), XacroError> {
        let doc = XacroProcessor::parse_file(&path)?;
        self.run_impl(
            doc,
            path.as_ref()
                .parent()
                .unwrap_or_else(|| std::path::Path::new(".")),
        )
    }

    /// Process xacro content from a string
    ///
    /// # Note
    /// Any `<xacro:include>` directives with relative paths will be resolved
    /// relative to the current working directory.
    pub fn run_from_string(
        &self,
        content: &str,
    ) -> Result<String, XacroError> {
        // Thin wrapper over run_from_string_with_deps that discards dependency list
        self.run_from_string_with_deps(content)
            .map(|(output, _)| output)
    }

    /// Process xacro content from a string and return included files
    ///
    /// Returns a tuple of (processed_output, included_files).
    /// The included_files list contains paths to all files that were included
    /// during processing via `<xacro:include>` directives.
    ///
    /// # Note
    /// Any `<xacro:include>` directives with relative paths will be resolved
    /// relative to the current working directory.
    pub fn run_from_string_with_deps(
        &self,
        content: &str,
    ) -> Result<(String, Vec<PathBuf>), XacroError> {
        let doc = crate::parse::document::XacroDocument::parse(content.as_bytes())?;
        // Use current directory as base path for any includes in test content
        self.run_impl(doc, std::path::Path::new("."))
    }

    /// Internal implementation
    fn run_impl(
        &self,
        mut doc: crate::parse::document::XacroDocument,
        base_path: &std::path::Path,
    ) -> Result<(String, Vec<PathBuf>), XacroError> {
        // Extract xacro namespace from document root (if present)
        // Strategy:
        // 1. Try standard "xacro" prefix (e.g., xmlns:xacro="...")
        // 2. Fallback: search for any prefix bound to a known xacro URI
        // 3. If not found, use empty string (lazy checking - only error if xacro elements actually used)
        //
        // Documents with NO xacro elements don't need xacro namespace declaration.
        // Only error during finalize_tree if xacro elements are found.
        //
        // When in namespace compat mode, skip URI validation to accept files with
        // "typo" URIs like xmlns:xacro="...#interface" that Python xacro accepts
        let xacro_ns = extract_xacro_namespace(&doc.root, self.compat_mode.namespace)?;

        // Create expansion context with CLI arguments, compat mode, and extensions
        // Math constants (pi, e, tau, etc.) are automatically initialized by EvalContext::new()
        // CLI args are passed to the context and take precedence over XML defaults
        //
        // Note: $(arg ...) is handled specially in resolve_extension using the shared
        // args map, so ArgExtension is not included in the extensions list
        let args_rc = Rc::new(RefCell::new(self.args.clone()));

        let mut ctx = XacroContext::new_with_extensions(
            base_path.to_path_buf(),
            xacro_ns.clone(),
            args_rc,
            self.compat_mode,
            self.extensions.clone(),
            #[cfg(feature = "yaml")]
            self.yaml_tag_handlers.clone(), // Rc clone is cheap
        );
        ctx.set_max_recursion_depth(self.max_recursion_depth);

        // Expand the root element itself. This will handle attributes on the root
        // and any xacro directives at the root level (though unlikely).
        let expanded_nodes = expand_node(XMLNode::Element(doc.root), &ctx)?;

        // The expansion of the root must result in a single root element.
        if expanded_nodes.len() != 1 {
            return Err(XacroError::InvalidRoot(format!(
                "Root element expanded to {} nodes, expected 1",
                expanded_nodes.len()
            )));
        }

        doc.root = match expanded_nodes.into_iter().next().unwrap() {
            XMLNode::Element(elem) => elem,
            _ => {
                return Err(XacroError::InvalidRoot(
                    "Root element expanded to a non-element node (e.g., text or comment)"
                        .to_string(),
                ))
            }
        };

        // Final cleanup: check for unprocessed xacro elements and remove namespace
        Self::finalize_tree(&mut doc.root, &xacro_ns, &self.compat_mode)?;

        let output = XacroProcessor::serialize(&doc)?;
        let includes = ctx.get_all_includes();
        Ok((output, includes))
    }

    fn finalize_tree_children(
        element: &mut xmltree::Element,
        xacro_ns: &str,
        compat_mode: &CompatMode,
    ) -> Result<(), XacroError> {
        for child in &mut element.children {
            if let Some(child_elem) = child.as_mut_element() {
                Self::finalize_tree(child_elem, xacro_ns, compat_mode)?;
            }
        }
        Ok(())
    }

    fn finalize_tree(
        element: &mut xmltree::Element,
        xacro_ns: &str,
        compat_mode: &CompatMode,
    ) -> Result<(), XacroError> {
        // Check if this element is in the xacro namespace (indicates unprocessed feature)
        // Must check namespace URI, not prefix, to handle namespace aliasing (e.g., xmlns:x="...")

        // Remove xacro namespace declarations from all elements first
        // Strategy: Remove prefixes that are:
        // 1. Literally named "xacro" (regardless of URI)
        // 2. Bound to known standard xacro URIs (defense-in-depth)
        // This handles both standard URIs and non-standard URIs accepted in compat mode.
        // Do this BEFORE checking for unprocessed features to ensure cleanup happens regardless.
        //
        // Note: We DON'T remove non-"xacro" prefixes bound to non-standard URIs, even if
        // that URI happens to be used as xacro_ns in compat mode. Those prefixes may be
        // actively used in the document (namespace collision case).
        if !xacro_ns.is_empty() {
            if let Some(ref mut ns) = element.namespaces {
                // Find prefixes to remove
                let prefixes_to_remove: Vec<String> =
                    ns.0.iter()
                        .filter(|(prefix, uri)| {
                            // Always remove "xacro" prefix
                            if prefix.as_str() == "xacro" {
                                return true;
                            }
                            // Remove other prefixes only if bound to known standard URIs
                            is_known_xacro_uri(uri.as_str())
                        })
                        .map(|(prefix, _)| prefix.clone())
                        .collect();

                // Remove all found prefixes
                for prefix in prefixes_to_remove {
                    ns.0.remove(&prefix);
                }
            }
        }

        // Case 1: Element has namespace and matches declared xacro namespace
        if !xacro_ns.is_empty() && element.namespace.as_deref() == Some(xacro_ns) {
            // Compat mode: handle namespace collision (same URI bound to multiple prefixes)
            // If the element uses a non-"xacro" prefix, Python xacro ignores it based on prefix string check.
            // In strict mode, this is a hard error (poor XML practice).
            if compat_mode.namespace {
                let prefix = element.prefix.as_deref().unwrap_or("");
                if prefix != "xacro" {
                    let element_display = if prefix.is_empty() {
                        format!("<{}>", element.name)
                    } else {
                        format!("<{}:{}>", prefix, element.name)
                    };
                    log::warn!(
                        "Namespace collision: {} uses xacro namespace URI but different prefix (compat mode)",
                        element_display
                    );
                    // Pass through - recursively finalize children but don't error
                    Self::finalize_tree_children(element, xacro_ns, compat_mode)?;
                    return Ok(());
                }
            }

            // Use centralized feature lists for consistent error messages
            return Err(XacroError::UnimplementedFeature(format!(
                "<xacro:{}>\n\
                     This element was not processed. Either:\n\
                     1. The feature is not implemented yet (known unimplemented: {})\n\
                     2. There's a bug in the processor\n\
                     \n\
                     Currently implemented: {}",
                element.name,
                UNIMPLEMENTED_FEATURES.join(", "),
                IMPLEMENTED_FEATURES.join(", ")
            )));
        }

        // Case 2: Element has a known xacro namespace but no namespace was declared in root
        // This is the lazy checking: only error if xacro elements are actually used
        if xacro_ns.is_empty() {
            if let Some(elem_ns) = element.namespace.as_deref() {
                if is_known_xacro_uri(elem_ns) {
                    return Err(XacroError::MissingNamespace(format!(
                        "Found xacro element <{}> with namespace '{}', but no xacro namespace declared in document root. \
                         Please add xmlns:xacro=\"{}\" to your root element.",
                        element.name, elem_ns, elem_ns
                    )));
                }
            }
        }

        // Recursively process children
        Self::finalize_tree_children(element, xacro_ns, compat_mode)?;

        Ok(())
    }

    /// Parse a xacro document from a file path
    pub(crate) fn parse_file<P: AsRef<std::path::Path>>(
        path: P
    ) -> Result<crate::parse::XacroDocument, XacroError> {
        let file = std::fs::File::open(path)?;
        crate::parse::XacroDocument::parse(file)
    }

    /// Serialize a xacro document to a string
    pub(crate) fn serialize(doc: &crate::parse::XacroDocument) -> Result<String, XacroError> {
        let mut writer = Vec::new();
        doc.write(&mut writer)?;
        Ok(String::from_utf8(writer)?)
    }
}

use crate::{
    error::XacroError,
    expander::{expand_node, XacroContext},
    utils::xml::{extract_xacro_namespace, is_known_xacro_uri},
};
use xmltree::XMLNode;

use core::str::FromStr;
use std::collections::HashMap;
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
}

impl XacroProcessor {
    /// Create a new xacro processor with default settings
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self::new_with_args(HashMap::new())
    }

    /// Create a new xacro processor with CLI arguments
    ///
    /// # Arguments
    /// * `args` - Map of argument names to values (from CLI key:=value format)
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
    /// let processor = XacroProcessor::new_with_args(args);
    /// ```
    pub fn new_with_args(args: HashMap<String, String>) -> Self {
        Self::new_with_compat(args, false)
    }

    /// Create a new xacro processor with custom max recursion depth
    ///
    /// # Arguments
    /// * `max_depth` - Maximum recursion depth before triggering error (recommended: 50-100)
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    /// let processor = XacroProcessor::new_with_depth(100);
    /// ```
    pub fn new_with_depth(max_depth: usize) -> Self {
        Self {
            max_recursion_depth: max_depth,
            args: HashMap::new(),
            compat_mode: CompatMode::none(),
        }
    }

    /// Create a new xacro processor with CLI arguments and compat mode
    ///
    /// # Arguments
    /// * `args` - Map of argument names to values (from CLI key:=value format)
    /// * `compat` - Enable Python xacro compatibility mode (accept buggy inputs)
    ///
    /// # Example
    /// ```
    /// use xacro::XacroProcessor;
    /// use std::collections::HashMap;
    ///
    /// let args = HashMap::new();
    /// let processor = XacroProcessor::new_with_compat(args, true);
    /// ```
    pub fn new_with_compat(
        args: HashMap<String, String>,
        compat: bool,
    ) -> Self {
        Self {
            max_recursion_depth: XacroContext::DEFAULT_MAX_DEPTH,
            args,
            compat_mode: if compat {
                CompatMode::all()
            } else {
                CompatMode::none()
            },
        }
    }

    /// Create a new xacro processor with specific compatibility modes
    ///
    /// # Arguments
    /// * `args` - Map of argument names to values (from CLI key:=value format)
    /// * `compat_mode` - Compatibility mode configuration
    ///
    /// # Example
    /// ```
    /// use xacro::{XacroProcessor, CompatMode};
    /// use std::collections::HashMap;
    ///
    /// let args = HashMap::new();
    /// let compat = "namespace,duplicate_params".parse().unwrap();
    /// let processor = XacroProcessor::new_with_compat_mode(args, compat);
    /// ```
    pub fn new_with_compat_mode(
        args: HashMap<String, String>,
        compat_mode: CompatMode,
    ) -> Self {
        Self {
            max_recursion_depth: XacroContext::DEFAULT_MAX_DEPTH,
            args,
            compat_mode,
        }
    }

    /// Process xacro content from a file path
    pub fn run<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> Result<String, XacroError> {
        let xml = XacroProcessor::parse_file(&path)?;
        self.run_impl(
            xml,
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
        let xml = xmltree::Element::parse(content.as_bytes())?;
        // Use current directory as base path for any includes in test content
        self.run_impl(xml, std::path::Path::new("."))
    }

    /// Internal implementation
    fn run_impl(
        &self,
        mut root: xmltree::Element,
        base_path: &std::path::Path,
    ) -> Result<String, XacroError> {
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
        let xacro_ns = extract_xacro_namespace(&root, self.compat_mode.namespace)?;

        // Create expansion context with CLI arguments and compat mode
        // Math constants (pi, e, tau, etc.) are automatically initialized by PropertyProcessor::new()
        // CLI args are passed to the context and take precedence over XML defaults
        let mut ctx = XacroContext::new_with_compat(
            base_path.to_path_buf(),
            xacro_ns.clone(),
            self.args.clone(),
            self.compat_mode,
        );
        ctx.set_max_recursion_depth(self.max_recursion_depth);

        // Expand the root element itself. This will handle attributes on the root
        // and any xacro directives at the root level (though unlikely).
        let expanded_nodes = expand_node(XMLNode::Element(root), &ctx)?;

        // The expansion of the root must result in a single root element.
        if expanded_nodes.len() != 1 {
            return Err(XacroError::InvalidRoot(format!(
                "Root element expanded to {} nodes, expected 1",
                expanded_nodes.len()
            )));
        }

        root = match expanded_nodes.into_iter().next().unwrap() {
            XMLNode::Element(elem) => elem,
            _ => {
                return Err(XacroError::InvalidRoot(
                    "Root element expanded to a non-element node (e.g., text or comment)"
                        .to_string(),
                ))
            }
        };

        // Final cleanup: check for unprocessed xacro elements and remove namespace
        Self::finalize_tree(&mut root, &xacro_ns)?;

        XacroProcessor::serialize(&root)
    }

    fn finalize_tree(
        element: &mut xmltree::Element,
        xacro_ns: &str,
    ) -> Result<(), XacroError> {
        // Check if this element is in the xacro namespace (indicates unprocessed feature)
        // Must check namespace URI, not prefix, to handle namespace aliasing (e.g., xmlns:x="...")

        // Case 1: Element has namespace and matches declared xacro namespace
        if !xacro_ns.is_empty() && element.namespace.as_deref() == Some(xacro_ns) {
            // Use centralized feature lists for consistent error messages
            use crate::error::{IMPLEMENTED_FEATURES, UNIMPLEMENTED_FEATURES};
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

        // Remove ALL known xacro namespace declarations (if namespace was declared)
        // This handles cases where included files use different xacro URI variants
        // Find and remove whichever prefixes are bound to ANY known xacro namespace URI
        // This handles both standard (xmlns:xacro="...") and non-standard (xmlns:foo="...") prefixes
        if !xacro_ns.is_empty() {
            if let Some(ref mut ns) = element.namespaces {
                // Find all prefixes bound to ANY known xacro namespace URI
                let prefixes_to_remove: Vec<String> =
                    ns.0.iter()
                        .filter(|(_, uri)| is_known_xacro_uri(uri.as_str()))
                        .map(|(prefix, _)| prefix.clone())
                        .collect();

                // Remove all found prefixes
                for prefix in prefixes_to_remove {
                    ns.0.remove(&prefix);
                }
            }
        }

        // Recursively process children
        for child in &mut element.children {
            if let Some(child_elem) = child.as_mut_element() {
                Self::finalize_tree(child_elem, xacro_ns)?;
            }
        }

        Ok(())
    }
}

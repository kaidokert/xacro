use crate::{
    error::XacroError,
    expander::{expand_node, XacroContext},
};

pub struct XacroProcessor {
    /// Maximum recursion depth for macro expansion and insert_block
    /// Default: 50 (set conservatively to prevent stack overflow)
    max_recursion_depth: usize,
}

impl XacroProcessor {
    /// Known xacro namespace URIs used in the wild
    /// Used for fallback namespace detection and lazy checking
    const KNOWN_XACRO_URIS: &'static [&'static str] = &[
        "http://www.ros.org/wiki/xacro",
        "http://ros.org/wiki/xacro",
        "http://wiki.ros.org/xacro",
        "http://www.ros.org/xacro",
        "http://playerstage.sourceforge.net/gazebo/xmlschema/#xacro",
    ];

    /// Create a new xacro processor with default settings
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            max_recursion_depth: 50,
        }
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
        }
    }

    /// Search namespace map for any prefix bound to a known xacro URI
    fn find_xacro_namespace_in_map(ns: &xmltree::Namespace) -> Option<String> {
        ns.0.values()
            .find(|uri| Self::KNOWN_XACRO_URIS.contains(&uri.as_str()))
            .map(|s| s.to_string())
    }

    /// Check if a namespace URI is a known xacro namespace
    fn is_known_xacro_uri(uri: &str) -> bool {
        Self::KNOWN_XACRO_URIS.contains(&uri)
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
        // Validate xacro namespace and extract it
        // First check if "xacro" prefix exists but is bound to invalid URI (catch typos)
        if let Some(ns) = root.namespaces.as_ref() {
            if let Some(xacro_uri) = ns.get("xacro") {
                let uri_str: &str = xacro_uri;
                if !Self::KNOWN_XACRO_URIS.contains(&uri_str) {
                    return Err(XacroError::MissingNamespace(format!(
                        "The 'xacro' prefix is bound to an unknown URI: '{}'. \
                         This might be a typo. Known xacro URIs are: {}",
                        xacro_uri,
                        Self::KNOWN_XACRO_URIS.join(", ")
                    )));
                }
            }
        }

        let xacro_ns: String = root
            .namespaces
            .as_ref()
            .and_then(|ns| {
                ns.get("xacro")
                    .map(|s| s.to_string())
                    .or_else(|| Self::find_xacro_namespace_in_map(ns))
            })
            .unwrap_or_default();

        // Create expansion context
        let mut ctx = XacroContext::new(base_path.to_path_buf(), xacro_ns.clone());
        ctx.set_max_recursion_depth(self.max_recursion_depth);

        // Initialize math constants (pi, e, etc.)
        // These are automatically added by PropertyProcessor::new()
        // but we need to ensure they're in the context
        let math_constants = [
            ("pi", core::f64::consts::PI.to_string()),
            ("e", core::f64::consts::E.to_string()),
            ("tau", core::f64::consts::TAU.to_string()),
            ("M_PI", core::f64::consts::PI.to_string()),
            ("inf", f64::INFINITY.to_string()),
            ("nan", f64::NAN.to_string()),
        ];
        for (name, value) in math_constants {
            ctx.properties.add_raw_property(name.to_string(), value);
        }

        // Expand the root element's children (keep root element itself)
        let mut expanded_children = Vec::new();
        for child in core::mem::take(&mut root.children) {
            expanded_children.extend(expand_node(child, &mut ctx)?);
        }
        root.children = expanded_children;

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
                if Self::is_known_xacro_uri(elem_ns) {
                    return Err(XacroError::MissingNamespace(format!(
                        "Found xacro element <{}> with namespace '{}', but no xacro namespace declared in document root. \
                         Please add xmlns:xacro=\"{}\" to your root element.",
                        element.name, elem_ns, elem_ns
                    )));
                }
            }
        }

        // Remove xacro namespace declaration (if namespace was declared)
        // Find and remove whichever prefix is bound to the xacro namespace URI
        // This handles both standard (xmlns:xacro="...") and non-standard (xmlns:foo="...") prefixes
        if !xacro_ns.is_empty() {
            if let Some(ref mut ns) = element.namespaces {
                // Find all prefixes bound to the xacro namespace URI
                let prefixes_to_remove: Vec<String> =
                    ns.0.iter()
                        .filter(|(_, uri)| uri.as_str() == xacro_ns)
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

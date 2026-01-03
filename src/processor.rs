use crate::{
    error::XacroError,
    features::{
        conditions::ConditionProcessor, includes::IncludeProcessor, loops::LoopProcessor,
        macros::MacroProcessor, properties::PropertyProcessor,
    },
};

pub struct XacroProcessor {
    macros: MacroProcessor,
    properties: PropertyProcessor,
    conditions: ConditionProcessor,
    loops: LoopProcessor,
    includes: IncludeProcessor,
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

    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            includes: IncludeProcessor::new(),
            macros: MacroProcessor::new(),
            properties: PropertyProcessor::new(),
            conditions: ConditionProcessor::new(),
            loops: LoopProcessor::new(),
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
        self.run_impl(xml, path.as_ref())
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
        xml: xmltree::Element,
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
        if let Some(ns) = xml.namespaces.as_ref() {
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

        let xacro_ns: String = xml
            .namespaces
            .as_ref()
            .and_then(|ns| {
                // Try standard "xacro" prefix (already validated above)
                ns.get("xacro").map(|s| s.to_string()).or_else(|| {
                    // Fallback: find any prefix bound to a known xacro URI
                    Self::find_xacro_namespace_in_map(ns)
                })
            })
            .unwrap_or_default(); // Use empty string if no namespace declared (lazy checking)

        // Process features in order
        let xml = self.includes.process(xml, base_path, &xacro_ns)?;

        // The HashMap contains all properties for use by subsequent processors
        let (xml, properties) = self.properties.process(xml, &xacro_ns)?;

        // Pass properties to MacroProcessor so macros can reference global properties
        let xml = self.macros.process(xml, &properties, &xacro_ns)?;

        // Pass properties to ConditionProcessor for expression evaluation
        let xml = self.conditions.process(xml, &properties, &xacro_ns)?;
        let mut xml = self.loops.process(xml, &xacro_ns)?;

        // Final cleanup: check for unprocessed xacro:* elements and remove xacro namespace
        // Does both in a single recursive pass for efficiency
        Self::finalize_tree(&mut xml, &xacro_ns)?;

        XacroProcessor::serialize(xml)
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

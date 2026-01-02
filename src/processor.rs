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
        // Extract xacro namespace from document root
        // Strategy:
        // 1. Try standard "xacro" prefix (e.g., xmlns:xacro="...")
        // 2. Fallback: search for any prefix bound to a known xacro URI
        // This makes us more robust than Python xacro while maintaining compatibility
        let xacro_ns: String = xml
            .namespaces
            .as_ref()
            .and_then(|ns| {
                // First try standard "xacro" prefix
                ns.get("xacro").map(|s| s.to_string()).or_else(|| {
                    // Fallback: find any prefix bound to a known xacro URI
                    const KNOWN_XACRO_URIS: &[&str] = &[
                        "http://www.ros.org/wiki/xacro",
                        "http://ros.org/wiki/xacro",
                        "http://wiki.ros.org/xacro",
                        "http://www.ros.org/xacro",
                        "http://playerstage.sourceforge.net/gazebo/xmlschema/#xacro",
                    ];

                    ns.0.values().find(|uri| KNOWN_XACRO_URIS.contains(&uri.as_str())).map(|s| s.to_string())
                })
            })
            .ok_or_else(|| {
                XacroError::MissingNamespace(
                    "No xacro namespace declared. Expected xmlns:xacro=\"...\" or any prefix bound to a known xacro URI in root element"
                        .into(),
                )
            })?;

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
        if element.namespace.as_deref() == Some(xacro_ns) {
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

        // Remove xacro namespace declaration
        // Find and remove whichever prefix is bound to the xacro namespace URI
        // This handles both standard (xmlns:xacro="...") and non-standard (xmlns:foo="...") prefixes
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

        // Recursively process children
        for child in &mut element.children {
            if let Some(child_elem) = child.as_mut_element() {
                Self::finalize_tree(child_elem, xacro_ns)?;
            }
        }

        Ok(())
    }
}

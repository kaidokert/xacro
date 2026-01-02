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
        // Process features in order
        let xml = self.includes.process(xml, base_path)?;

        // The HashMap contains all properties for use by subsequent processors
        let (xml, properties) = self.properties.process(xml)?;

        // Pass properties to MacroProcessor so macros can reference global properties
        let xml = self.macros.process(xml, &properties)?;

        // Pass properties to ConditionProcessor for expression evaluation
        let xml = self.conditions.process(xml, &properties)?;
        let mut xml = self.loops.process(xml)?;

        // Final cleanup: check for unprocessed xacro:* elements and remove xacro namespace
        // Does both in a single recursive pass for efficiency
        Self::finalize_tree(&mut xml)?;

        XacroProcessor::serialize(xml)
    }

    fn finalize_tree(element: &mut xmltree::Element) -> Result<(), XacroError> {
        // Check if this element has xacro: prefix (indicates unimplemented feature)
        if let Some(ref prefix) = element.prefix {
            if prefix == "xacro" {
                return Err(XacroError::UnimplementedFeature(format!(
                    "<xacro:{}>\n\
                         This element was not processed. Either:\n\
                         1. The feature is not implemented yet (e.g., xacro:arg, xacro:element)\n\
                         2. There's a bug in the processor\n",
                    element.name
                )));
            }
        }

        // Remove xacro namespace declaration
        // Keep all other namespaces (gazebo, ignition, etc.)
        if let Some(ref mut ns) = element.namespaces {
            ns.0.remove("xacro");
        }

        // Recursively process children
        for child in &mut element.children {
            if let Some(child_elem) = child.as_mut_element() {
                Self::finalize_tree(child_elem)?;
            }
        }

        Ok(())
    }
}

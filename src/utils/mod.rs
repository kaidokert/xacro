pub mod eval;
pub mod lexer;
pub mod xml;

use crate::{error::XacroError, XacroProcessor};

impl XacroProcessor {
    pub(crate) fn parse_file<P: AsRef<std::path::Path>>(
        path: P
    ) -> Result<xmltree::Element, XacroError> {
        let file = std::fs::File::open(path)?;
        Ok(xmltree::Element::parse(file)?)
    }

    pub(crate) fn serialize(xml: &xmltree::Element) -> Result<String, XacroError> {
        let mut writer = Vec::new();
        // Apply pretty-printing to match Python xacro's output formatting
        // Python uses doc.toprettyxml(indent='  ') which formats with 2-space indent
        // Python uses self-closing <tag/> for leaf elements
        xml.write_with_config(
            &mut writer,
            xmltree::EmitterConfig::new()
                .perform_indent(true)
                .indent_string("  ")
                .pad_self_closing(false), // Use <tag/> not <tag />
        )?;
        Ok(String::from_utf8(writer)?)
    }
}

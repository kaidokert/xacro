pub mod document;
pub mod eval;
pub mod lexer;
pub mod xml;

use crate::{error::XacroError, XacroProcessor};
use document::XacroDocument;

impl XacroProcessor {
    pub(crate) fn parse_file<P: AsRef<std::path::Path>>(
        path: P
    ) -> Result<XacroDocument, XacroError> {
        let file = std::fs::File::open(path)?;
        XacroDocument::parse(file)
    }

    pub(crate) fn serialize(doc: &XacroDocument) -> Result<String, XacroError> {
        let mut writer = Vec::new();
        doc.write(&mut writer)?;
        Ok(String::from_utf8(writer)?)
    }
}

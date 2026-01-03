pub mod eval;
pub mod lexer;
pub mod xml;

use crate::{error::XacroError, XacroProcessor};
use std::collections::HashMap;

#[cfg(test)]
use similar::{ChangeTag, TextDiff};

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

    /// Helper for debug/logging that never panics
    ///
    /// Returns serialized XML or error message if serialization fails.
    /// Use this in debug!() statements to avoid panics in logging code.
    pub(crate) fn serialize_or_err(xml: &xmltree::Element) -> String {
        Self::serialize(xml).unwrap_or_else(|e| format!("<xacro serialize error: {}>", e))
    }
}

#[cfg(test)]
pub(crate) fn print_diff(
    expected: &str,
    actual: &str,
) {
    let diff = TextDiff::from_lines(expected, actual);

    for change in diff.iter_all_changes() {
        let sign = match change.tag() {
            ChangeTag::Delete => "-",
            ChangeTag::Insert => "+",
            ChangeTag::Equal => " ",
        };
        print!("{}{}", sign, change);
    }
}

pub(crate) fn pretty_print_hashmap<K, V>(map: &HashMap<K, V>) -> String
where
    K: core::fmt::Debug + core::cmp::Ord,
    V: core::fmt::Debug,
{
    let mut entries: Vec<_> = map.iter().collect();
    entries.sort_by(|a, b| a.0.cmp(b.0));

    let mut output = String::from("{\n");
    for (key, value) in entries {
        output.push_str(&format!("  {:?}: {:?},\n", key, value));
    }
    output.push('}');
    output
}

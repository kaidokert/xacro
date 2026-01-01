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

    pub(crate) fn serialize(xml: xmltree::Element) -> Result<String, XacroError> {
        let mut writer = Vec::new();
        xml.write(&mut writer)?;
        Ok(String::from_utf8(writer)?)
    }
}

pub(crate) fn pretty_print_xml(xml: &xmltree::Element) -> String {
    let mut writer = Vec::new();
    xml.write_with_config(
        &mut writer,
        xmltree::EmitterConfig::new()
            .perform_indent(true)
            .indent_string("  "),
    )
    .unwrap();
    String::from_utf8(writer).unwrap()
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

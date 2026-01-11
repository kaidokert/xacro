//! XacroDocument - Preserves processing instructions and comments outside root element
//!

use crate::error::XacroError;
use std::io::Write;
use xmltree::{Element, XMLNode};

/// Represents a complete xacro document with preamble and root element
///
/// The preamble contains all nodes that appear before the root element:
/// - Processing instructions (<?xml-model?>, <?xml-stylesheet?>, etc.)
/// - Comments
///
/// This structure allows xacro to be "transparent" - preserving all
/// document-level constructs while only processing xacro-namespaced elements.
///
/// # Note on DOCTYPE
///
/// DOCTYPE declarations are NOT preserved because xmltree silently discards them
/// This is accepted as a known limitation.
///
/// # Example
///
/// ```rust,ignore
/// let xml = r#"<?xml version="1.0"?>
/// <?xml-model href="schema.xsd"?>
/// <!-- User comment -->
/// <robot name="test"/>"#;
///
/// let doc = XacroDocument::parse(xml.as_bytes())?;
/// // doc.preamble contains the PI and comment
/// // doc.root is the <robot> element
/// ```
#[derive(Debug)]
pub struct XacroDocument {
    /// Nodes that appear before the root element (PIs, comments)
    /// Preserves order to maintain document structure
    pub preamble: Vec<XMLNode>,

    /// The root XML element (contains xacro directives to process)
    pub root: Element,
}

impl XacroDocument {
    /// Parse an XML document preserving preamble nodes
    ///
    /// Uses `Element::parse_all()` to capture all nodes before the root element.
    /// The first Element node becomes the root; all non-element nodes before it
    /// go into the preamble.
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - XML is malformed
    /// - No root element found
    /// - Multiple root elements found
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let xml = r#"<?xml version="1.0"?>
    /// <?xml-model href="schema.xsd"?>
    /// <robot name="test"/>"#;
    ///
    /// let doc = XacroDocument::parse(xml.as_bytes())?;
    /// assert_eq!(doc.preamble.len(), 1); // One PI
    /// assert_eq!(doc.root.name, "robot");
    /// ```
    pub fn parse<R: std::io::Read>(reader: R) -> Result<Self, XacroError> {
        let all_nodes = Element::parse_all(reader)?;

        let mut preamble = Vec::new();
        let mut root = None;

        for node in all_nodes {
            match node {
                XMLNode::Element(elem) => {
                    // First element is the root
                    if root.is_none() {
                        root = Some(elem);
                    } else {
                        // Multiple root elements - invalid XML
                        return Err(XacroError::InvalidXml(
                            "Document has multiple root elements".into(),
                        ));
                    }
                }
                // All non-element nodes before root go in preamble
                _ => {
                    if root.is_none() {
                        preamble.push(node);
                    }
                    // Note: Nodes AFTER root are discarded (invalid XML anyway)
                }
            }
        }

        let root =
            root.ok_or_else(|| XacroError::InvalidXml("Document has no root element".into()))?;

        Ok(XacroDocument { preamble, root })
    }

    /// Write the document with preamble preserved
    ///
    /// Output format:
    /// 1. XML declaration: `<?xml version="1.0" ?>`
    /// 2. Preamble nodes (PIs, comments) in order
    /// 3. Root element
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - I/O write fails
    /// - XML serialization fails
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let doc = XacroDocument::parse(xml.as_bytes())?;
    /// let mut output = Vec::new();
    /// doc.write(&mut output)?;
    /// let output_str = String::from_utf8(output)?;
    /// ```
    pub fn write<W: Write>(
        &self,
        writer: &mut W,
    ) -> Result<(), XacroError> {
        // 1. Write XML declaration
        writeln!(writer, "<?xml version=\"1.0\" ?>")?;

        // 2. Write preamble (PIs, comments)
        for node in &self.preamble {
            match node {
                XMLNode::ProcessingInstruction(target, data) => {
                    // Handle empty data (no trailing space)
                    if let Some(d) = data.as_ref().filter(|s| !s.is_empty()) {
                        writeln!(writer, "<?{} {}?>", target, d)?;
                    } else {
                        writeln!(writer, "<?{}?>", target)?;
                    }
                }
                XMLNode::Comment(comment) => {
                    writeln!(writer, "<!--{}-->", comment)?;
                }
                // Other node types in preamble are unusual but handle gracefully
                XMLNode::Text(text) => {
                    // Text outside root is usually just whitespace, preserve it
                    write!(writer, "{}", text)?;
                }
                XMLNode::CData(_) | XMLNode::Element(_) => {
                    // CDATA or Element in preamble is invalid XML, but don't crash
                    // Element would have been captured as root, so shouldn't reach here
                }
            }
        }

        // 3. Write root element
        self.root.write_with_config(
            writer,
            xmltree::EmitterConfig::new()
                .perform_indent(true)
                .write_document_declaration(false) // Already wrote it
                .indent_string("  ")
                .pad_self_closing(false), // Use <tag/> not <tag />
        )?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_with_pi() {
        let xml = r#"<?xml version="1.0"?>
<?xml-model href="schema.xsd"?>
<robot name="test"/>"#;

        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();

        // Verify preamble captured
        assert_eq!(doc.preamble.len(), 1);
        match &doc.preamble[0] {
            XMLNode::ProcessingInstruction(target, data) => {
                assert_eq!(target, "xml-model");
                assert!(data.as_ref().unwrap().contains("schema.xsd"));
            }
            _ => panic!("Expected ProcessingInstruction in preamble"),
        }

        // Verify root captured
        assert_eq!(doc.root.name, "robot");
    }

    #[test]
    fn test_parse_no_preamble() {
        let xml = r#"<?xml version="1.0"?>
<robot name="test"/>"#;

        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();

        // Empty preamble
        assert!(doc.preamble.is_empty());

        // Root still parsed
        assert_eq!(doc.root.name, "robot");
    }

    #[test]
    fn test_parse_with_comment() {
        let xml = r#"<?xml version="1.0"?>
<!-- User comment -->
<robot name="test"/>"#;

        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();

        // Comment in preamble
        assert_eq!(doc.preamble.len(), 1);
        match &doc.preamble[0] {
            XMLNode::Comment(text) => {
                assert_eq!(text.trim(), "User comment");
            }
            _ => panic!("Expected Comment in preamble"),
        }
    }

    #[test]
    fn test_parse_multiple_preamble_nodes() {
        let xml = r#"<?xml version="1.0"?>
<!-- Comment 1 -->
<?pi1 data1?>
<!-- Comment 2 -->
<?pi2 data2?>
<robot name="test"/>"#;

        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();

        // Should have 4 preamble nodes
        assert_eq!(doc.preamble.len(), 4);

        // Verify order
        assert!(matches!(doc.preamble[0], XMLNode::Comment(_)));
        assert!(matches!(
            doc.preamble[1],
            XMLNode::ProcessingInstruction(ref t, _) if t == "pi1"
        ));
        assert!(matches!(doc.preamble[2], XMLNode::Comment(_)));
        assert!(matches!(
            doc.preamble[3],
            XMLNode::ProcessingInstruction(ref t, _) if t == "pi2"
        ));
    }

    #[test]
    fn test_parse_multiple_roots_error() {
        let xml = r#"<?xml version="1.0"?>
<robot name="test1"/>
<robot name="test2"/>"#;

        let result = XacroDocument::parse(xml.as_bytes());

        // Should error
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("multiple root"),
            "Error should mention multiple roots, got: {}",
            err_msg
        );
    }

    #[test]
    fn test_parse_no_root_error() {
        let xml = r#"<?xml version="1.0"?>
<?xml-model href="schema.xsd"?>
<!-- Just a comment -->"#;

        let result = XacroDocument::parse(xml.as_bytes());

        // Should error
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("no root"),
            "Error should mention no root, got: {}",
            err_msg
        );
    }

    #[test]
    fn test_write_preserves_pi() {
        let xml = r#"<?xml version="1.0"?>
<?xml-model href="schema.xsd"?>
<robot name="test"/>"#;

        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();

        let mut output = Vec::new();
        doc.write(&mut output).unwrap();
        let output_str = String::from_utf8(output).unwrap();

        // PI preserved in output
        assert!(
            output_str.contains(r#"<?xml-model href="schema.xsd"?>"#),
            "PI should be preserved in output"
        );
        assert!(
            output_str.contains(r#"<robot name="test""#),
            "Root element should be in output"
        );
    }

    #[test]
    fn test_write_preserves_order() {
        let xml = r#"<?xml version="1.0"?>
<!-- First comment -->
<?xml-model href="schema.xsd"?>
<!-- Second comment -->
<robot name="test"/>"#;

        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();

        let mut output = Vec::new();
        doc.write(&mut output).unwrap();
        let output_str = String::from_utf8(output).unwrap();

        // Find positions
        let comment1_pos = output_str
            .find("<!-- First comment -->")
            .expect("First comment should be in output");
        let pi_pos = output_str
            .find("<?xml-model")
            .expect("PI should be in output");
        let comment2_pos = output_str
            .find("<!-- Second comment -->")
            .expect("Second comment should be in output");

        // Verify order preserved
        assert!(comment1_pos < pi_pos, "First comment should come before PI");
        assert!(
            pi_pos < comment2_pos,
            "PI should come before second comment"
        );
    }

    #[test]
    fn test_write_empty_pi_data() {
        let xml = r#"<?xml version="1.0"?>
<?target?>
<robot name="test"/>"#;

        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();

        let mut output = Vec::new();
        doc.write(&mut output).unwrap();
        let output_str = String::from_utf8(output).unwrap();

        // Should preserve PI without extra space
        assert!(
            output_str.contains(r#"<?target?>"#),
            "Empty PI should not have trailing space"
        );
    }
}

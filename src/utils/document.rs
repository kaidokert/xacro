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
                // All other non-element nodes
                node => {
                    if root.is_none() {
                        // Nodes before root go in preamble (PIs, comments, text)
                        // CDATA before root is invalid per XML spec, but xmltree rejects it at parse time
                        preamble.push(node);
                    } else {
                        // Nodes after root - warn if significant
                        match &node {
                            XMLNode::Text(text) if !text.trim().is_empty() => {
                                log::warn!(
                                    "Non-whitespace text found after root element: {:?}",
                                    text.trim()
                                );
                            }
                            XMLNode::Text(_) => { /* ignore whitespace */ }
                            XMLNode::CData(_) => {
                                log::warn!("CDATA section found after root element, discarding");
                            }
                            _ => {
                                log::warn!("Unexpected node after root element: {:?}", node);
                            }
                        }
                    }
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
                    // Validate PI target and data per XML spec
                    if target.eq_ignore_ascii_case("xml") {
                        return Err(XacroError::InvalidXml(
                            "Processing instruction target cannot be 'xml' (reserved)".into(),
                        ));
                    }

                    // Handle empty data (no trailing space)
                    if let Some(d) = data.as_ref().filter(|s| !s.is_empty()) {
                        if d.contains("?>") {
                            return Err(XacroError::InvalidXml(
                                "Processing instruction data cannot contain '?>'".into(),
                            ));
                        }
                        writeln!(writer, "<?{} {}?>", target, d)?;
                    } else {
                        writeln!(writer, "<?{}?>", target)?;
                    }
                }
                XMLNode::Comment(comment) => {
                    // Validate comment content per XML spec
                    if comment.contains("--") || comment.ends_with('-') {
                        return Err(XacroError::InvalidXml(
                            "Comments cannot contain '--' or end with '-'".into(),
                        ));
                    }
                    writeln!(writer, "<!--{}-->", comment)?;
                }
                // Other node types in preamble are unusual but handle gracefully
                XMLNode::Text(text) => {
                    // Text outside root is usually just whitespace, preserve it
                    write!(writer, "{}", text)?;
                }
                XMLNode::CData(_) | XMLNode::Element(_) => {
                    // These are invalid in the preamble and cannot occur here.
                    // Element is always captured as root, CDATA is never added to preamble.
                    unreachable!(
                        "Invalid node type in preamble: CData and Element are not allowed."
                    );
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
    fn test_parse_cdata_in_preamble_error() {
        let xml = r#"<?xml version="1.0"?>
<![CDATA[data before root]]>
<robot name="test"/>"#;

        let result = XacroDocument::parse(xml.as_bytes());

        // Should error - CDATA not allowed outside root element (xmltree rejects at parse time)
        assert!(result.is_err(), "CDATA before root should be rejected");
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

    #[test]
    fn test_write_invalid_pi_target_xml() {
        // Create a mock document with invalid PI target 'xml'
        let mut doc = XacroDocument {
            preamble: vec![XMLNode::ProcessingInstruction(
                "xml".to_string(),
                Some("data=\"invalid\"".to_string()),
            )],
            root: xmltree::Element::new("robot"),
        };
        doc.root
            .attributes
            .insert(xmltree::AttributeName::local("name"), "test".to_string());

        let mut output = Vec::new();
        let result = doc.write(&mut output);

        // Should error - PI target cannot be 'xml' (reserved)
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("Processing instruction target cannot be 'xml'"));
    }

    #[test]
    fn test_write_invalid_pi_data_contains_close() {
        // Create a mock document with invalid PI data
        let mut doc = XacroDocument {
            preamble: vec![XMLNode::ProcessingInstruction(
                "target".to_string(),
                Some("data with ?> inside".to_string()),
            )],
            root: xmltree::Element::new("robot"),
        };
        doc.root
            .attributes
            .insert(xmltree::AttributeName::local("name"), "test".to_string());

        let mut output = Vec::new();
        let result = doc.write(&mut output);

        // Should error - PI data cannot contain '?>'
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("Processing instruction data cannot contain '?>'"));
    }

    #[test]
    fn test_write_invalid_comment_contains_double_dash() {
        // Create a mock document with invalid comment
        let mut doc = XacroDocument {
            preamble: vec![XMLNode::Comment(" Invalid -- comment ".to_string())],
            root: xmltree::Element::new("robot"),
        };
        doc.root
            .attributes
            .insert(xmltree::AttributeName::local("name"), "test".to_string());

        let mut output = Vec::new();
        let result = doc.write(&mut output);

        // Should error - Comments cannot contain '--'
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("Comments cannot contain '--'"));
    }

    #[test]
    fn test_write_invalid_comment_ends_with_dash() {
        // Create a mock document with invalid comment
        let mut doc = XacroDocument {
            preamble: vec![XMLNode::Comment(" Invalid comment-".to_string())],
            root: xmltree::Element::new("robot"),
        };
        doc.root
            .attributes
            .insert(xmltree::AttributeName::local("name"), "test".to_string());

        let mut output = Vec::new();
        let result = doc.write(&mut output);

        // Should error - Comments cannot end with '-'
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("Comments cannot contain '--' or end with '-'"));
    }

    #[test]
    fn test_doctype_not_preserved() {
        let xml = r#"<?xml version="1.0"?>
<!DOCTYPE robot SYSTEM "robot.dtd">
<robot name="test"/>"#;

        // DOCTYPE is silently discarded by xmltree (known limitation)
        let doc = XacroDocument::parse(xml.as_bytes()).unwrap();
        let mut output = Vec::new();
        doc.write(&mut output).unwrap();
        let output_str = String::from_utf8(output).unwrap();

        // DOCTYPE should NOT be preserved (known xmltree limitation)
        assert!(
            !output_str.contains("<!DOCTYPE"),
            "DOCTYPE is a known limitation and should not be preserved"
        );
    }
}

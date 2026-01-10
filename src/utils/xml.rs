use crate::error::XacroError;
use std::io::Write;
use xml::escape::escape_str_pcdata;
use xmltree::{Element, XMLNode};

/// The standard ROS xacro namespace URI
///
/// This is the most common xacro namespace URI. The xacro processor dynamically
/// extracts the actual namespace from each document's root element (xmlns:xacro="..."),
/// so this constant serves as a reference value for external crates or testing.
pub const XACRO_NAMESPACE: &str = "http://www.ros.org/wiki/xacro";

/// Known xacro namespace URIs used in the wild
///
/// Used for fallback namespace detection and validation. The processor will recognize
/// any of these URIs as valid xacro namespaces, allowing compatibility with different
/// namespace variants used across the ROS ecosystem.
pub const KNOWN_XACRO_URIS: &[&str] = &[
    "http://www.ros.org/wiki/xacro",
    "http://ros.org/wiki/xacro",
    "http://wiki.ros.org/xacro",
    "http://www.ros.org/xacro",
    "http://playerstage.sourceforge.net/gazebo/xmlschema/#xacro",
];

/// Search namespace map for any prefix bound to a known xacro URI
///
/// Used as fallback when the standard "xacro" prefix is not found, allowing
/// documents with non-standard prefixes (e.g., xmlns:x="...") to still be recognized.
pub fn find_xacro_namespace_in_map(ns: &xmltree::Namespace) -> Option<String> {
    ns.0.values()
        .find(|uri| KNOWN_XACRO_URIS.contains(&uri.as_str()))
        .map(|s| s.to_string())
}

/// Check if a namespace URI is a known xacro namespace
///
/// Returns true if the given URI matches any of the known xacro namespace URIs.
pub fn is_known_xacro_uri(uri: &str) -> bool {
    KNOWN_XACRO_URIS.contains(&uri)
}

/// Extract and validate xacro namespace from an XML element
///
/// This function is used for both the root document and included files
/// to ensure each file's namespace declaration is properly handled.
///
/// Returns the xacro namespace URI, or empty string if no xacro namespace is declared.
///
/// # Errors
/// Returns an error if the "xacro" prefix is bound to an unknown/invalid URI (likely a typo).
pub fn extract_xacro_namespace(element: &Element) -> Result<String, XacroError> {
    // Validate xacro namespace and extract it
    // First check if "xacro" prefix exists but is bound to invalid URI (catch typos)
    if let Some(ns) = element.namespaces.as_ref() {
        if let Some(xacro_uri) = ns.get("xacro") {
            let uri_str: &str = xacro_uri;
            if !KNOWN_XACRO_URIS.contains(&uri_str) {
                return Err(XacroError::MissingNamespace(format!(
                    "The 'xacro' prefix is bound to an unknown URI: '{}'. \
                     This might be a typo. Known xacro URIs are: {}",
                    xacro_uri,
                    KNOWN_XACRO_URIS.join(", ")
                )));
            }
        }
    }

    let xacro_ns: String = element
        .namespaces
        .as_ref()
        .and_then(|ns| {
            ns.get("xacro")
                .map(|s| s.to_string())
                .or_else(|| find_xacro_namespace_in_map(ns))
        })
        .unwrap_or_default();

    Ok(xacro_ns)
}

/// Check if an element is a xacro element with the given tag name
///
/// Recognizes elements in ANY known xacro namespace URI (not just the context's specific URI),
/// but ONLY if a xacro namespace was declared on the document root. This allows mixing different
/// valid xacro URIs within the same document tree (e.g., when included files use different
/// namespace variants), while still enforcing that the root declares a xacro namespace.
///
/// This checks the element's resolved namespace URI (not the prefix), which is
/// the correct way to identify xacro elements since the prefix can be aliased
/// (e.g., xmlns:x="http://www.ros.org/wiki/xacro").
///
/// # Arguments
/// * `element` - The XML element to check
/// * `tag_name` - The local tag name (e.g., "if", "unless", "property", "macro")
/// * `xacro_ns` - The xacro namespace URI from the document root (empty if not declared)
///
/// # Returns
/// `true` if the element is in any known xacro namespace with the given tag name,
/// and the root declared a xacro namespace
///
/// # Examples
/// ```
/// use xmltree::Element;
/// use xacro::utils::xml::is_xacro_element;
///
/// let xml = r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
///     <xacro:if value="true">
///         <link name="base"/>
///     </xacro:if>
/// </robot>"#;
///
/// let root = Element::parse(xml.as_bytes()).unwrap();
/// let xacro_ns = root.namespaces.as_ref().unwrap().get("xacro").unwrap();
/// let if_elem = root.get_child("if").unwrap();
/// assert!(is_xacro_element(if_elem, "if", xacro_ns));
/// ```
pub fn is_xacro_element(
    element: &Element,
    tag_name: &str,
    xacro_ns: &str,
) -> bool {
    element.name == tag_name
        && !xacro_ns.is_empty()
        && element
            .namespace
            .as_deref()
            .is_some_and(|ns| ns == xacro_ns || is_known_xacro_uri(ns))
}

/// Serialize a list of XMLNodes to an XML string
///
/// This function preserves:
/// - Element structure and attributes
/// - Text content (with XML escaping)
/// - Comments
/// - Namespace prefixes and declarations
/// - CDATA sections
/// - Processing instructions
///
/// Used by lazy properties to store XML content as strings during property
/// definition, which are later parsed and expanded at insertion time.
///
/// # Example
/// ```rust,ignore
/// let nodes = vec![
///     XMLNode::Element(link_element),
///     XMLNode::Comment("comment".to_string()),
/// ];
/// let xml_string = serialize_nodes(&nodes)?;
/// ```
pub fn serialize_nodes(nodes: &[XMLNode]) -> Result<String, XacroError> {
    let mut buffer = Vec::new();

    for node in nodes {
        match node {
            XMLNode::Element(elem) => {
                // Use xmltree's write_with_config to control output format
                // Disable XML declaration (write_document_declaration: false)
                elem.write_with_config(
                    &mut buffer,
                    xmltree::EmitterConfig::new()
                        .perform_indent(false) // No extra whitespace
                        .write_document_declaration(false) // No <?xml...?>
                        .pad_self_closing(false), // Use <tag/> not <tag />
                )?;
            }
            XMLNode::Text(text) => {
                // Escape XML special characters (<, >, &) for PCDATA
                let escaped = escape_str_pcdata(text);
                buffer.extend_from_slice(escaped.as_bytes());
            }
            XMLNode::Comment(comment) => {
                // Validate comment content per XML spec
                if comment.contains("--") || comment.ends_with('-') {
                    return Err(XacroError::InvalidXml(
                        "Comments cannot contain '--' or end with '-'".into(),
                    ));
                }
                write!(&mut buffer, "<!--{}-->", comment)?;
            }
            XMLNode::CData(data) => {
                // Validate CDATA content per XML spec
                if data.contains("]]>") {
                    return Err(XacroError::InvalidXml(
                        "CDATA sections cannot contain ']]>'".into(),
                    ));
                }
                write!(&mut buffer, "<![CDATA[{}]]>", data)?;
            }
            XMLNode::ProcessingInstruction(target, data) => {
                // Validate PI target per XML spec
                if target.eq_ignore_ascii_case("xml") {
                    return Err(XacroError::InvalidXml(
                        "Processing instruction target cannot be 'xml' (reserved)".into(),
                    ));
                }

                // Validate and write PI data per XML spec
                // Treat empty string as no data to avoid trailing space
                if let Some(d) = data.as_ref().filter(|s| !s.is_empty()) {
                    if d.contains("?>") {
                        return Err(XacroError::InvalidXml(
                            "Processing instruction data cannot contain '?>'".into(),
                        ));
                    }
                    write!(&mut buffer, "<?{} {}?>", target, d)?;
                } else {
                    write!(&mut buffer, "<?{}?>", target)?;
                }
            }
        }
    }

    String::from_utf8(buffer).map_err(XacroError::Utf8)
}

/// Parse an XML fragment (potentially multiple top-level nodes) into XMLNodes
///
/// This function handles fragments with multiple root elements by wrapping them
/// in a temporary container element. The container is discarded and only the
/// children are returned.
///
/// The dummy root includes the xacro namespace declaration to ensure nested
/// xacro directives can be recognized during expansion.
///
/// # Edge cases
/// - Empty string → returns empty Vec
/// - Single element → returns Vec with one node
/// - Multiple elements → returns Vec with all nodes
/// - Mixed content (text + elements) → returns all nodes preserving order
///
/// # Example
/// ```rust,ignore
/// let fragment = r#"<link name="a"/><link name="b"/>"#;
/// let nodes = parse_xml_fragment(fragment)?;
/// assert_eq!(nodes.len(), 2);
/// ```
pub fn parse_xml_fragment(fragment: &str) -> Result<Vec<XMLNode>, XacroError> {
    // Edge case: empty string
    let trimmed = fragment.trim();
    if trimmed.is_empty() {
        return Ok(Vec::new());
    }

    // Wrap in dummy root to allow multiple top-level elements
    // Include xacro namespace so nested directives are recognized
    let wrapped = format!(
        r#"<xacro_dummy_root xmlns:xacro="{}">{}</xacro_dummy_root>"#,
        XACRO_NAMESPACE, fragment
    );

    let root = Element::parse(wrapped.as_bytes())?;

    Ok(root.children)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serialize_nodes_text_escaping() {
        // Create text nodes with special XML characters
        let nodes = vec![
            XMLNode::Text("Text with <angle> brackets".to_string()),
            XMLNode::Text(" & ampersands".to_string()),
        ];

        let serialized = serialize_nodes(&nodes).unwrap();

        // Verify special characters are escaped
        assert!(
            serialized.contains("&lt;"),
            "Expected escaped '<', got: {}",
            serialized
        );
        assert!(
            serialized.contains("&gt;"),
            "Expected escaped '>', got: {}",
            serialized
        );
        assert!(
            serialized.contains("&amp;"),
            "Expected escaped '&', got: {}",
            serialized
        );

        // Verify the full escaped content
        assert_eq!(
            serialized,
            "Text with &lt;angle&gt; brackets &amp; ampersands"
        );
    }

    #[test]
    fn test_serialize_nodes_no_escaping_needed() {
        // Text without special characters should not be modified
        let nodes = vec![XMLNode::Text("Normal text 123".to_string())];

        let serialized = serialize_nodes(&nodes).unwrap();

        assert_eq!(serialized, "Normal text 123");
    }

    #[test]
    fn test_serialize_comment_invalid_double_dash() {
        let nodes = vec![XMLNode::Comment("This -- is invalid".to_string())];

        let result = serialize_nodes(&nodes);

        assert!(result.is_err(), "Should reject comment containing '--'");
        assert!(
            result.unwrap_err().to_string().contains("--"),
            "Error should mention '--'"
        );
    }

    #[test]
    fn test_serialize_comment_invalid_trailing_dash() {
        let nodes = vec![XMLNode::Comment("Trailing dash-".to_string())];

        let result = serialize_nodes(&nodes);

        assert!(result.is_err(), "Should reject comment ending with '-'");
    }

    #[test]
    fn test_serialize_comment_valid() {
        let nodes = vec![XMLNode::Comment("Valid comment".to_string())];

        let serialized = serialize_nodes(&nodes).unwrap();

        assert_eq!(serialized, "<!--Valid comment-->");
    }

    #[test]
    fn test_serialize_cdata_invalid() {
        let nodes = vec![XMLNode::CData("Invalid ]]> sequence".to_string())];

        let result = serialize_nodes(&nodes);

        assert!(result.is_err(), "Should reject CDATA containing ']]>'");
        assert!(
            result.unwrap_err().to_string().contains("]]>"),
            "Error should mention ']]>'"
        );
    }

    #[test]
    fn test_serialize_cdata_valid() {
        let nodes = vec![XMLNode::CData("Valid <raw> content".to_string())];

        let serialized = serialize_nodes(&nodes).unwrap();

        assert_eq!(serialized, "<![CDATA[Valid <raw> content]]>");
    }

    #[test]
    fn test_serialize_pi_invalid_target_xml() {
        let nodes = vec![XMLNode::ProcessingInstruction(
            "xml".to_string(),
            Some("encoding=\"UTF-8\"".to_string()),
        )];

        let result = serialize_nodes(&nodes);

        assert!(result.is_err(), "Should reject PI target 'xml' (reserved)");
        assert!(
            result.unwrap_err().to_string().contains("xml"),
            "Error should mention 'xml'"
        );
    }

    #[test]
    fn test_serialize_pi_invalid_target_xml_case_insensitive() {
        let nodes = vec![XMLNode::ProcessingInstruction(
            "XmL".to_string(),
            Some("data".to_string()),
        )];

        let result = serialize_nodes(&nodes);

        assert!(
            result.is_err(),
            "Should reject PI target 'XmL' (case-insensitive)"
        );
    }

    #[test]
    fn test_serialize_pi_invalid_data() {
        let nodes = vec![XMLNode::ProcessingInstruction(
            "target".to_string(),
            Some("Invalid ?> sequence".to_string()),
        )];

        let result = serialize_nodes(&nodes);

        assert!(result.is_err(), "Should reject PI data containing '?>'");
        assert!(
            result.unwrap_err().to_string().contains("?>"),
            "Error should mention '?>'"
        );
    }

    #[test]
    fn test_serialize_pi_valid_with_data() {
        let nodes = vec![XMLNode::ProcessingInstruction(
            "target".to_string(),
            Some("instruction data".to_string()),
        )];

        let serialized = serialize_nodes(&nodes).unwrap();

        assert_eq!(serialized, "<?target instruction data?>");
    }

    #[test]
    fn test_serialize_pi_valid_no_data() {
        let nodes = vec![XMLNode::ProcessingInstruction("target".to_string(), None)];

        let serialized = serialize_nodes(&nodes).unwrap();

        assert_eq!(serialized, "<?target?>");
    }

    #[test]
    fn test_serialize_pi_empty_data_treated_as_none() {
        let nodes = vec![XMLNode::ProcessingInstruction(
            "target".to_string(),
            Some("".to_string()),
        )];

        let serialized = serialize_nodes(&nodes).unwrap();

        // Empty string should be treated as no data (no trailing space)
        assert_eq!(serialized, "<?target?>");
    }

    #[test]
    fn test_parse_xml_fragment_empty() {
        let result = parse_xml_fragment("").unwrap();
        assert!(result.is_empty(), "Empty string should return empty vec");
    }

    #[test]
    fn test_parse_xml_fragment_whitespace_only() {
        let result = parse_xml_fragment("   \n\t  ").unwrap();
        assert!(
            result.is_empty(),
            "Whitespace-only string should return empty vec"
        );
    }

    #[test]
    fn test_parse_xml_fragment_single_element() {
        let result = parse_xml_fragment("<link name=\"test\"/>").unwrap();
        assert_eq!(result.len(), 1, "Should parse single element");

        let elem = match &result[0] {
            XMLNode::Element(e) => e,
            _ => panic!("Expected element node"),
        };
        assert_eq!(elem.name, "link");
        assert_eq!(
            elem.attributes.get(&xmltree::AttributeName::local("name")),
            Some(&"test".to_string())
        );
    }

    #[test]
    fn test_parse_xml_fragment_multiple_elements() {
        let result = parse_xml_fragment("<link name=\"a\"/><link name=\"b\"/>").unwrap();
        assert_eq!(result.len(), 2, "Should parse multiple elements");

        let elem1 = match &result[0] {
            XMLNode::Element(e) => e,
            _ => panic!("Expected element node"),
        };
        let elem2 = match &result[1] {
            XMLNode::Element(e) => e,
            _ => panic!("Expected element node"),
        };

        assert_eq!(
            elem1.attributes.get(&xmltree::AttributeName::local("name")),
            Some(&"a".to_string())
        );
        assert_eq!(
            elem2.attributes.get(&xmltree::AttributeName::local("name")),
            Some(&"b".to_string())
        );
    }

    #[test]
    fn test_parse_xml_fragment_mixed_content() {
        let result = parse_xml_fragment("Text before<elem/>Text after").unwrap();
        assert_eq!(result.len(), 3, "Should parse mixed content");

        match &result[0] {
            XMLNode::Text(t) => assert_eq!(t, "Text before"),
            _ => panic!("Expected text node"),
        }

        match &result[1] {
            XMLNode::Element(e) => assert_eq!(e.name, "elem"),
            _ => panic!("Expected element node"),
        }

        match &result[2] {
            XMLNode::Text(t) => assert_eq!(t, "Text after"),
            _ => panic!("Expected text node"),
        }
    }

    #[test]
    fn test_parse_xml_fragment_with_comment() {
        let result = parse_xml_fragment("<!-- comment --><elem/>").unwrap();
        assert_eq!(result.len(), 2, "Should parse comment and element");

        match &result[0] {
            XMLNode::Comment(c) => assert_eq!(c, " comment "),
            _ => panic!("Expected comment node"),
        }

        match &result[1] {
            XMLNode::Element(e) => assert_eq!(e.name, "elem"),
            _ => panic!("Expected element node"),
        }
    }

    #[test]
    fn test_parse_xml_fragment_with_cdata() {
        let result = parse_xml_fragment("<![CDATA[raw content]]><elem/>").unwrap();
        assert_eq!(result.len(), 2, "Should parse CDATA and element");

        match &result[0] {
            XMLNode::CData(d) => assert_eq!(d, "raw content"),
            _ => panic!("Expected CDATA node"),
        }

        match &result[1] {
            XMLNode::Element(e) => assert_eq!(e.name, "elem"),
            _ => panic!("Expected element node"),
        }
    }

    #[test]
    fn test_parse_xml_fragment_with_pi() {
        let result = parse_xml_fragment("<?target data?><elem/>").unwrap();
        assert_eq!(result.len(), 2, "Should parse PI and element");

        match &result[0] {
            XMLNode::ProcessingInstruction(t, d) => {
                assert_eq!(t, "target");
                assert_eq!(d.as_deref(), Some("data"));
            }
            _ => panic!("Expected PI node"),
        }

        match &result[1] {
            XMLNode::Element(e) => assert_eq!(e.name, "elem"),
            _ => panic!("Expected element node"),
        }
    }

    #[test]
    fn test_parse_xml_fragment_preserves_xacro_directives() {
        let result =
            parse_xml_fragment("<xacro:if value=\"true\"><link name=\"test\"/></xacro:if>")
                .unwrap();
        assert_eq!(result.len(), 1, "Should parse xacro directive");

        let elem = match &result[0] {
            XMLNode::Element(e) => e,
            _ => panic!("Expected element node"),
        };

        assert_eq!(elem.name, "if");
        assert_eq!(
            elem.namespace.as_deref(),
            Some(XACRO_NAMESPACE),
            "Should preserve xacro namespace"
        );
    }
}

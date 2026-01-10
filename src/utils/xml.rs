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
                // Validate PI content per XML spec
                if let Some(d) = data {
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
}

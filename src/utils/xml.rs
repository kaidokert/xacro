use crate::error::XacroError;
use xmltree::Element;

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
/// Uses dynamic namespace detection: the xacro_ns parameter should be extracted
/// from the document root's xmlns:xacro declaration. This makes us compatible with
/// all namespace URI variants in the wild without hardcoding a list.
///
/// This checks the element's resolved namespace URI (not the prefix), which is
/// the correct way to identify xacro elements since the prefix can be aliased
/// (e.g., xmlns:x="http://www.ros.org/wiki/xacro").
///
/// # Arguments
/// * `element` - The XML element to check
/// * `tag_name` - The local tag name (e.g., "if", "unless", "property", "macro")
/// * `xacro_ns` - The xacro namespace URI for this document (from xmlns:xacro="...")
///
/// # Returns
/// `true` if the element is in the xacro namespace with the given tag name
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
    element.name == tag_name && element.namespace.as_deref() == Some(xacro_ns)
}

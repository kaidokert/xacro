use xmltree::Element;

/// The standard ROS xacro namespace URI
///
/// This is the most common xacro namespace URI. The xacro processor dynamically
/// extracts the actual namespace from each document's root element (xmlns:xacro="..."),
/// so this constant serves as a reference value for external crates or testing.
pub const XACRO_NAMESPACE: &str = "http://www.ros.org/wiki/xacro";

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

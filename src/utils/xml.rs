use xmltree::Element;

/// The official ROS xacro namespace URI
pub const XACRO_NAMESPACE: &str = "http://www.ros.org/wiki/xacro";

/// Check if an element is a xacro element with the given tag name
///
/// This checks the element's resolved namespace URI (not the prefix),
/// which is the correct way to identify xacro elements since the prefix
/// can be aliased to anything (e.g., xmlns:x="http://www.ros.org/wiki/xacro").
///
/// # Arguments
/// * `element` - The XML element to check
/// * `tag_name` - The local tag name (e.g., "if", "unless", "property", "macro")
///
/// # Returns
/// `true` if the element is in the xacro namespace with the given tag name
///
/// # Examples
/// ```
/// use xmltree::Element;
/// use xacro::utils::xml::{is_xacro_element, XACRO_NAMESPACE};
///
/// let xml = r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
///     <xacro:if value="true">
///         <link name="base"/>
///     </xacro:if>
/// </robot>"#;
///
/// let root = Element::parse(xml.as_bytes()).unwrap();
/// let if_elem = root.get_child("if").unwrap();
/// assert!(is_xacro_element(if_elem, "if"));
/// ```
pub fn is_xacro_element(
    element: &Element,
    tag_name: &str,
) -> bool {
    element.name == tag_name && element.namespace.as_deref() == Some(XACRO_NAMESPACE)
}

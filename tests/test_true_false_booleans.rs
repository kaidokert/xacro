use xacro::XacroProcessor;

mod common;
use common::*;

/// Test True/False boolean constants in property values
#[test]
fn test_true_false_property_values() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="flag" value="True"/>
  <xacro:property name="disabled" value="False"/>

  <link name="test">
    <visual>
      <geometry>
        <box size="${flag} ${disabled} 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    assert_eq!(get_attr(box_elem, "size"), "1 0 1");
}

/// Test True/False in boolean comparisons
#[test]
fn test_true_false_comparisons() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="flag" value="True"/>

  <xacro:if value="${flag == True}">
    <link name="enabled"/>
  </xacro:if>

  <xacro:if value="${flag == False}">
    <link name="disabled"/>
  </xacro:if>
</robot>"#;

    let root = run_xacro_to_xml(input);

    // Should have <link name="enabled">
    let enabled = root
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .find(|e| e.name == "link" && get_attr_opt(e, "name") == Some("enabled"));
    assert!(enabled.is_some(), "Should contain enabled link");

    // Should NOT have <link name="disabled">
    let disabled = root
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .find(|e| e.name == "link" && get_attr_opt(e, "name") == Some("disabled"));
    assert!(disabled.is_none(), "Should not contain disabled link");
}

/// Test lowercase true/false
#[test]
fn test_lowercase_true_false() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="flag" value="true"/>

  <xacro:if value="${flag}">
    <link name="enabled"/>
  </xacro:if>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    assert_xacro_contains!(output, r#"<link name="enabled""#);
}

/// Test True/False in conditionals
#[test]
fn test_true_false_in_conditionals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="enabled" value="True"/>
  <xacro:property name="disabled" value="False"/>

  <link name="result1">
    <visual>
      <geometry>
        <box size="${1 if enabled else 0} 1 1"/>
      </geometry>
    </visual>
  </link>

  <link name="result2">
    <visual>
      <geometry>
        <box size="${1 if disabled else 0} 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);

    // Find result1 link element
    let result1_link = root
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .find(|e| e.name == "link" && get_attr_opt(e, "name") == Some("result1"))
        .expect("Should find <link name=\"result1\">");

    // Navigate to box element and check size: enabled=True should give "1 1 1"
    let result1_visual = find_child(result1_link, "visual");
    let result1_geometry = find_child(result1_visual, "geometry");
    let result1_box = find_child(result1_geometry, "box");
    assert_eq!(get_attr(result1_box, "size"), "1 1 1");

    // Find result2 link element
    let result2_link = root
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .find(|e| e.name == "link" && get_attr_opt(e, "name") == Some("result2"))
        .expect("Should find <link name=\"result2\">");

    // Navigate to box element and check size: disabled=False should give "0 1 1"
    let result2_visual = find_child(result2_link, "visual");
    let result2_geometry = find_child(result2_visual, "geometry");
    let result2_box = find_child(result2_geometry, "box");
    assert_eq!(get_attr(result2_box, "size"), "0 1 1");
}

/// Test True/False comparison between properties
/// The type coercion makes boolean properties comparable
#[test]
fn test_boolean_property_comparisons() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="flag1" value="True"/>
  <xacro:property name="flag2" value="True"/>
  <xacro:property name="flag3" value="False"/>

  <xacro:if value="${flag1 == flag2}">
    <link name="equal"/>
  </xacro:if>

  <xacro:if value="${flag1 == flag3}">
    <link name="not_equal"/>
  </xacro:if>
</robot>"#;

    let root = run_xacro_to_xml(input);

    // Should have <link name="equal">
    let equal = root
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .find(|e| e.name == "link" && get_attr_opt(e, "name") == Some("equal"));
    assert!(equal.is_some(), "Should contain <link name=\"equal\">");

    // Should NOT have <link name="not_equal">
    let not_equal = root
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .find(|e| e.name == "link" && get_attr_opt(e, "name") == Some("not_equal"));
    assert!(
        not_equal.is_none(),
        "Should not contain <link name=\"not_equal\">"
    );
}

/// Test True/False passed as macro parameters
#[test]
fn test_true_false_macro_parameters() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="enabled">
    <xacro:if value="${enabled}">
      <link name="macro_enabled"/>
    </xacro:if>
  </xacro:macro>

  <xacro:test_macro enabled="True"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    assert_xacro_contains!(output, r#"<link name="macro_enabled""#);
}

/// Test True/False with NOT operator
#[test]
fn test_true_false_with_not() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="disabled" value="False"/>

  <xacro:if value="${not disabled}">
    <link name="enabled_by_not"/>
  </xacro:if>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    assert_xacro_contains!(output, r#"<link name="enabled_by_not""#);
}

/// Test mixed case True/False (should remain strings)
#[test]
fn test_mixed_case_true_false() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="val1" value="TRUE"/>
  <xacro:property name="val2" value="TrUe"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- These should remain as strings since they're not exact matches -->
        <box size="${val1} ${val2} 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // Mixed case should not be converted to booleans
    assert_xacro_contains!(output, "TRUE");
    assert_xacro_contains!(output, "TrUe");
}

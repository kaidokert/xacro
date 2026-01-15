mod common;
use common::*;

/// Test that True/False literals are tracked and formatted correctly
#[test]
fn test_boolean_literal_tracking() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="enabled" value="True"/>
  <xacro:property name="disabled" value="False"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- Direct references to boolean properties should output as True/False -->
        <box size="${enabled} ${disabled} 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Boolean literals should be preserved as "True" and "False"
    assert_eq!(get_attr(box_elem, "size"), "True False 1");
}

/// Test that boolean tracking propagates through property references
#[test]
fn test_boolean_propagation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="flag" value="True"/>
  <xacro:property name="flag_copy" value="${flag}"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- Boolean metadata should propagate through property references -->
        <box size="${flag_copy} 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Boolean metadata should propagate
    assert_eq!(get_attr(box_elem, "size"), "True 1 1");
}

/// Test that complex expressions are NOT formatted as boolean
#[test]
fn test_complex_expressions_not_boolean() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="flag" value="True"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- Complex expressions should output numeric values -->
        <box size="${1 if flag else 0} ${flag * 1} ${flag + 0}"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Complex expressions should NOT be formatted as boolean
    assert_eq!(get_attr(box_elem, "size"), "1 1 1");
}

/// Test that arithmetic with boolean properties outputs numeric values
#[test]
fn test_boolean_arithmetic() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="enabled" value="True"/>
  <xacro:property name="disabled" value="False"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- Arithmetic operations should output numeric values -->
        <box size="${enabled + 1} ${disabled + 1} ${enabled * 2}"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Arithmetic operations should produce numeric results
    assert_eq!(get_attr(box_elem, "size"), "2 1 2");
}

/// Test that comparison results are NOT formatted as boolean
/// (because pyisheval returns Number, not a marked boolean)
#[test]
fn test_comparison_results() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="5"/>
  <xacro:property name="y" value="3"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- Comparison results return 1/0, not True/False -->
        <box size="${x == 5} ${x != y} ${y == 3}"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Comparison results should be numeric (limitation of pyisheval)
    assert_eq!(get_attr(box_elem, "size"), "1 1 1");
}

/// Test boolean tracking in macro parameters
#[test]
fn test_boolean_in_macro_params() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="flag">
    <link name="test">
      <visual>
        <geometry>
          <box size="${flag} 1 1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:property name="enabled" value="True"/>
  <xacro:test_macro flag="${enabled}"/>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Boolean tracking should work through macro parameters
    assert_eq!(get_attr(box_elem, "size"), "True 1 1");
}

/// Test lowercase boolean literals (XML/xacro convention)
#[test]
fn test_lowercase_boolean_tracking() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="enabled" value="true"/>
  <xacro:property name="disabled" value="false"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- Lowercase booleans should be formatted as True/False -->
        <box size="${enabled} ${disabled} 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Lowercase "true"/"false" should be formatted as "True"/"False"
    assert_eq!(get_attr(box_elem, "size"), "True False 1");
}

/// Test the pelican.xacro case: namespace:=true from command line
/// This test verifies that boolean metadata is recomputed after extension resolution
#[test]
fn test_namespace_arg_lowercase() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="namespace" default="true"/>
  <xacro:property name="ns" value="$(arg namespace)"/>

  <link name="${ns}/base_link"/>
  <joint name="${ns}/base_joint" type="fixed">
    <parent link="${ns}/base_link"/>
    <child link="${ns}/base_link_inertia"/>
  </joint>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");

    // The namespace should be formatted as "True", not "1"
    assert_eq!(get_attr(link, "name"), "True/base_link");
}

/// Test macro parameters receiving boolean values from $(arg ...) extensions
/// This verifies that metadata is recomputed for scoped properties (macro params)
#[test]
fn test_macro_param_with_arg_extension() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="use_namespace" default="true"/>

  <xacro:macro name="test_component" params="enable">
    <link name="component">
      <visual>
        <geometry>
          <box size="${enable} 1 1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:test_component enable="$(arg use_namespace)"/>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Macro parameter should be formatted as "True" even when resolved from $(arg)
    assert_eq!(get_attr(box_elem, "size"), "True 1 1");
}

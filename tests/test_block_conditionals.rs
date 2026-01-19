/// Block parameter conditional tests
///
/// These tests verify that xacro:if/xacro:unless conditionals are processed
/// BEFORE collecting block parameters from macro calls.
///
/// This matches Python xacro's behavior: eval_all() processes conditionals at line 813,
/// then collect blocks at line 816.
mod common;
use crate::common::*;

#[test]
fn test_if_around_block_parameter_true() {
    // When condition is true, the block inside xacro:if should be captured
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*origin">
    <joint name="test_joint" type="fixed">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:property name="enable" value="1"/>

  <xacro:test_macro>
    <xacro:if value="${enable}">
      <origin xyz="1 0 0" rpy="0 0 0"/>
    </xacro:if>
  </xacro:test_macro>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let joint = find_child(&root, "joint");
    assert_eq!(get_attr(joint, "name"), "test_joint");

    let origin = find_child(joint, "origin");
    assert_eq!(get_attr(origin, "xyz"), "1 0 0");
}

#[test]
fn test_if_around_block_parameter_false() {
    // When condition is false, the xacro:if should be removed,
    // leaving no block parameter (should error with MissingBlockParameter)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*origin">
    <joint name="test_joint" type="fixed">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:arg name="enable" default="False"/>

  <xacro:test_macro>
    <xacro:if value="${$(arg enable)}">
      <origin xyz="1 0 0" rpy="0 0 0"/>
    </xacro:if>
  </xacro:test_macro>
</robot>"#;

    let result = std::panic::catch_unwind(|| run_xacro(input));
    assert!(
        result.is_err(),
        "Should fail with missing block parameter when condition is false"
    );
}

#[test]
fn test_if_unless_pair_with_block_parameter() {
    // The corpus case: xacro:if and xacro:unless both containing origin blocks
    // Only one should survive based on condition
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="base" params="*origin">
    <joint name="base_joint" type="fixed">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:arg name="enable_carrier" default="False"/>

  <xacro:base>
    <xacro:if value="${$(arg enable_carrier)}">
      <origin xyz="0.0 0.0 0.2808" rpy="0 0 0"/>
    </xacro:if>
    <xacro:unless value="${$(arg enable_carrier)}">
      <origin xyz="0.0 0.0 0.1138" rpy="0 0 0"/>
    </xacro:unless>
  </xacro:base>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let joint = find_child(&root, "joint");
    let origin = find_child(joint, "origin");
    // Default is false, so unless branch should be taken
    assert_eq!(get_attr(origin, "xyz"), "0.0 0.0 0.1138");
}

#[test]
fn test_if_unless_pair_with_block_parameter_enabled() {
    // Same as above but with enable_carrier=true
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="base" params="*origin">
    <joint name="base_joint" type="fixed">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:arg name="enable_carrier" default="True"/>

  <xacro:base>
    <xacro:if value="${$(arg enable_carrier)}">
      <origin xyz="0.0 0.0 0.2808" rpy="0 0 0"/>
    </xacro:if>
    <xacro:unless value="${$(arg enable_carrier)}">
      <origin xyz="0.0 0.0 0.1138" rpy="0 0 0"/>
    </xacro:unless>
  </xacro:base>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let joint = find_child(&root, "joint");
    let origin = find_child(joint, "origin");
    // enable_carrier is true, so if branch should be taken
    assert_eq!(get_attr(origin, "xyz"), "0.0 0.0 0.2808");
}

#[test]
fn test_multiple_block_params_with_conditionals() {
    // Multiple block parameters with conditionals mixed in
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*origin *inertia">
    <link name="test">
      <xacro:insert_block name="origin"/>
      <xacro:insert_block name="inertia"/>
    </link>
  </xacro:macro>

  <xacro:arg name="use_simple" default="True"/>

  <xacro:test_macro>
    <xacro:if value="${$(arg use_simple)}">
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:if>
    <xacro:unless value="${$(arg use_simple)}">
      <origin xyz="1 1 1" rpy="0 0 0"/>
    </xacro:unless>

    <inertial>
      <mass value="1.0"/>
    </inertial>
  </xacro:test_macro>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let link = find_child(&root, "link");
    let origin = find_child(link, "origin");
    assert_eq!(get_attr(origin, "xyz"), "0 0 0");

    let inertial = find_child(link, "inertial");
    let mass = find_child(inertial, "mass");
    assert_eq!(get_attr(mass, "value"), "1.0");
}

#[test]
fn test_nested_conditionals_in_block() {
    // Nested xacro:if inside a block parameter
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*content">
    <result>
      <xacro:insert_block name="content"/>
    </result>
  </xacro:macro>

  <xacro:arg name="outer" default="True"/>

  <xacro:test_macro>
    <xacro:if value="${$(arg outer)}">
      <wrapper>
        <xacro:if value="1">
          <inner>nested</inner>
        </xacro:if>
      </wrapper>
    </xacro:if>
  </xacro:test_macro>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let result = find_child(&root, "result");
    let wrapper = find_child(result, "wrapper");
    let inner = find_child(wrapper, "inner");
    assert_eq!(inner.get_text().as_deref(), Some("nested"));
}

#[test]
fn test_conditional_removes_all_children() {
    // All children are inside conditionals that evaluate to false
    // Should error with missing block parameter
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*origin">
    <joint name="test_joint">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:test_macro>
    <xacro:if value="false">
      <origin xyz="1 0 0"/>
    </xacro:if>
    <xacro:if value="false">
      <origin xyz="0 1 0"/>
    </xacro:if>
  </xacro:test_macro>
</robot>"#;

    let result = std::panic::catch_unwind(|| run_xacro(input));
    assert!(
        result.is_err(),
        "Should fail when all conditionals are false and no block remains"
    );
}

#[test]
fn test_property_in_conditional_guard() {
    // Use property in the conditional expression
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="use_option_a" value="1"/>

  <xacro:macro name="test_macro" params="*origin">
    <joint name="test_joint">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:test_macro>
    <xacro:if value="${use_option_a}">
      <origin xyz="1 0 0" rpy="0 0 0"/>
    </xacro:if>
    <xacro:unless value="${use_option_a}">
      <origin xyz="0 1 0" rpy="0 0 0"/>
    </xacro:unless>
  </xacro:test_macro>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let joint = find_child(&root, "joint");
    let origin = find_child(joint, "origin");
    assert_eq!(get_attr(origin, "xyz"), "1 0 0");
}

#[test]
fn test_lazy_block_with_conditionals() {
    // Test ** (lazy block) with conditionals
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="**items">
    <container>
      <xacro:insert_block name="items"/>
    </container>
  </xacro:macro>

  <xacro:arg name="include_second" default="True"/>

  <xacro:test_macro>
    <wrapper>
      <item id="1"/>
      <xacro:if value="${$(arg include_second)}">
        <item id="2"/>
      </xacro:if>
      <item id="3"/>
    </wrapper>
  </xacro:test_macro>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let container = find_child(&root, "container");
    // ** strips wrapper, so we should see items directly
    let items: Vec<_> = container
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .filter(|e| e.name == "item")
        .collect();

    assert_eq!(items.len(), 3, "Should have 3 items (conditional expanded)");
    assert_eq!(get_attr(items[0], "id"), "1");
    assert_eq!(get_attr(items[1], "id"), "2");
    assert_eq!(get_attr(items[2], "id"), "3");
}

#[test]
fn test_conditional_with_expression_in_block() {
    // Expression evaluation inside conditional block
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base_height" value="0.1"/>
  <xacro:property name="carrier_height" value="0.15"/>

  <xacro:macro name="base" params="*origin">
    <joint name="base_joint">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:arg name="with_carrier" default="False"/>

  <xacro:base>
    <xacro:if value="${$(arg with_carrier)}">
      <origin xyz="0 0 ${base_height + carrier_height}"/>
    </xacro:if>
    <xacro:unless value="${$(arg with_carrier)}">
      <origin xyz="0 0 ${base_height}"/>
    </xacro:unless>
  </xacro:base>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let joint = find_child(&root, "joint");
    let origin = find_child(joint, "origin");
    // with_carrier is false, so should use base_height only
    assert_eq!(get_attr(origin, "xyz"), "0 0 0.1");
}

#[test]
fn test_multiple_conditionals_different_blocks() {
    // Different conditionals control different block parameters
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*first *second">
    <result>
      <xacro:insert_block name="first"/>
      <xacro:insert_block name="second"/>
    </result>
  </xacro:macro>

  <xacro:arg name="use_a" default="True"/>
  <xacro:arg name="use_b" default="False"/>

  <xacro:test_macro>
    <xacro:if value="${$(arg use_a)}">
      <first_a/>
    </xacro:if>
    <xacro:unless value="${$(arg use_a)}">
      <first_b/>
    </xacro:unless>

    <xacro:if value="${$(arg use_b)}">
      <second_a/>
    </xacro:if>
    <xacro:unless value="${$(arg use_b)}">
      <second_b/>
    </xacro:unless>
  </xacro:test_macro>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let result = find_child(&root, "result");
    // use_a=true, use_b=false
    assert!(
        result
            .children
            .iter()
            .any(|n| n.as_element().map(|e| e.name == "first_a").unwrap_or(false)),
        "Should have first_a"
    );
    assert!(
        result.children.iter().any(|n| n
            .as_element()
            .map(|e| e.name == "second_b")
            .unwrap_or(false)),
        "Should have second_b"
    );
}

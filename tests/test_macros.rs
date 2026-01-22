//! Macro-related integration tests
//!
//! Tests for macro definitions, parameter handling, defaults, and nested calls.

mod common;
use crate::common::*;
use xacro::error::XacroError;

// ============================================================================
// Macro Definition Tests
// ============================================================================

/// Test that macro definitions (without calls) don't trigger undefined variable errors
///
/// If a file only defines a macro but never calls it, the macro parameters
/// should never be evaluated during definition.
#[test]
fn test_macro_definition_without_call() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="name">
    <link name="${name}"/>
  </xacro:macro>
</robot>"#;

    let output = run_xacro_expect(input, "Macro definition should not evaluate parameters");

    // Output should be empty robot (macro definition removed, no expansion)
    assert_xacro_not_contains!(output, "link", "Should not contain any links");
    assert_xacro_contains!(output, "<robot", "Should have robot element");
}

/// Test the actual failing case from PR2
#[test]
fn test_pr2_head_gazebo_macro_only() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
       xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
       xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="pr2_head_gazebo_v0" params="name">
    <gazebo reference="${name}_plate_frame">
      <material value="Gazebo/Grey" />
    </gazebo>
  </xacro:macro>

</robot>"#;

    let _output = run_xacro_expect(
        input,
        "PR2 head gazebo macro (definition only) should process without error",
    );
}

/// Test macro with parameter used in attribute value
#[test]
fn test_macro_definition_param_in_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="prefix">
    <link name="${prefix}_link"/>
  </xacro:macro>
</robot>"#;

    let _output = run_xacro_expect(
        input,
        "Macro param in attribute should not be evaluated during definition",
    );
}

/// Test macro with parameter used in text node
#[test]
fn test_macro_definition_param_in_text() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="value">
    <item>${value}</item>
  </xacro:macro>
</robot>"#;

    let _output = run_xacro_expect(
        input,
        "Macro param in text node should not be evaluated during definition",
    );
}

/// Test macro with complex expression using parameter
#[test]
fn test_macro_definition_param_in_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="size">
    <box size="${size * 2} ${size} ${size / 2}"/>
  </xacro:macro>
</robot>"#;

    let _output = run_xacro_expect(
        input,
        "Macro param in expression should not be evaluated during definition",
    );
}

// ============================================================================
// Macro Default Parameters Tests
// ============================================================================

/// Helper function to extract box size attribute from processed xacro
fn get_box_size(input: &str) -> String {
    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");
    get_attr(box_elem, "size").to_string()
}

/// Test that macro parameters can use = or := for default values
#[test]
fn test_macro_default_param_with_equals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_width" value="0.5"/>

  <xacro:macro name="test_box" params="length width=${default_width}">
    <link name="box">
      <visual>
        <geometry>
          <box size="${length} ${width} 0.1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:test_box length="1.0"/>
</robot>"#;

    assert_eq!(get_box_size(input), "1.0 0.5 0.1");
}

/// Test that := syntax also works (original syntax)
#[test]
fn test_macro_default_param_with_colon_equals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_width" value="0.5"/>

  <xacro:macro name="test_box" params="length width:=${default_width}">
    <link name="box">
      <visual>
        <geometry>
          <box size="${length} ${width} 0.1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:test_box length="1.0"/>
</robot>"#;

    assert_eq!(get_box_size(input), "1.0 0.5 0.1");
}

/// Test overriding default parameter
#[test]
fn test_macro_default_param_override() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_width" value="0.5"/>

  <xacro:macro name="test_box" params="length width=${default_width}">
    <link name="box">
      <visual>
        <geometry>
          <box size="${length} ${width} 0.1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:test_box length="1.0" width="2.0"/>
</robot>"#;

    assert_eq!(get_box_size(input), "1.0 2.0 0.1");
}

/// Test that block parameters cannot have default values with =
#[test]
fn test_block_param_with_equals_rejected() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*block=default">
    <link name="test"/>
  </xacro:macro>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        matches!(result, Err(ref e) if matches!(e, XacroError::BlockParameterWithDefault { .. })),
        "Expected BlockParameterWithDefault error, got: {:?}",
        result
    );
}

/// Test that block parameters cannot have default values with :=
#[test]
fn test_block_param_with_colon_equals_rejected() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="*block:=default">
    <link name="test"/>
  </xacro:macro>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        matches!(result, Err(ref e) if matches!(e, XacroError::BlockParameterWithDefault { .. })),
        "Expected BlockParameterWithDefault error, got: {:?}",
        result
    );
}

// ============================================================================
// Nested Macro Parameter Expression Tests
// ============================================================================

/// Test that macro calls can use parameter expressions in their attributes
///
/// ```xml
/// <xacro:macro name="outer" params="prefix">
///   <xacro:inner prefix="${prefix}"/>  <!-- ${prefix} must be evaluated in outer's scope -->
/// </xacro:macro>
/// ```
#[test]
fn test_nested_macro_with_parameter_expression_in_call() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Inner macro that takes a prefix parameter -->
  <xacro:macro name="wheel" params="prefix side">
    <link name="${prefix}_${side}_wheel"/>
  </xacro:macro>

  <!-- Outer macro that calls inner macro with expression in attribute -->
  <xacro:macro name="rocker" params="prefix">
    <link name="${prefix}_rocker"/>
    <xacro:wheel prefix="${prefix}" side="left"/>
    <xacro:wheel prefix="${prefix}" side="right"/>
  </xacro:macro>

  <!-- Call outer macro -->
  <xacro:rocker prefix="front"/>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Nested macro with parameter expression in call should work",
    );

    // Should expand outer macro with prefix="front"
    assert_xacro_contains!(
        output,
        r#"name="front_rocker""#,
        "Outer macro should expand with prefix"
    );

    // Inner macro calls should receive evaluated prefix value
    assert_xacro_contains!(
        output,
        r#"name="front_left_wheel""#,
        "Inner macro should receive evaluated prefix parameter"
    );
    assert_xacro_contains!(
        output,
        r#"name="front_right_wheel""#,
        "Inner macro should receive evaluated prefix parameter"
    );
}

/// Test with multiple levels of nesting
#[test]
fn test_triple_nested_macro_with_parameter_expressions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Innermost macro -->
  <xacro:macro name="component" params="name">
    <link name="${name}_component"/>
  </xacro:macro>

  <!-- Middle macro -->
  <xacro:macro name="assembly" params="prefix side">
    <xacro:component name="${prefix}_${side}"/>
  </xacro:macro>

  <!-- Outermost macro -->
  <xacro:macro name="system" params="prefix">
    <xacro:assembly prefix="${prefix}" side="left"/>
    <xacro:assembly prefix="${prefix}" side="right"/>
  </xacro:macro>

  <!-- Call top-level macro -->
  <xacro:system prefix="robot"/>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Triple nested macros with parameter expressions should work",
    );
    assert_xacro_contains!(
        output,
        r#"name="robot_left_component""#,
        "Triple nesting should pass parameters through all levels"
    );
    assert_xacro_contains!(
        output,
        r#"name="robot_right_component""#,
        "Triple nesting should pass parameters through all levels"
    );
}

/// Test that parameters can be used in complex expressions in nested calls
#[test]
fn test_nested_macro_with_arithmetic_expression_in_call() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="box" params="size">
    <box size="${size} ${size} ${size}"/>
  </xacro:macro>

  <xacro:macro name="scaled_box" params="base_size scale">
    <xacro:box size="${base_size * scale}"/>
  </xacro:macro>

  <xacro:scaled_box base_size="1.0" scale="2.5"/>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Nested macro with arithmetic expression in call should work",
    );
    assert_xacro_contains!(
        output,
        "2.5 2.5 2.5",
        "Arithmetic expression in nested macro call should be evaluated"
    );
}

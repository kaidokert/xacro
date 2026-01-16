mod common;
use common::*;

// Test 'and' operator in xacro conditionals and property resolution
#[test]
fn test_and_operator_in_conditionals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="true"/>
  <xacro:property name="bar" value="true"/>
  <xacro:property name="baz" value="false"/>

  <!-- Test 'and' with two true values -->
  <xacro:if value="${foo and bar}">
    <link name="both_true"/>
  </xacro:if>

  <!-- Test 'and' with one false value -->
  <xacro:if value="${foo and baz}">
    <link name="one_false"/>
  </xacro:if>

  <!-- Test 'and' with both false -->
  <xacro:if value="${baz and baz}">
    <link name="both_false"/>
  </xacro:if>
</robot>"#;

    let output = run_xacro(input);

    // Should include link when both are true
    assert_xacro_contains!(&output, r#"<link name="both_true""#);

    // Should NOT include link when one is false
    assert_xacro_not_contains!(&output, r#"<link name="one_false""#);

    // Should NOT include link when both are false
    assert_xacro_not_contains!(&output, r#"<link name="both_false""#);
}

// Test 'or' operator in xacro conditionals and property resolution
#[test]
fn test_or_operator_in_conditionals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="true"/>
  <xacro:property name="bar" value="true"/>
  <xacro:property name="baz" value="false"/>

  <!-- Test 'or' with both true -->
  <xacro:if value="${foo or bar}">
    <link name="both_true"/>
  </xacro:if>

  <!-- Test 'or' with one true -->
  <xacro:if value="${foo or baz}">
    <link name="one_true"/>
  </xacro:if>

  <!-- Test 'or' with both false -->
  <xacro:if value="${baz or baz}">
    <link name="both_false"/>
  </xacro:if>
</robot>"#;

    let output = run_xacro(input);

    // Should include link when both are true
    assert_xacro_contains!(&output, r#"<link name="both_true""#);

    // Should include link when at least one is true
    assert_xacro_contains!(&output, r#"<link name="one_true""#);

    // Should NOT include link when both are false
    assert_xacro_not_contains!(&output, r#"<link name="both_false""#);
}

// Test 'and'/'or' combined with comparison operators
#[test]
fn test_logical_operators_with_comparisons() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="5"/>
  <xacro:property name="bar" value="10"/>
  <xacro:property name="baz" value="15"/>

  <!-- Test 'and' with comparisons -->
  <xacro:if value="${foo > 0 and bar > 5}">
    <link name="and_comparison"/>
  </xacro:if>

  <!-- Test 'or' with comparisons -->
  <xacro:if value="${foo > 100 or baz == 15}">
    <link name="or_comparison"/>
  </xacro:if>

  <!-- Test complex condition -->
  <xacro:if value="${(foo > 0) and (bar > 0)}">
    <link name="complex_condition"/>
  </xacro:if>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(&output, r#"<link name="and_comparison""#);
    assert_xacro_contains!(&output, r#"<link name="or_comparison""#);
    assert_xacro_contains!(&output, r#"<link name="complex_condition""#);
}

// Test real-world use case: robot namespace validation
#[test]
fn test_robot_namespace_validation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="robot_ns" value="my_robot"/>

  <!-- Check that namespace is valid (not empty and not root) -->
  <xacro:if value="${robot_ns != '' and robot_ns != '/'}">
    <link name="valid_namespace">
      <visual>
        <geometry>
          <box size="1 1 1"/>
        </geometry>
      </visual>
    </link>
  </xacro:if>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"<link name="valid_namespace""#);
}

// Test real-world use case: hardware mode selection
#[test]
fn test_hardware_mode_selection() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="use_fake_hardware" value="false"/>
  <xacro:property name="sim_gazebo" value="true"/>
  <xacro:property name="sim_ignition" value="false"/>

  <!-- Include simulation plugin if ANY simulation mode is active -->
  <xacro:if value="${use_fake_hardware or sim_gazebo or sim_ignition}">
    <gazebo>
      <plugin name="simulation_plugin"/>
    </gazebo>
  </xacro:if>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"<plugin name="simulation_plugin""#);
}

// Test property evaluation with logical operators
#[test]
fn test_property_evaluation_with_logical_operators() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="true"/>
  <xacro:property name="bar" value="false"/>

  <!-- Compute properties using logical operators -->
  <xacro:property name="result_and" value="${foo and bar}"/>
  <xacro:property name="result_or" value="${foo or bar}"/>
  <xacro:property name="result_complex" value="${(foo and foo) or (bar and bar)}"/>

  <link name="test">
    <visual>
      <geometry>
        <!-- result_and should be False, result_or should be True, result_complex should be True -->
        <box size="${result_and} ${result_or} ${result_complex}"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // pyisheval v0.13 outputs True/False for boolean results
    assert_xacro_attr!(box_elem, "size", "False True True");
}

// Test chained logical operators
#[test]
fn test_chained_logical_operators() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="foo" value="true"/>
  <xacro:property name="bar" value="true"/>
  <xacro:property name="baz" value="true"/>

  <!-- Test chained 'and' -->
  <xacro:if value="${foo and bar and baz}">
    <link name="all_true"/>
  </xacro:if>

  <!-- Test chained 'or' with one true -->
  <xacro:property name="qux" value="false"/>
  <xacro:if value="${qux or qux or foo}">
    <link name="at_least_one_true"/>
  </xacro:if>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"<link name="all_true""#);
    assert_xacro_contains!(&output, r#"<link name="at_least_one_true""#);
}

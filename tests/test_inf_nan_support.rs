//! Integration tests for inf and nan constant support
//!
//! Python xacro provides inf and nan via math.inf/math.nan.
//! We inject them directly into the pyisheval context HashMap to bypass parsing.

mod common;
use crate::common::*;
use std::collections::HashMap;

/// Test that inf is available as a constant in expressions
#[test]
fn test_inf_constant_in_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="pos_inf" value="${inf}" />
  <link name="test">
    <visual>
      <origin xyz="${pos_inf} 0 0" rpy="0 0 0"/>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro_expect(input, "Processing should succeed");
    assert_xacro_contains!(output, "inf", "Output should contain 'inf'");
    assert_xacro_contains!(output, r#"xyz="inf"#, "Output should have xyz='inf ...'");
    assert_xacro_not_contains!(
        output,
        "xacro:property",
        "Should not contain xacro directives"
    );
}

/// Test that nan is available as a constant in expressions
#[test]
fn test_nan_constant_in_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="not_a_number" value="${nan}" />
  <link name="test">
    <visual>
      <origin xyz="${not_a_number} 0 0" rpy="0 0 0"/>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro_expect(input, "Processing should succeed");
    // NaN can format as "NaN" or "nan" depending on platform
    assert!(
        output.contains("NaN") || output.contains("nan"),
        "Output should contain NaN or nan, got: {}",
        output
    );
    assert_xacro_not_contains!(
        output,
        "xacro:property",
        "Should not contain xacro directives"
    );
}

/// Test that inf works in arithmetic expressions
#[test]
fn test_inf_in_arithmetic() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="twice_inf" value="${inf * 2}" />
  <xacro:property name="neg_inf" value="${-inf}" />
  <link name="test">
    <visual>
      <origin xyz="${twice_inf} ${neg_inf} 0" rpy="0 0 0"/>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro_expect(input, "Processing should succeed");
    assert_xacro_contains!(output, "inf", "inf * 2 should still be inf");
    assert_xacro_contains!(output, "-inf", "Output should contain -inf for negation");
}

/// Test that nan works in arithmetic expressions (nan propagates)
#[test]
fn test_nan_in_arithmetic() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="nan_plus_one" value="${nan + 1}" />
  <xacro:property name="nan_times_two" value="${nan * 2}" />
  <link name="test">
    <visual>
      <origin xyz="${nan_plus_one} ${nan_times_two} 0" rpy="0 0 0"/>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro_expect(input, "Processing should succeed");
    // NaN propagates through arithmetic - both should be NaN
    let nan_count = output.matches("NaN").count() + output.matches("nan").count();
    assert!(
        nan_count >= 2,
        "Output should have at least 2 NaN/nan occurrences"
    );
}

/// Test that inf and nan can be used in conditionals (comparisons)
#[test]
fn test_inf_nan_in_conditionals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="x" value="5" />
  <xacro:if value="${x &lt; inf}">
    <link name="finite"/>
  </xacro:if>
  <xacro:if value="${x == nan}">
    <link name="not_shown"/>
  </xacro:if>
</robot>"#;

    let output = run_xacro_expect(input, "Processing should succeed");
    assert_xacro_contains!(
        output,
        r#"name="finite"#,
        "Should have finite link (5 < inf is true)"
    );
    assert_xacro_not_contains!(
        output,
        r#"name="not_shown"#,
        "Should not have not_shown link (5 == nan is false)"
    );
    assert_xacro_not_contains!(output, "xacro:if", "Should not contain xacro directives");
}

/// Test that properties can be assigned inf/nan values
#[test]
fn test_property_assignment_with_inf_nan() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="infinite_mass" value="${inf}" />
  <xacro:property name="undefined_inertia" value="${nan}" />
  <link name="test">
    <inertial>
      <mass value="${infinite_mass}"/>
      <inertia ixx="${undefined_inertia}" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
</robot>"#;

    let output = run_xacro_expect(input, "Processing should succeed");
    let root = parse_xml(&output);
    let link = find_child(&root, "link");
    let inertial = find_child(link, "inertial");

    // Mass is an element with value attribute
    let mass = find_child(inertial, "mass");
    assert_xacro_attr!(mass, "value", "inf");

    // ixx is an attribute on inertia element
    let inertia = find_child(inertial, "inertia");
    let ixx_str = get_attr(inertia, "ixx");
    let ixx: f64 = ixx_str.parse().unwrap();
    assert!(ixx.is_nan(), "ixx should be NaN");
}

/// Test that inf/nan properties can be referenced by other properties
#[test]
fn test_inf_nan_property_references() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="base_inf" value="${inf}" />
  <xacro:property name="derived_inf" value="${base_inf * 3}" />
  <link name="test">
    <visual>
      <origin xyz="${derived_inf} 0 0" rpy="0 0 0"/>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro_expect(input, "Processing should succeed");
    assert_xacro_contains!(
        output,
        r#"xyz="inf"#,
        "derived_inf (inf * 3) should still be inf"
    );
}

/// Test inf/nan with xacro:arg (command-line arguments)
#[test]
fn test_inf_nan_with_args() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="max_value" default="${inf}" />
  <xacro:property name="limit" value="$(arg max_value)" />
  <link name="test">
    <visual>
      <origin xyz="${limit} 0 0" rpy="0 0 0"/>
    </visual>
  </link>
</robot>"#;

    let args = HashMap::new(); // Use default value (inf)
    let output = run_xacro_with_args(input, args);
    assert_xacro_contains!(
        output,
        r#"xyz="inf"#,
        "limit should be inf from default arg"
    );
    assert_xacro_not_contains!(
        output,
        "xacro:arg",
        "Should not contain xacro:arg directives"
    );
    assert_xacro_not_contains!(output, "$(arg", "Should not contain $(arg references");
}

/// Test that lambda expressions can reference properties with inf values
#[test]
fn test_lambda_referencing_inf_property() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="test">
  <xacro:property name="my_inf" value="${inf}"/>
  <xacro:property name="is_inf" value="lambda x: x == my_inf"/>
  <link name="base">
    <test value="${is_inf(inf)}"/>
  </link>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Lambda should be able to reference infinite property",
    );
    assert_xacro_contains!(
        output,
        r#"value="1"#,
        "Lambda should correctly evaluate inf == inf as true"
    );
}

/// Test that lambda expressions CANNOT reference properties with nan values
///
/// LIMITATION: pyisheval cannot create NaN values (0.0/0.0 triggers DivisionByZero error).
/// Properties with NaN values are not loaded into the interpreter context, so lambdas
/// that reference them will fail with "undefined variable" errors.
///
/// This test is ignored because it demonstrates a known limitation that cannot be fixed
/// without modifying pyisheval's division operator to allow IEEE-754 behavior.
///
#[test]
#[ignore = "pyisheval cannot create NaN - lambdas cannot reference nan properties"]
fn test_lambda_referencing_nan_property_fails() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="test">
  <xacro:property name="my_nan" value="${nan}"/>
  <xacro:property name="check_nan" value="lambda x: x != x"/>
  <link name="base">
    <test value="${check_nan(my_nan)}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Lambda referencing NaN property should fail (known limitation)"
    );
}

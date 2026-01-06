//! Integration tests for inf and nan constant support
//!
//! Python xacro provides inf and nan via math.inf/math.nan.
//! We inject them directly into the pyisheval context HashMap to bypass parsing.

use std::collections::HashMap;
use xacro::XacroProcessor;

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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // Check that inf appears in the output
    assert!(
        result.contains("inf"),
        "Output should contain 'inf', got: {}",
        result
    );
    assert!(
        result.contains(r#"xyz="inf"#),
        "Output should have xyz='inf ...'"
    );
    // Should not contain xacro directives
    assert!(!result.contains("xacro:property"));
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // NaN can format as "NaN" or "nan" depending on platform
    assert!(
        result.contains("NaN") || result.contains("nan"),
        "Output should contain NaN or nan, got: {}",
        result
    );
    assert!(!result.contains("xacro:property"));
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // inf * 2 should still be inf
    assert!(result.contains("inf"), "Output should contain inf");
    // Should have both positive and negative infinity
    assert!(
        result.contains("-inf"),
        "Output should contain -inf for negation"
    );
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // NaN propagates through arithmetic - both should be NaN
    let nan_count = result.matches("NaN").count() + result.matches("nan").count();
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // Should have finite link (5 < inf is true)
    assert!(
        result.contains(r#"name="finite"#),
        "Should have finite link"
    );
    // Should NOT have not_shown link (x == nan is always false)
    assert!(
        !result.contains(r#"name="not_shown"#),
        "Should not have not_shown link"
    );
    // Should not contain xacro directives
    assert!(!result.contains("xacro:if"));
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // mass should be inf
    assert!(result.contains(r#"<mass value="inf"#), "Mass should be inf");
    // ixx should be NaN or nan
    assert!(
        result.contains(r#"ixx="NaN"#) || result.contains(r#"ixx="nan"#),
        "ixx should be NaN or nan"
    );
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // derived_inf (inf * 3) should still be inf
    assert!(result.contains(r#"xyz="inf"#), "derived_inf should be inf");
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
    let processor = XacroProcessor::new_with_args(args);
    let result = processor
        .run_from_string(input)
        .expect("Processing should succeed");

    // limit should be inf from the default arg value
    assert!(
        result.contains(r#"xyz="inf"#),
        "limit should be inf from default arg"
    );
    assert!(!result.contains("xacro:arg"));
    assert!(!result.contains("$(arg"));
}

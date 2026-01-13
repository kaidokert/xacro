/// Comprehensive tests for xacro:arg feature
///
/// Tests cover:
/// 1. Basic arg with default value
/// 2. CLI args override XML defaults (The Precedence Rake)
/// 3. Undefined arg error handling
/// 4. Transitive defaults (args referencing other args)
/// 5. Args in expressions ${$(arg x)}
/// 6. Circular dependency detection
/// 7. Unknown extension types
/// 8. Invalid extension syntax
mod common;
use crate::common::*;
use std::collections::HashMap;

#[test]
fn test_arg_basic_with_default() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="robot_name" default="my_robot"/>
  <link name="$(arg robot_name)_base"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, r#"name="my_robot_base"#);
    assert_xacro_not_contains!(output, "xacro:arg");
    assert_xacro_not_contains!(output, "$(arg");
}

#[test]
fn test_arg_cli_override() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="robot_name" default="my_robot"/>
  <link name="$(arg robot_name)_base"/>
</robot>"#;

    let mut args = HashMap::new();
    args.insert("robot_name".to_string(), "custom_robot".to_string());

    let output = run_xacro_with_args(input, args);

    // CLI arg should override default
    assert_xacro_contains!(output, r#"name="custom_robot_base"#);
    assert_xacro_not_contains!(output, "my_robot");
}

#[test]
fn test_arg_undefined_error() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(arg undefined_arg)_base"/>
</robot>"#;

    assert_xacro_error!(input, "Undefined argument");
    assert_xacro_error!(input, "undefined_arg");
}

#[test]
fn test_arg_no_default_requires_cli() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="required_arg"/>
  <link name="$(arg required_arg)_base"/>
</robot>"#;

    // Without CLI value, should error
    let result = test_xacro(input);
    assert!(result.is_err());

    // With CLI value, should work
    let mut args = HashMap::new();
    args.insert("required_arg".to_string(), "provided".to_string());
    let output = run_xacro_with_args(input, args);
    assert_xacro_contains!(output, r#"name="provided_base"#);
}

#[test]
fn test_arg_transitive_defaults() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="base_name" default="robot"/>
  <xacro:arg name="full_name" default="$(arg base_name)_v2"/>
  <link name="$(arg full_name)_link"/>
</robot>"#;

    let output = run_xacro(input);

    // full_name should resolve to "robot_v2"
    assert_xacro_contains!(output, r#"name="robot_v2_link"#);
}

#[test]
fn test_arg_in_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="scale" default="2.0"/>
  <xacro:property name="length" value="${$(arg scale) * 0.5}"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="${length} 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro(input);

    // $(arg scale) = 2.0, property evaluates: ${2.0 * 0.5} = 1.0
    assert!(output.contains("1.0 0.1 0.1") || output.contains("1 0.1 0.1"));
}

#[test]
fn test_arg_circular_dependency() {
    // Self-referential arg: tries to reference itself before it's defined
    // Due to document-order processing, this results in "Undefined" not "Circular"
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="a" default="$(arg a)"/>
  <link name="base"/>
</robot>"#;

    // Document-order processing: arg 'a' isn't defined when its default is evaluated
    assert_xacro_error!(input, "Undefined argument");
    assert_xacro_error!(input, "a");
}

#[test]
fn test_arg_forward_reference_error() {
    // Args are processed in document order - forward references are undefined
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="a" default="$(arg b)"/>
  <xacro:arg name="b" default="value"/>
  <link name="base"/>
</robot>"#;

    // Should get undefined error, not circular dependency
    assert_xacro_error!(input, "Undefined argument");
    assert_xacro_error!(input, "b");
}

#[test]
fn test_extension_unknown_type() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(unknown_ext foo)_base"/>
</robot>"#;

    assert_xacro_error!(input, "Unknown extension type");
    assert_xacro_error!(input, "unknown_ext");
}

#[test]
fn test_extension_invalid_syntax_empty() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$()_base"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("Invalid extension syntax") || err.contains("empty"));
}

#[test]
fn test_extension_invalid_syntax_no_space() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(argfoo)_base"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    // With new extension system, "argfoo" is treated as unknown extension
    assert!(
        err.contains("Invalid extension syntax")
            || err.contains("format")
            || err.contains("Unknown extension")
    );
}

#[test]
fn test_arg_in_attribute_and_text() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="prefix" default="my"/>
  <link name="$(arg prefix)_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="$(arg prefix)_material"/>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, r#"name="my_link"#);
    assert_xacro_contains!(output, r#"name="my_material"#);
}

#[test]
fn test_arg_multiple_uses() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="base" default="robot"/>
  <link name="$(arg base)_link1"/>
  <link name="$(arg base)_link2"/>
  <link name="$(arg base)_link3"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, r#"name="robot_link1"#);
    assert_xacro_contains!(output, r#"name="robot_link2"#);
    assert_xacro_contains!(output, r#"name="robot_link3"#);
}

#[test]
fn test_arg_with_property() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="prefix" default="my"/>
  <xacro:property name="suffix" value="_link"/>
  <link name="$(arg prefix)${suffix}"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, r#"name="my_link"#);
}

#[test]
fn test_arg_cli_only_no_default() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(arg external_arg)_base"/>
</robot>"#;

    // Provide via CLI only (no xacro:arg in XML)
    let mut args = HashMap::new();
    args.insert("external_arg".to_string(), "from_cli".to_string());

    let output = run_xacro_with_args(input, args);

    assert_xacro_contains!(output, r#"name="from_cli_base"#);
}

#[test]
fn test_arg_dynamic_name_evaluation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="arg_name" value="my_arg"/>
  <xacro:arg name="${arg_name}" default="value"/>
  <link name="$(arg my_arg)_link"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, r#"name="value_link"#);
}

#[test]
fn test_arg_multiple_in_single_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="a" default="2"/>
  <xacro:arg name="b" default="3"/>
  <xacro:property name="result" value="${$(arg a) * $(arg b)}"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="${result} 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro(input);

    // 2 * 3 = 6
    assert_xacro_contains!(output, "6 0.1 0.1");
}

#[test]
fn test_find_extension_unimplemented() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(find my_package)/base"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("$(find") || err.contains("not yet implemented"));
}

#[test]
fn test_env_extension_implemented() {
    // Set a test environment variable with automatic cleanup
    let _guard = EnvVarGuard::new("XACRO_TEST_VAR", "test_value");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(env XACRO_TEST_VAR)_base"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_ok(), "$(env) extension should work");
    let output = result.unwrap();
    assert_xacro_contains!(output, "test_value_base");
    // _guard automatically cleans up on drop
}

// ============================================================================
// WHITESPACE HANDLING TESTS
// ============================================================================

#[test]
fn test_extension_multiple_spaces() {
    // Test that multiple spaces between type and arg are handled correctly
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="robot_name" default="myrobot"/>
  <link name="$(arg    robot_name)_link"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, "myrobot_link");
    assert_xacro_not_contains!(output, "$(arg");
}

#[test]
fn test_extension_tab_whitespace() {
    // Test that tabs between type and arg are handled correctly
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="robot_name" default="myrobot"/>
  <link name="$(arg	robot_name)_link"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, "myrobot_link");
    assert_xacro_not_contains!(output, "$(arg");
}

#[test]
fn test_extension_mixed_whitespace() {
    // Test mixed spaces and tabs
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="value" default="42"/>
  <link name="link_$(arg 	 	value)"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, "link_42");
}

#[test]
fn test_extension_leading_trailing_spaces() {
    // Test leading and trailing spaces (should be trimmed by split_whitespace)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="name" default="base"/>
  <link name="$(  arg name  )_link"/>
</robot>"#;

    let output = run_xacro(input);

    assert_xacro_contains!(output, "base_link");
}

#[test]
fn test_extension_newline_as_whitespace() {
    // Newlines within extensions are treated as whitespace separators
    // XML parsers normalize newlines in attributes, and split_whitespace treats them as delimiters
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="val" default="test"/>
  <link name="$(arg
val)_link"/>
</robot>"#;

    let output = run_xacro(input);

    // Newline is treated as whitespace, so "arg\nval" splits to ["arg", "val"]
    assert_xacro_contains!(output, "test_link");
    assert_xacro_not_contains!(output, "$(arg");
}

#[test]
fn test_extension_only_whitespace() {
    // Extension with only whitespace should error
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(   )_link"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("empty") || err.contains("Extension"));
}

#[test]
fn test_extension_three_parts_rejected() {
    // Extension with three parts should be rejected (catches multi-word args)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(arg foo bar)_link"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    // New error format from extension handler
    assert!(
        err.contains("expects 1 argument") || err.contains("Extra parts") || err.contains("format")
    );
}

/// Test that $(arg ...) extensions are resolved in xacro:if conditionals
#[test]
fn test_arg_in_conditional_if() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="use_lidar" default="true"/>
  <xacro:if value="$(arg use_lidar)">
    <link name="lidar"/>
  </xacro:if>
</robot>"#;

    let output = run_xacro_expect(input, "Should resolve $(arg) in xacro:if");
    assert_xacro_contains!(output, "lidar");
}

/// Test that $(arg ...) extensions are resolved in xacro:unless conditionals
#[test]
fn test_arg_in_conditional_unless() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="headless" default="true"/>
  <xacro:unless value="$(arg headless)">
    <link name="scene_broadcaster"/>
  </xacro:unless>
</robot>"#;

    let output = run_xacro_expect(input, "Should resolve $(arg) in xacro:unless");
    // headless=true, so unless (not true) = false, branch is skipped
    assert_xacro_not_contains!(output, "scene_broadcaster");
}

/// Test $(arg) in conditional with false value
#[test]
fn test_arg_in_conditional_false() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="use_camera" default="false"/>
  <xacro:if value="$(arg use_camera)">
    <link name="camera"/>
  </xacro:if>
</robot>"#;

    let output = run_xacro_expect(input, "Should resolve $(arg) with false value");
    // use_camera=false, so branch is skipped
    assert_xacro_not_contains!(output, "camera");
}

/// Test that $(arg) in conditional propagates undefined argument errors
#[test]
fn test_arg_in_conditional_undefined_arg_errors() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <!-- note: no xacro:arg declaration for 'missing_arg' -->
  <xacro:if value="$(arg missing_arg)">
    <link name="should_not_appear"/>
  </xacro:if>
</robot>"#;

    // Ensure eval_boolean correctly propagates undefined-arg errors
    assert_xacro_error!(input, "Undefined argument");
    assert_xacro_error!(input, "missing_arg");
}

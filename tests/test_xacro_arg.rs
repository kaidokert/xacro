/// Comprehensive tests for xacro:arg feature (Phase 5)
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
use std::collections::HashMap;
use xacro::XacroProcessor;

#[test]
fn test_arg_basic_with_default() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="robot_name" default="my_robot"/>
  <link name="$(arg robot_name)_base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    assert!(result.contains(r#"name="my_robot_base"#));
    assert!(!result.contains("xacro:arg"));
    assert!(!result.contains("$(arg"));
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

    let processor = XacroProcessor::new_with_args(args);
    let result = processor.run_from_string(input).unwrap();

    // CLI arg should override default
    assert!(result.contains(r#"name="custom_robot_base"#));
    assert!(!result.contains("my_robot"));
}

#[test]
fn test_arg_undefined_error() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(arg undefined_arg)_base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("Undefined argument"));
    assert!(err.contains("undefined_arg"));
}

#[test]
fn test_arg_no_default_requires_cli() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="required_arg"/>
  <link name="$(arg required_arg)_base"/>
</robot>"#;

    // Without CLI value, should error
    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);
    assert!(result.is_err());

    // With CLI value, should work
    let mut args = HashMap::new();
    args.insert("required_arg".to_string(), "provided".to_string());
    let processor = XacroProcessor::new_with_args(args);
    let result = processor.run_from_string(input).unwrap();
    assert!(result.contains(r#"name="provided_base"#));
}

#[test]
fn test_arg_transitive_defaults() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="base_name" default="robot"/>
  <xacro:arg name="full_name" default="$(arg base_name)_v2"/>
  <link name="$(arg full_name)_link"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // full_name should resolve to "robot_v2"
    assert!(result.contains(r#"name="robot_v2_link"#));
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // $(arg scale) = 2.0, property evaluates: ${2.0 * 0.5} = 1.0
    assert!(result.contains("1.0 0.1 0.1") || result.contains("1 0.1 0.1"));
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    // Document-order processing: arg 'a' isn't defined when its default is evaluated
    assert!(err.contains("Undefined argument") && err.contains("a"));
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    // Should get undefined error, not circular dependency
    assert!(err.contains("Undefined argument") && err.contains("b"));
}

#[test]
fn test_extension_unknown_type() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(unknown_ext foo)_base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("Unknown extension type"));
    assert!(err.contains("unknown_ext"));
}

#[test]
fn test_extension_invalid_syntax_empty() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$()_base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("Invalid extension syntax") || err.contains("format"));
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    assert!(result.contains(r#"name="my_link"#));
    assert!(result.contains(r#"name="my_material"#));
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    assert!(result.contains(r#"name="robot_link1"#));
    assert!(result.contains(r#"name="robot_link2"#));
    assert!(result.contains(r#"name="robot_link3"#));
}

#[test]
fn test_arg_with_property() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="prefix" default="my"/>
  <xacro:property name="suffix" value="_link"/>
  <link name="$(arg prefix)${suffix}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    assert!(result.contains(r#"name="my_link"#));
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

    let processor = XacroProcessor::new_with_args(args);
    let result = processor.run_from_string(input).unwrap();

    assert!(result.contains(r#"name="from_cli_base"#));
}

#[test]
fn test_arg_dynamic_name_evaluation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="arg_name" value="my_arg"/>
  <xacro:arg name="${arg_name}" default="value"/>
  <link name="$(arg my_arg)_link"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    assert!(result.contains(r#"name="value_link"#));
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // 2 * 3 = 6
    assert!(result.contains("6 0.1 0.1"));
}

#[test]
fn test_find_extension_unimplemented() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(find my_package)/base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("$(find") || err.contains("not yet implemented"));
}

#[test]
fn test_env_extension_unimplemented() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(env HOME)/base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("$(env") || err.contains("not yet implemented"));
}

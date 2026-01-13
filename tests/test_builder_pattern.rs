use xacro::{extensions::ExtensionHandler, XacroProcessor};

/// Test extension that returns a fixed value
struct TestExtension;

impl ExtensionHandler for TestExtension {
    fn resolve(
        &self,
        command: &str,
        args_raw: &str,
    ) -> Result<Option<String>, Box<dyn std::error::Error>> {
        if command == "test" {
            Ok(Some(format!("TEST:{}", args_raw)))
        } else {
            Ok(None)
        }
    }
}

#[test]
fn test_builder_basic() {
    let processor = XacroProcessor::builder().build();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base"/>
</robot>"#;
    let result = processor.run_from_string(input);
    assert!(result.is_ok());
}

#[test]
fn test_builder_with_arg() {
    let processor = XacroProcessor::builder()
        .with_arg("robot_name", "my_robot")
        .build();

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="robot_name" default="default"/>
  <link name="$(arg robot_name)_base"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok());
    let output = result.unwrap();
    assert!(output.contains("my_robot_base"));
}

#[test]
fn test_builder_with_max_depth() {
    let processor = XacroProcessor::builder().with_max_depth(100).build();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base"/>
</robot>"#;
    let result = processor.run_from_string(input);
    assert!(result.is_ok());
}

#[test]
fn test_builder_with_compat_all() {
    let processor = XacroProcessor::builder().with_compat_all().build();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base"/>
</robot>"#;
    let result = processor.run_from_string(input);
    assert!(result.is_ok());
}

#[test]
fn test_builder_with_custom_extension() {
    let processor = XacroProcessor::builder()
        .with_extension(Box::new(TestExtension))
        .build();

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(test foo bar)_link"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok());
    let output = result.unwrap();
    assert!(output.contains("TEST:foo bar_link"));
}

#[test]
fn test_builder_clear_extensions_and_add_custom() {
    let processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(TestExtension))
        .build();

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(test hello world)_link"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok());
    let output = result.unwrap();
    assert!(output.contains("TEST:hello world_link"));
}

#[test]
fn test_builder_clear_extensions_disables_defaults() {
    let processor = XacroProcessor::builder().clear_extensions().build();

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(cwd)_link"/>
</robot>"#;

    let result = processor.run_from_string(input);
    // Should fail because cwd extension was cleared
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(err.contains("Unknown extension"));
}

#[test]
fn test_builder_chaining() {
    let processor = XacroProcessor::builder()
        .with_arg("name", "robot")
        .with_arg("version", "v1")
        .with_max_depth(200)
        .with_compat_all()
        .with_extension(Box::new(TestExtension))
        .build();

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="name" default="default"/>
  <link name="$(arg name)_$(test custom)"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok());
    let output = result.unwrap();
    assert!(output.contains("robot_TEST:custom"));
}

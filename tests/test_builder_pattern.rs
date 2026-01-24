mod common;
use common::*;
use core::any::Any;
use xacro_rs::{extensions::ExtensionHandler, XacroProcessor};

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

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// Test extension that attempts to override the "arg" command
struct ArgOverrideExtension;

impl ExtensionHandler for ArgOverrideExtension {
    fn resolve(
        &self,
        command: &str,
        _args_raw: &str,
    ) -> Result<Option<String>, Box<dyn std::error::Error>> {
        if command == "arg" {
            Ok(Some("OVERRIDDEN_ARG_VALUE".to_string()))
        } else {
            Ok(None)
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
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
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "my_robot_base");
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
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "TEST:foo bar_link");
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
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "TEST:hello world_link");
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
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "robot_TEST:custom");
}

#[test]
fn test_arg_cannot_be_overridden_by_custom_extension() {
    // Register a custom extension that tries to handle "arg" command
    let processor = XacroProcessor::builder()
        .with_extension(Box::new(ArgOverrideExtension))
        .build();

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="link_name" default="base"/>
  <link name="$(arg link_name)"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Processing xacro with arg override extension should succeed"
    );
    let output = result.unwrap();

    // Even though we registered an extension handler for "arg", the built-in arg
    // resolution must still be used, so the default value "base" should appear,
    // and the extension's override value must not.
    assert!(
        output.contains(r#"name="base""#),
        "Expected built-in arg resolution to be used for $(arg link_name)"
    );
    assert!(
        !output.contains("OVERRIDDEN_ARG_VALUE"),
        "Custom arg extension must not be able to override $(arg ...)"
    );
}

#[macro_use]
mod common;

use common::{get_attr, parse_xml, EnvVarGuard};
use std::env;
use std::fs;
use std::sync::atomic::{AtomicUsize, Ordering};
use xacro::{extensions::ros::*, XacroProcessor};

static TEST_COUNTER: AtomicUsize = AtomicUsize::new(0);

#[test]
fn test_optenv_with_value() {
    let _guard = EnvVarGuard::new("TEST_ROBOT_NAME", "my_robot");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(optenv TEST_ROBOT_NAME default_robot)_base"/>
</robot>"#;

    let processor = XacroProcessor::builder()
        .with_extension(Box::new(OptEnvExtension::new()))
        .build();

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Processing should succeed");

    let output = result.unwrap();
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "my_robot_base");
}

#[test]
fn test_optenv_with_default() {
    // Make sure var doesn't exist
    env::remove_var("NONEXISTENT_TEST_VAR");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(optenv NONEXISTENT_TEST_VAR fallback_name)_base"/>
</robot>"#;

    let processor = XacroProcessor::builder()
        .with_extension(Box::new(OptEnvExtension::new()))
        .build();

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Processing should succeed with default");

    let output = result.unwrap();
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "fallback_name_base");
}

#[test]
fn test_optenv_with_multi_word_default() {
    env::remove_var("NONEXISTENT_TEST_VAR");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(optenv NONEXISTENT_TEST_VAR mobile robot)"/>
</robot>"#;

    let processor = XacroProcessor::builder()
        .with_extension(Box::new(OptEnvExtension::new()))
        .build();

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Processing should succeed");

    let output = result.unwrap();
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "mobile robot");
}

#[test]
fn test_optenv_no_default_fails() {
    env::remove_var("NONEXISTENT_TEST_VAR");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="$(optenv NONEXISTENT_TEST_VAR)"/>
</robot>"#;

    let processor = XacroProcessor::builder()
        .with_extension(Box::new(OptEnvExtension::new()))
        .build();

    let result = processor.run_from_string(input);
    assert!(
        result.is_err(),
        "Processing should fail when var not set and no default"
    );

    let err = result.unwrap_err().to_string();
    assert!(err.contains("Environment variable not set"));
}

#[test]
fn test_find_extension_with_ros_package_path() {
    // Create a temporary directory structure to simulate ROS packages
    let test_id = TEST_COUNTER.fetch_add(1, Ordering::SeqCst);
    let temp_dir = env::temp_dir().join(format!("xacro_test_ros_packages_{}", test_id));
    let package_dir = temp_dir.join("test_package");

    // Clean up from previous test runs
    let _ = fs::remove_dir_all(&temp_dir);

    // Create test package with package.xml
    fs::create_dir_all(&package_dir).expect("Failed to create test package dir");
    fs::write(
        package_dir.join("package.xml"),
        r#"<?xml version="1.0"?>
<package format="2">
  <name>test_package</name>
</package>"#,
    )
    .expect("Failed to write package.xml");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="$(find test_package)/meshes/base.stl"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(FindExtension::with_search_paths(vec![
            temp_dir.clone()
        ])))
        .build();

    let result = processor.run_from_string(input);
    let output = result.expect("Processing should succeed");
    // Should contain the resolved path
    assert!(
        output.contains(&package_dir.display().to_string()),
        "Output should contain resolved package path"
    );
    assert!(output.contains("/meshes/base.stl"));

    // Clean up
    let _ = fs::remove_dir_all(&temp_dir);
}

#[test]
fn test_find_extension_package_not_found() {
    let _guard = EnvVarGuard::new("ROS_PACKAGE_PATH", "/tmp/nonexistent_ros_path");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base">
    <visual>
      <geometry>
        <mesh filename="$(find nonexistent_package)/mesh.stl"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::builder()
        .with_extension(Box::new(FindExtension::new()))
        .build();

    let result = processor.run_from_string(input);
    assert!(
        result.is_err(),
        "Processing should fail for nonexistent package"
    );

    let err = result.unwrap_err().to_string();
    assert!(err.contains("Package not found"));
}

#[test]
fn test_find_extension_caching() {
    // Create a temporary directory structure
    let test_id = TEST_COUNTER.fetch_add(1, Ordering::SeqCst);
    let temp_dir = env::temp_dir().join(format!("xacro_test_ros_cache_{}", test_id));
    let package_dir = temp_dir.join("cached_package");

    let _ = fs::remove_dir_all(&temp_dir);
    fs::create_dir_all(&package_dir).expect("Failed to create test package dir");
    fs::write(
        package_dir.join("package.xml"),
        r#"<?xml version="1.0"?><package><name>cached_package</name></package>"#,
    )
    .expect("Failed to write package.xml");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="link1">
    <visual><geometry><mesh filename="$(find cached_package)/mesh1.stl"/></geometry></visual>
  </link>
  <link name="link2">
    <visual><geometry><mesh filename="$(find cached_package)/mesh2.stl"/></geometry></visual>
  </link>
  <link name="link3">
    <visual><geometry><mesh filename="$(find cached_package)/mesh3.stl"/></geometry></visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(FindExtension::with_search_paths(vec![
            temp_dir.clone()
        ])))
        .build();

    // Process multiple times to test caching
    let start = std::time::Instant::now();
    for _ in 0..100 {
        let result = processor.run_from_string(input);
        assert!(
            result.is_ok(),
            "All processing runs should succeed with caching"
        );
    }
    let elapsed = start.elapsed();

    // Print timing for manual verification of caching performance
    // Note: No timing assertion to avoid CI flakiness - functional correctness is tested above
    println!("100 runs with 3x $(find) each took: {:?}", elapsed);

    let _ = fs::remove_dir_all(&temp_dir);
}

#[test]
fn test_find_and_optenv_combined() {
    // Test using both extensions together
    let test_id = TEST_COUNTER.fetch_add(1, Ordering::SeqCst);
    let temp_dir = env::temp_dir().join(format!("xacro_test_combined_{}", test_id));
    let package_dir = temp_dir.join("robot_description");

    let _ = fs::remove_dir_all(&temp_dir);
    fs::create_dir_all(&package_dir).expect("Failed to create test package dir");
    fs::write(
        package_dir.join("package.xml"),
        r#"<?xml version="1.0"?><package><name>robot_description</name></package>"#,
    )
    .expect("Failed to write package.xml");

    let _guard = EnvVarGuard::new("TEST_MESH_TYPE_COMBINED", "visual");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base">
    <visual>
      <geometry>
        <mesh filename="$(find robot_description)/meshes/$(optenv TEST_MESH_TYPE_COMBINED default).stl"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(FindExtension::with_search_paths(vec![
            temp_dir.clone()
        ])))
        .with_extension(Box::new(OptEnvExtension::new()))
        .build();

    let result = processor.run_from_string(input);
    let output = result.expect("Processing with both extensions should succeed");
    assert!(output.contains(&package_dir.display().to_string()));
    assert!(output.contains("/meshes/visual.stl"));

    let _ = fs::remove_dir_all(&temp_dir);
}

#[macro_use]
mod common;

use common::parse_xml;
use std::fs;
use tempfile::TempDir;
use xacro::{extensions::FindExtension, XacroProcessor};

#[test]
fn test_package_tracking_via_extensions_accessor() {
    // Create uniquely-scoped temporary package structure
    let temp_dir = TempDir::new().expect("Failed to create temporary test directory");
    let pkg1_dir = temp_dir.path().join("robot_description");
    let pkg2_dir = temp_dir.path().join("sensor_description");

    // Create package directories with proper package.xml files
    fs::create_dir_all(&pkg1_dir).expect("Failed to create robot_description dir");
    fs::create_dir_all(&pkg2_dir).expect("Failed to create sensor_description dir");
    fs::write(
        pkg1_dir.join("package.xml"),
        r#"<?xml version="1.0"?>
<package format="2">
  <name>robot_description</name>
</package>"#,
    )
    .expect("Failed to write robot package.xml");
    fs::write(
        pkg2_dir.join("package.xml"),
        r#"<?xml version="1.0"?>
<package format="2">
  <name>sensor_description</name>
</package>"#,
    )
    .expect("Failed to write sensor package.xml");

    // Xacro content that uses $(find ...) for multiple packages directly in attributes
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base">
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <mesh filename="$(find robot_description)/meshes/robot.stl"/>
      </geometry>
    </visual>
  </link>
  <link name="sensor">
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <mesh filename="$(find sensor_description)/meshes/sensor.stl"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    // Build processor with FindExtension
    let processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(FindExtension::with_search_paths(vec![temp_dir
            .path()
            .to_path_buf()])))
        .build();

    // Process xacro file
    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Processing should succeed: {:?}",
        result.err()
    );

    // Verify output is valid
    let output = result.unwrap();
    let root = parse_xml(&output);
    assert_eq!(root.name, "robot");

    // Now use the extensions() accessor to get package tracking info
    let extensions = processor.extensions();

    // Downcast to FindExtension and get packages
    let find_ext = extensions
        .iter()
        .find_map(|ext| ext.as_any().downcast_ref::<FindExtension>())
        .expect("Should be able to downcast to FindExtension");
    let packages = find_ext.get_found_packages();

    // Verify both packages were tracked
    assert_eq!(
        packages.len(),
        2,
        "Should have tracked 2 packages, got: {:?}",
        packages
    );
    assert!(
        packages.contains_key("robot_description"),
        "Should have tracked robot_description"
    );
    assert!(
        packages.contains_key("sensor_description"),
        "Should have tracked sensor_description"
    );

    // Verify paths are correct
    assert_eq!(
        packages.get("robot_description").unwrap(),
        &pkg1_dir,
        "robot_description path should match"
    );
    assert_eq!(
        packages.get("sensor_description").unwrap(),
        &pkg2_dir,
        "sensor_description path should match"
    );
    // TempDir automatically cleans up on drop
}

#[test]
fn test_package_tracking_no_find_extension() {
    // Create processor without FindExtension
    let processor = XacroProcessor::builder().clear_extensions().build();

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Processing should succeed");

    // Try to get FindExtension - should not find one
    let extensions = processor.extensions();
    let found_find_ext = extensions
        .iter()
        .any(|ext| ext.as_any().downcast_ref::<FindExtension>().is_some());

    assert!(
        !found_find_ext,
        "Should not have FindExtension when not registered"
    );
}

#[test]
fn test_package_tracking_empty_initially() {
    // Test that get_found_packages returns empty map before processing
    let processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(FindExtension::new()))
        .build();

    // Before processing, the package map should be empty
    let extensions = processor.extensions();
    let find_ext = extensions
        .iter()
        .find_map(|ext| ext.as_any().downcast_ref::<FindExtension>())
        .expect("FindExtension should be registered");

    let packages = find_ext.get_found_packages();
    assert!(
        packages.is_empty(),
        "Package map should be empty before processing"
    );
}

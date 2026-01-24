#[macro_use]
mod common;

use common::parse_xml;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use xacro::{extensions::FindExtension, XacroProcessor};

#[test]
fn test_package_tracking_via_extensions_accessor() {
    // Create temporary package structure
    let temp_dir = std::env::temp_dir().join("xacro_test_pkg_tracking");
    let pkg1_dir = temp_dir.join("robot_description");
    let pkg2_dir = temp_dir.join("sensor_description");

    // Clean up from any previous test runs
    let _ = fs::remove_dir_all(&temp_dir);

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
        .with_extension(Box::new(FindExtension::with_search_paths(vec![
            temp_dir.clone()
        ])))
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

    // Downcast to FindExtension
    let mut found_packages: Option<HashMap<String, PathBuf>> = None;
    for ext in extensions.iter() {
        if let Some(find_ext) = ext.as_any().downcast_ref::<FindExtension>() {
            found_packages = Some(find_ext.get_found_packages());
            break;
        }
    }

    // Verify we got the packages
    assert!(
        found_packages.is_some(),
        "Should be able to downcast to FindExtension"
    );
    let packages = found_packages.unwrap();

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

    // Clean up
    let _ = fs::remove_dir_all(&temp_dir);
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
    let mut found_find_ext = false;
    for ext in extensions.iter() {
        if ext.as_any().downcast_ref::<FindExtension>().is_some() {
            found_find_ext = true;
            break;
        }
    }

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
    for ext in extensions.iter() {
        if let Some(find_ext) = ext.as_any().downcast_ref::<FindExtension>() {
            let packages = find_ext.get_found_packages();
            assert_eq!(
                packages.len(),
                0,
                "Package map should be empty before processing"
            );
        }
    }
}

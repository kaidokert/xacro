#[macro_use]
mod common;

use common::{get_attr, parse_xml, EnvVarGuard};
use std::env;
use std::fs;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use xacro_rs::{extensions::*, XacroProcessor};

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
fn test_optenv_no_default_returns_empty() {
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
        result.is_ok(),
        "Processing should succeed with empty string"
    );

    let output = result.unwrap();
    let root = parse_xml(&output);
    let link = root.get_child("link").expect("Should have a link element");
    assert_eq!(get_attr(link, "name"), "");
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

    // Parse XML and check mesh filename attribute
    let root = parse_xml(&output);
    let mesh = root
        .get_child("link")
        .expect("Should have link")
        .get_child("visual")
        .expect("Should have visual")
        .get_child("geometry")
        .expect("Should have geometry")
        .get_child("mesh")
        .expect("Should have mesh");
    let filename = get_attr(mesh, "filename");

    // Verify resolved path contains package directory and mesh path
    let filename_path = Path::new(&filename);
    assert!(
        filename_path.starts_with(&package_dir),
        "Mesh filename should start with package directory"
    );
    let expected_suffix = Path::new("meshes").join("base.stl");
    assert!(
        filename_path.ends_with(&expected_suffix),
        "Mesh filename should end with meshes/base.stl"
    );

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
</robot>"#;

    let processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(FindExtension::with_search_paths(vec![
            temp_dir.clone()
        ])))
        .build();

    // 1. First run: should find the package and cache its path
    let result1 = processor.run_from_string(input);
    assert!(result1.is_ok(), "Initial processing should succeed");
    let output1 = result1.unwrap();

    // 2. Remove the package from the filesystem
    fs::remove_dir_all(&package_dir).expect("Failed to remove package dir");

    // 3. Second run: should still succeed by using the cached path
    let result2 = processor.run_from_string(input);
    assert!(
        result2.is_ok(),
        "Processing should succeed from cache after source is deleted"
    );
    let output2 = result2.unwrap();

    // Verify outputs are identical (cache returns same result)
    assert_eq!(output1, output2, "Cached result should match original");

    // 4. Create a new processor with fresh cache
    let new_processor = XacroProcessor::builder()
        .clear_extensions()
        .with_extension(Box::new(FindExtension::with_search_paths(vec![
            temp_dir.clone()
        ])))
        .build();

    // 5. With fresh cache and no source package, should fail
    let result3 = new_processor.run_from_string(input);
    assert!(
        result3.is_err(),
        "Processing should fail with fresh cache and no source package"
    );
    assert!(
        result3
            .unwrap_err()
            .to_string()
            .contains("Package not found"),
        "Error should indicate package not found"
    );

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

    // Parse XML and check mesh filename attribute
    let root = parse_xml(&output);
    let mesh = root
        .get_child("link")
        .expect("Should have link")
        .get_child("visual")
        .expect("Should have visual")
        .get_child("geometry")
        .expect("Should have geometry")
        .get_child("mesh")
        .expect("Should have mesh");
    let filename = get_attr(mesh, "filename");

    // Verify resolved path contains package directory and mesh path with optenv expansion
    let filename_path = Path::new(&filename);
    assert!(
        filename_path.starts_with(&package_dir),
        "Mesh filename should start with package directory"
    );
    let expected_suffix = Path::new("meshes").join("visual.stl");
    assert!(
        filename_path.ends_with(&expected_suffix),
        "Mesh filename should end with meshes/visual.stl"
    );

    let _ = fs::remove_dir_all(&temp_dir);
}

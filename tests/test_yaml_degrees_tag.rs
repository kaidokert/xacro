/// Test YAML !degrees custom tag support
///
/// Universal Robots xacro files use !degrees YAML tag to convert degree values to radians
/// Example: max_position: !degrees 360.0  should become 6.283185307179586 (2*pi radians)
///
mod common;
use crate::common::*;
use std::io::Write;
use tempfile::NamedTempFile;

#[test]
#[cfg(feature = "yaml")]
fn test_degrees_tag_converts_to_radians() {
    // Create YAML file with !degrees tag (used in UR robot joint limits)
    let yaml_content = r#"
joint_limits:
  shoulder_pan_joint:
    max_position: !degrees  360.0
    max_velocity: !degrees  180.0
    min_position: !degrees -360.0
"#;
    let mut yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    yaml_file
        .as_file_mut()
        .write_all(yaml_content.as_bytes())
        .expect("Failed to write YAML file");
    yaml_file
        .as_file_mut()
        .flush()
        .expect("Failed to flush YAML file");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="config" value="${{xacro.load_yaml('{}')}}" />
  <xacro:property name="limits" value="${{config['joint_limits']['shoulder_pan_joint']}}" />

  <!-- Should output radians, not degrees -->
  <joint name="shoulder_pan_joint" type="revolute">
    <limit lower="${{limits['min_position']}}"
           upper="${{limits['max_position']}}"
           velocity="${{limits['max_velocity']}}"/>
  </joint>
</robot>"#,
        yaml_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);
    let joint = find_child(&root, "joint");
    let limit = find_child(joint, "limit");

    // Python xacro converts !degrees 360.0 to radians: 6.283185307179586 (2*pi)
    let lower = get_attr(limit, "lower");
    let upper = get_attr(limit, "upper");
    let velocity = get_attr(limit, "velocity");

    // Expected values in radians
    // -360 degrees = -2*pi radians = -6.283185307179586
    //  360 degrees =  2*pi radians =  6.283185307179586
    //  180 degrees =    pi radians =  3.141592653589793
    assert!(
        lower.starts_with("-6.28"),
        "Expected lower limit in radians (~-6.28), got: {}",
        lower
    );
    assert!(
        upper.starts_with("6.28"),
        "Expected upper limit in radians (~6.28), got: {}",
        upper
    );
    assert!(
        velocity.starts_with("3.14"),
        "Expected velocity in radians (~3.14), got: {}",
        velocity
    );
}

#[test]
#[cfg(feature = "yaml")]
fn test_radians_tag_identity() {
    // Also test !radians tag (should be identity - no conversion)
    let yaml_content = r#"
joint_limits:
  elbow_joint:
    max_position: !radians  3.141592653589793
    min_position: !radians -3.141592653589793
"#;
    let mut yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    yaml_file
        .as_file_mut()
        .write_all(yaml_content.as_bytes())
        .expect("Failed to write YAML file");
    yaml_file
        .as_file_mut()
        .flush()
        .expect("Failed to flush YAML file");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="config" value="${{xacro.load_yaml('{}')}}" />
  <xacro:property name="limits" value="${{config['joint_limits']['elbow_joint']}}" />

  <joint name="elbow_joint" type="revolute">
    <limit lower="${{limits['min_position']}}"
           upper="${{limits['max_position']}}"/>
  </joint>
</robot>"#,
        yaml_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);
    let joint = find_child(&root, "joint");
    let limit = find_child(joint, "limit");

    let lower = get_attr(limit, "lower");
    let upper = get_attr(limit, "upper");

    // !radians should be identity (no conversion)
    assert!(
        lower.starts_with("-3.14"),
        "Expected lower limit unchanged (~-3.14), got: {}",
        lower
    );
    assert!(
        upper.starts_with("3.14"),
        "Expected upper limit unchanged (~3.14), got: {}",
        upper
    );
}

#[test]
#[cfg(feature = "yaml")]
fn test_degrees_tag_with_expressions() {
    // Test expression-based values like !degrees 45*2
    let yaml_content = r#"
joint_limits:
  test_joint:
    max_position: !degrees 45*2
    min_position: !degrees -45*2
"#;
    let mut yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    yaml_file
        .as_file_mut()
        .write_all(yaml_content.as_bytes())
        .expect("Failed to write YAML file");
    yaml_file
        .as_file_mut()
        .flush()
        .expect("Failed to flush YAML file");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="config" value="${{xacro.load_yaml('{}')}}" />
  <xacro:property name="limits" value="${{config['joint_limits']['test_joint']}}" />

  <joint name="test_joint" type="revolute">
    <limit lower="${{limits['min_position']}}"
           upper="${{limits['max_position']}}"/>
  </joint>
</robot>"#,
        yaml_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);
    let joint = find_child(&root, "joint");
    let limit = find_child(joint, "limit");

    let lower = get_attr(limit, "lower");
    let upper = get_attr(limit, "upper");

    // 45*2 = 90 degrees = pi/2 radians = 1.5707963267948966
    let lower_val: f64 = lower
        .parse()
        .unwrap_or_else(|_| panic!("Failed to parse lower '{}' as f64", lower));
    let upper_val: f64 = upper
        .parse()
        .unwrap_or_else(|_| panic!("Failed to parse upper '{}' as f64", upper));

    let expected = std::f64::consts::PI / 2.0; // 90 degrees in radians
    let epsilon = 1e-6;

    assert!(
        (lower_val + expected).abs() < epsilon,
        "Expected lower limit ≈ -π/2 ({}), got: {} (parsed: {})",
        -expected,
        lower,
        lower_val
    );
    assert!(
        (upper_val - expected).abs() < epsilon,
        "Expected upper limit ≈ π/2 ({}), got: {} (parsed: {})",
        expected,
        upper,
        upper_val
    );
}

#[test]
#[cfg(feature = "yaml")]
fn test_length_unit_tags() {
    // Test all length unit tags: !meters, !millimeters, !foot, !inches
    let yaml_content = r#"
dimensions:
  length_m: !meters 1.5
  length_mm: !millimeters -2250.0
  length_ft: !foot 3
  length_in: !inches 4.0
"#;
    let mut yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    yaml_file
        .as_file_mut()
        .write_all(yaml_content.as_bytes())
        .expect("Failed to write YAML file");
    yaml_file
        .as_file_mut()
        .flush()
        .expect("Failed to flush YAML file");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="dims" value="${{xacro.load_yaml('{}')}}" />

  <link name="test">
    <visual>
      <geometry>
        <box size="${{dims['dimensions']['length_m']}} ${{dims['dimensions']['length_mm']}} ${{dims['dimensions']['length_ft']}}"/>
      </geometry>
      <origin xyz="${{dims['dimensions']['length_in']}} 0 0"/>
    </visual>
  </link>
</robot>"#,
        yaml_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");
    let origin = find_child(visual, "origin");

    let size = get_attr(box_elem, "size");
    let xyz = get_attr(origin, "xyz");

    // Expected conversions:
    // !meters 1.5      → 1.5 (identity)
    // !millimeters -2250.0 → -2.25 (×0.001)
    // !foot 3          → 0.9144 (×0.3048)
    // !inches 4.0      → 0.1016 (×0.0254)

    let size_parts: Vec<f64> = size
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    let xyz_parts: Vec<f64> = xyz.split_whitespace().map(|s| s.parse().unwrap()).collect();

    let epsilon = 1e-6;

    assert!(
        (size_parts[0] - 1.5).abs() < epsilon,
        "Expected !meters 1.5 → 1.5, got: {}",
        size_parts[0]
    );
    assert!(
        (size_parts[1] - (-2.25)).abs() < epsilon,
        "Expected !millimeters -2250.0 → -2.25, got: {}",
        size_parts[1]
    );
    assert!(
        (size_parts[2] - 0.9144).abs() < epsilon,
        "Expected !foot 3 → 0.9144, got: {}",
        size_parts[2]
    );
    assert!(
        (xyz_parts[0] - 0.1016).abs() < epsilon,
        "Expected !inches 4.0 → 0.1016, got: {}",
        xyz_parts[0]
    );
}

#[test]
#[cfg(feature = "yaml")]
fn test_unknown_yaml_tag_passthrough() {
    // Test that unknown YAML tags pass through value unchanged
    let yaml_content = r#"
test_values:
  unknown_numeric: !unknown_unit 123.456
  unknown_string: !custom_tag test_value
"#;
    let mut yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    yaml_file
        .as_file_mut()
        .write_all(yaml_content.as_bytes())
        .expect("Failed to write YAML file");
    yaml_file
        .as_file_mut()
        .flush()
        .expect("Failed to flush YAML file");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="vals" value="${{xacro.load_yaml('{}')}}" />

  <link name="test">
    <inertial>
      <mass value="${{vals['test_values']['unknown_numeric']}}"/>
    </inertial>
  </link>
  <link name="${{vals['test_values']['unknown_string']}}"/>
</robot>"#,
        yaml_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);

    // First link should have mass with the numeric value unchanged
    let link1 = find_child(&root, "link");
    let inertial = find_child(link1, "inertial");
    let mass = find_child(inertial, "mass");
    let mass_value = get_attr(mass, "value");

    assert!(
        mass_value.starts_with("123.45"),
        "Expected unknown tag value to be passed through (~123.45), got: {}",
        mass_value
    );

    // Find second link by checking all links
    let links: Vec<_> = root
        .children
        .iter()
        .filter_map(|n| n.as_element())
        .filter(|n| n.name == "link")
        .collect();
    assert_eq!(links.len(), 2, "Expected 2 links in output");

    // Second link should have the string value unchanged
    let link2_name = get_attr(links[1], "name");
    assert_eq!(
        link2_name, "test_value",
        "Expected unknown tag string to be passed through, got: {}",
        link2_name
    );
}

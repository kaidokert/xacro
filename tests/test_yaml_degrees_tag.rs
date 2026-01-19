/// Test YAML !degrees custom tag support
///
/// Universal Robots xacro files use !degrees YAML tag to convert degree values to radians
/// Example: max_position: !degrees 360.0  should become 6.283185307179586 (2*pi radians)
///
/// Bug: https://github.com/kaidokert/xacro/issues/XXX
mod common;
use crate::common::*;
use std::fs;
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
    let yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    fs::write(yaml_file.path(), yaml_content).expect("Failed to write YAML file");

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
    let yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    fs::write(yaml_file.path(), yaml_content).expect("Failed to write YAML file");

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

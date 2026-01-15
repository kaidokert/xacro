mod common;
use common::*;

/// Test that macro parameters can use = or := for default values
#[test]
fn test_macro_default_param_with_equals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_width" value="0.5"/>

  <xacro:macro name="test_box" params="length width=${default_width}">
    <link name="box">
      <visual>
        <geometry>
          <box size="${length} ${width} 0.1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:test_box length="1.0"/>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Default width should be used
    assert_eq!(get_attr(box_elem, "size"), "1.0 0.5 0.1");
}

/// Test that := syntax also works (original syntax)
#[test]
fn test_macro_default_param_with_colon_equals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_width" value="0.5"/>

  <xacro:macro name="test_box" params="length width:=${default_width}">
    <link name="box">
      <visual>
        <geometry>
          <box size="${length} ${width} 0.1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:test_box length="1.0"/>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Default width should be used
    assert_eq!(get_attr(box_elem, "size"), "1.0 0.5 0.1");
}

/// Test overriding default parameter
#[test]
fn test_macro_default_param_override() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_width" value="0.5"/>

  <xacro:macro name="test_box" params="length width=${default_width}">
    <link name="box">
      <visual>
        <geometry>
          <box size="${length} ${width} 0.1"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:test_box length="1.0" width="2.0"/>
</robot>"#;

    let root = run_xacro_to_xml(input);
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let geometry = find_child(visual, "geometry");
    let box_elem = find_child(geometry, "box");

    // Explicit width should override default
    assert_eq!(get_attr(box_elem, "size"), "1.0 2.0 0.1");
}

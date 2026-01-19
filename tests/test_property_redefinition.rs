/// Property redefinition tests
///
/// Tests that empty property redefinitions (<xacro:property name="foo"/>)
/// don't overwrite existing value properties.
mod common;
use crate::common::*;

#[test]
fn test_property_redefinition_preserves_value() {
    // Corpus case: property defined with value, then redefined without value
    // The second definition should NOT overwrite the first
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="mass" value="0.5"/>
  <xacro:property name="mass"/>

  <link name="test">
    <inertial>
      <mass value="${mass}"/>
    </inertial>
  </link>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let link = find_child(&root, "link");
    let inertial = find_child(link, "inertial");
    let mass = find_child(inertial, "mass");
    assert_eq!(get_attr(mass, "value"), "0.5");
}

#[test]
fn test_property_redefinition_with_arithmetic() {
    // Properties should preserve their numeric type through redefinition
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="mass" value="0.5"/>
  <xacro:property name="radius" value="0.1"/>
  <xacro:property name="mass"/>

  <link name="test">
    <inertial>
      <mass value="${mass}"/>
      <inertia ixx="${(2/5)*mass*radius*radius}"/>
    </inertial>
  </link>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let link = find_child(&root, "link");
    let inertial = find_child(link, "inertial");
    let mass = find_child(inertial, "mass");
    let inertia = find_child(inertial, "inertia");

    assert_eq!(get_attr(mass, "value"), "0.5");
    assert_attr_float!(inertia, "ixx", 0.002, 1e-10);
}

#[test]
fn test_property_redefinition_in_included_file() {
    use tempfile::Builder;

    // Complex corpus case: property defined and redefined in included file,
    // then used in macro expansion

    // Create temporary files with .xacro extension for proper testing
    let included_file = Builder::new()
        .prefix("test_property_redef_included_")
        .suffix(".xacro")
        .tempfile()
        .expect("Failed to create temp file");

    std::fs::write(
        included_file.path(),
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="wheel_mass" value="0.5"/>
  <xacro:property name="wheel_radius" value="0.125"/>
  <xacro:property name="wheel_mass"/>

  <xacro:macro name="wheel" params="name">
    <link name="${name}">
      <inertial>
        <mass value="${wheel_mass}"/>
        <inertia ixx="${(2/5)*wheel_mass*wheel_radius*wheel_radius}"/>
      </inertial>
    </link>
  </xacro:macro>
</robot>"#,
    )
    .expect("Failed to write test file");

    let main_file = Builder::new()
        .prefix("test_property_redef_main_")
        .suffix(".xacro")
        .tempfile()
        .expect("Failed to create temp file");

    std::fs::write(
        main_file.path(),
        format!(
            r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="{}"/>
  <xacro:wheel name="test_wheel"/>
</robot>"#,
            included_file.path().display()
        ),
    )
    .expect("Failed to write test file");

    let output = run_xacro_file(main_file.path());
    let root = parse_xml(&output);

    let link = find_child(&root, "link");
    let inertial = find_child(link, "inertial");
    let mass = find_child(inertial, "mass");
    let inertia = find_child(inertial, "inertia");

    assert_eq!(get_attr(mass, "value"), "0.5");
    assert_attr_float!(inertia, "ixx", 0.003125, 1e-10);

    // Temp files automatically cleaned up when dropped (RAII)
}

#[test]
fn test_empty_lazy_property_not_confused_with_value() {
    // Empty lazy block properties (stored as **name) should not conflict
    // with regular value properties (stored as name)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="mass" value="1.0"/>
  <xacro:property name="empty"></xacro:property>

  <test>
    <mass value="${mass}"/>
    <xacro:insert_block name="empty"/>
  </test>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);

    let test = find_child(&root, "test");
    let mass = find_child(test, "mass");
    assert_eq!(get_attr(mass, "value"), "1.0");
}

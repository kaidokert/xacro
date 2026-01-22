//! Property-related integration tests
//!
//! Tests for property scope, lazy evaluation, redefinition, and usage in expressions.

mod common;
use crate::common::*;

// ============================================================================
// Property in Macro Conditional Tests
// ============================================================================

/// Test property defined inside macro body, used in conditional
///
/// Pattern: property inside macro references macro parameter, then used in conditional
#[test]
fn test_property_inside_macro_used_in_conditional() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="var" value="test"/>

  <xacro:macro name="arm" params="prefix">
    <xacro:property name="prefix_" value="${prefix}_"/>

    <xacro:if value="${var == 'test'}">
      <link name="${prefix_}link"/>
    </xacro:if>
  </xacro:macro>

  <xacro:arm prefix="left"/>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Property inside macro should be available in conditional",
    );
    assert_xacro_contains!(
        output,
        r#"name="left_link""#,
        "Expected link name='left_link' in output"
    );
}

/// Simplified case: property in macro without conditional
#[test]
fn test_property_inside_macro_simple() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="prefix">
    <xacro:property name="prefix_" value="${prefix}_"/>
    <link name="${prefix_}link"/>
  </xacro:macro>

  <xacro:test prefix="my"/>
</robot>"#;

    let output = run_xacro_expect(input, "Property inside macro should work");
    assert_xacro_contains!(
        output,
        r#"name="my_link""#,
        "Expected link name='my_link' in output"
    );
}

/// Test the actual failing case from xurdf/data/sample.xacro
#[test]
fn test_xurdf_sample_property_in_macro() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="var" value="test"/>

  <xacro:macro name="arm" params="prefix:='' parent reflect:=1">
    <xacro:property name="prefix_" value="${prefix}_" />

    <upperarm prefix="${prefix}" reflect="${reflect}" parent="${parent}" />

    <xacro:if value="${var == 'test'}">
      <forearm prefix="${prefix}" reflect="${reflect}" parent="${prefix_}elbow" />
    </xacro:if>
  </xacro:macro>

  <xacro:arm prefix="left" reflect="1" parent="torso" />
  <xacro:arm prefix="right" reflect="-1" parent="torso" />
</robot>"#;

    let output = run_xacro_expect(input, "xurdf sample should process without error");
    assert_xacro_contains!(output, r#"prefix="left""#, "Missing left arm");
    assert_xacro_contains!(output, r#"prefix="right""#, "Missing right arm");
    assert_xacro_contains!(output, r#"parent="left_elbow""#, "Missing left_elbow");
    assert_xacro_contains!(output, r#"parent="right_elbow""#, "Missing right_elbow");
}

/// Test property defined inside macro body, used in conditional with non-standard prefix
///
/// This is the same as test_property_inside_macro_used_in_conditional but uses
/// xmlns:foo="..." instead of xmlns:xacro="..." to verify alias detection works
/// in this scenario.
#[test]
fn test_property_inside_macro_used_in_conditional_with_alias_prefix() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:foo="http://www.ros.org/wiki/xacro">
  <foo:property name="var" value="test"/>

  <foo:macro name="arm" params="prefix">
    <foo:property name="prefix_" value="${prefix}_"/>

    <foo:if value="${var == 'test'}">
      <link name="${prefix_}link"/>
    </foo:if>
  </foo:macro>

  <foo:arm prefix="left"/>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Property inside macro should be available in conditional with alias prefix",
    );
    assert_xacro_contains!(
        output,
        r#"name="left_link""#,
        "Expected link name='left_link' in output"
    );
    assert_xacro_not_contains!(
        output,
        "xmlns:foo",
        "Alias xacro prefix should be removed from output"
    );
    assert_xacro_not_contains!(
        output,
        "foo:",
        "No foo: prefixed elements should remain in output"
    );
}

// ============================================================================
// Property Redefinition Tests
// ============================================================================

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

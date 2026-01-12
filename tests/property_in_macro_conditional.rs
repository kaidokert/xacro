mod common;
use crate::common::*;

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

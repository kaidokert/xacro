use xacro::XacroProcessor;

/// Test property defined inside macro body, used in conditional
///
/// This is Sample 2 from UNDEFINED_VAR_SAMPLES.md
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Property inside macro should be available in conditional. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="left_link""#),
        "Expected link name='left_link' in output:\n{}",
        output
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Property inside macro should work. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="my_link""#),
        "Expected link name='my_link' in output:\n{}",
        output
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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "xurdf sample should process without error. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // Check both macro expansions worked
    assert!(output.contains(r#"prefix="left""#), "Missing left arm");
    assert!(output.contains(r#"prefix="right""#), "Missing right arm");
    assert!(
        output.contains(r#"parent="left_elbow""#),
        "Missing left_elbow"
    );
    assert!(
        output.contains(r#"parent="right_elbow""#),
        "Missing right_elbow"
    );
}

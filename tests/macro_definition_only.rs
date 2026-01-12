mod common;
use crate::common::*;

/// Test that macro definitions (without calls) don't trigger undefined variable errors
///
/// This is a CRITICAL test - if a file only defines a macro but never calls it,
/// the macro parameters should never be evaluated during definition.
#[test]
fn test_macro_definition_without_call() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="name">
    <link name="${name}"/>
  </xacro:macro>
</robot>"#;

    let output = run_xacro_expect(input, "Macro definition should not evaluate parameters");

    // Output should be empty robot (macro definition removed, no expansion)
    assert_xacro_not_contains!(output, "link", "Should not contain any links");
    assert_xacro_contains!(output, "<robot", "Should have robot element");
}

/// Test the actual failing case from PR2
#[test]
fn test_pr2_head_gazebo_macro_only() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
       xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
       xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="pr2_head_gazebo_v0" params="name">
    <gazebo reference="${name}_plate_frame">
      <material value="Gazebo/Grey" />
    </gazebo>
  </xacro:macro>

</robot>"#;

    let _output = run_xacro_expect(
        input,
        "PR2 head gazebo macro (definition only) should process without error",
    );
}

/// Test macro with parameter used in attribute value
#[test]
fn test_macro_definition_param_in_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="prefix">
    <link name="${prefix}_link"/>
  </xacro:macro>
</robot>"#;

    let _output = run_xacro_expect(
        input,
        "Macro param in attribute should not be evaluated during definition",
    );
}

/// Test macro with parameter used in text node
#[test]
fn test_macro_definition_param_in_text() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="value">
    <item>${value}</item>
  </xacro:macro>
</robot>"#;

    let _output = run_xacro_expect(
        input,
        "Macro param in text node should not be evaluated during definition",
    );
}

/// Test macro with complex expression using parameter
#[test]
fn test_macro_definition_param_in_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="size">
    <box size="${size * 2} ${size} ${size / 2}"/>
  </xacro:macro>
</robot>"#;

    let _output = run_xacro_expect(
        input,
        "Macro param in expression should not be evaluated during definition",
    );
}

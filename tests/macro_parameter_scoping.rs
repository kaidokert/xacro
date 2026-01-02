use xacro::XacroProcessor;

/// Integration test for macro parameter scoping bug
///
/// This test uses the FULL XacroProcessor pipeline (not just MacroProcessor),
/// which runs PropertyProcessor BEFORE MacroProcessor. This exposes the bug
/// where PropertyProcessor tries to evaluate ${name} inside macro definitions
/// before the macro parameters are defined.
#[test]
fn test_macro_with_parameters_full_pipeline() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="box_link" params="name size">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${size} ${size} ${size}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:box_link name="base_link" size="0.5"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // This SHOULD succeed but currently fails with:
    // "Evaluation error in '${name}': Undefined variable: name"
    assert!(
        result.is_ok(),
        "Macro parameters should be evaluated during expansion, not definition. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Verify the macro was expanded with correct parameter values
    assert!(
        output.contains(r#"name="base_link""#),
        "Macro should expand with name parameter"
    );
    assert!(
        output.contains(r#"0.5 0.5 0.5"#),
        "Macro should expand with size parameter"
    );

    // Verify no macro elements remain
    assert!(
        !output.contains("xacro:macro"),
        "Macro definition should be removed"
    );
    assert!(
        !output.contains("xacro:box_link"),
        "Macro call should be removed"
    );
}

/// Test that global properties work inside macros
#[test]
fn test_macro_with_global_property_full_pipeline() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="global_size" value="0.5"/>

  <xacro:macro name="box_link" params="name">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${global_size} ${global_size} ${global_size}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:box_link name="base_link"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Global properties should be accessible inside macros. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains(r#"name="base_link""#));
    assert!(output.contains(r#"0.5 0.5 0.5"#));
}

/// Test mixed macro parameters and global properties
#[test]
fn test_macro_params_override_globals_full_pipeline() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="size" value="1.0"/>

  <xacro:macro name="box_link" params="name size">
    <link name="${name}">
      <visual>
        <geometry>
          <!-- Macro param 'size' should override global property 'size' -->
          <box size="${size} ${size} ${size}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:box_link name="base_link" size="0.5"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Macro parameters should override global properties. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains(r#"name="base_link""#));
    // Should use macro param size=0.5, NOT global property size=1.0
    assert!(
        output.contains(r#"0.5 0.5 0.5"#),
        "Macro parameter should override global property"
    );
    assert!(
        !output.contains(r#"1.0 1.0 1.0"#),
        "Should not use global property value"
    );
}

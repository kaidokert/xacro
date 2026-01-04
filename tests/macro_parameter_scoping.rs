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

/// Test property defined inside macro that references macro parameter
#[test]
fn test_property_inside_macro_referencing_parameter() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="link" params="name">
    <xacro:property name="full_name" value="link_${name}"/>
    <link name="${full_name}"/>
  </xacro:macro>

  <xacro:link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    let output = result.expect("Property inside macro should reference macro parameter");
    assert!(
        output.contains(r#"name="link_base""#),
        "Property should have been evaluated with macro parameter"
    );
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

/// Test macro parameter interdependencies (one default referencing another parameter)
///
/// This validates our 2-phase parameter resolution where defaults can reference
/// previously resolved parameters in the cumulative evaluation context.
#[test]
fn test_macro_parameter_dependency() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="a b:=${a*2}">
    <item a="${a}" b="${b}"/>
  </xacro:macro>

  <xacro:foo a="5"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Parameter default should be able to reference another parameter. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Verify 'a' is 5 and 'b' is 10 (a*2)
    assert!(output.contains(r#"a="5""#), "Parameter 'a' should be 5");
    assert!(
        output.contains(r#"b="10""#),
        "Parameter 'b' should be 10 (a*2 where a=5)"
    );
}

/// Test chained parameter defaults (default depends on another optional parameter)
///
/// Validates that when multiple parameters have defaults, they are resolved in
/// declaration order with each default able to reference previously resolved parameters.
#[test]
fn test_macro_chained_parameter_defaults() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="a:=1 b:=${a*2} c:=${b*3}">
    <item a="${a}" b="${b}" c="${c}"/>
  </xacro:macro>

  <xacro:foo/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Chained parameter defaults should resolve in declaration order. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Verify: a=1 (default), b=2 (1*2), c=6 (2*3)
    assert!(
        output.contains(r#"a="1""#),
        "Parameter 'a' should be 1 (default)"
    );
    assert!(
        output.contains(r#"b="2""#),
        "Parameter 'b' should be 2 (a*2 where a=1)"
    );
    assert!(
        output.contains(r#"c="6""#),
        "Parameter 'c' should be 6 (b*3 where b=2)"
    );
}

/// Test that missing required parameter in default expression produces clear error
///
/// When a parameter default references another parameter that has no default
/// and isn't provided at the call site, we should get MissingParameter error.
#[test]
fn test_macro_missing_parameter_in_default() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="a b:=${a*2}">
    <item a="${a}" b="${b}"/>
  </xacro:macro>

  <xacro:foo/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_err(),
        "Should error when required parameter 'a' is missing"
    );

    let err = result.err().unwrap();
    let err_str = format!("{:?}", err);

    // Should get MissingParameter error for 'a', not treat it as falsy/zero
    assert!(
        err_str.contains("MissingParameter") || err_str.contains("parameter"),
        "Error should indicate missing parameter 'a', got: {}",
        err_str
    );
}

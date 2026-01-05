use xacro::XacroProcessor;

/// Test that macro calls can use parameter expressions in their attributes
///
/// This is a critical pattern used in real-world files like ridgeback.urdf.xacro:
///
/// ```xml
/// <xacro:macro name="outer" params="prefix">
///   <xacro:inner prefix="${prefix}"/>  <!-- ${prefix} must be evaluated in outer's scope -->
/// </xacro:macro>
/// ```
#[test]
fn test_nested_macro_with_parameter_expression_in_call() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Inner macro that takes a prefix parameter -->
  <xacro:macro name="wheel" params="prefix side">
    <link name="${prefix}_${side}_wheel"/>
  </xacro:macro>

  <!-- Outer macro that calls inner macro with expression in attribute -->
  <xacro:macro name="rocker" params="prefix">
    <link name="${prefix}_rocker"/>
    <xacro:wheel prefix="${prefix}" side="left"/>
    <xacro:wheel prefix="${prefix}" side="right"/>
  </xacro:macro>

  <!-- Call outer macro -->
  <xacro:rocker prefix="front"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Nested macro with parameter expression in call should work: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Should expand outer macro with prefix="front"
    assert!(
        output.contains(r#"name="front_rocker""#),
        "Outer macro should expand with prefix"
    );

    // Inner macro calls should receive evaluated prefix value
    assert!(
        output.contains(r#"name="front_left_wheel""#),
        "Inner macro should receive evaluated prefix parameter"
    );
    assert!(
        output.contains(r#"name="front_right_wheel""#),
        "Inner macro should receive evaluated prefix parameter"
    );
}

/// Test with multiple levels of nesting
#[test]
fn test_triple_nested_macro_with_parameter_expressions() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Innermost macro -->
  <xacro:macro name="component" params="name">
    <link name="${name}_component"/>
  </xacro:macro>

  <!-- Middle macro -->
  <xacro:macro name="assembly" params="prefix side">
    <xacro:component name="${prefix}_${side}"/>
  </xacro:macro>

  <!-- Outermost macro -->
  <xacro:macro name="system" params="prefix">
    <xacro:assembly prefix="${prefix}" side="left"/>
    <xacro:assembly prefix="${prefix}" side="right"/>
  </xacro:macro>

  <!-- Call top-level macro -->
  <xacro:system prefix="robot"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Triple nested macros with parameter expressions should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="robot_left_component""#),
        "Triple nesting should pass parameters through all levels"
    );
    assert!(
        output.contains(r#"name="robot_right_component""#),
        "Triple nesting should pass parameters through all levels"
    );
}

/// Test that parameters can be used in complex expressions in nested calls
#[test]
fn test_nested_macro_with_arithmetic_expression_in_call() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="box" params="size">
    <box size="${size} ${size} ${size}"/>
  </xacro:macro>

  <xacro:macro name="scaled_box" params="base_size scale">
    <xacro:box size="${base_size * scale}"/>
  </xacro:macro>

  <xacro:scaled_box base_size="1.0" scale="2.5"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Nested macro with arithmetic expression in call should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("2.5 2.5 2.5"),
        "Arithmetic expression in nested macro call should be evaluated"
    );
}

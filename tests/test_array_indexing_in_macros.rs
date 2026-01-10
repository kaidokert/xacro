use xacro::XacroProcessor;

/// Test that macro parameters containing arrays/lists can be indexed
///
/// This test covers the bug fix where macro parameters were stored as raw strings
/// instead of being evaluated first, causing array indexing to fail.
///
/// Real-world example from tiptap_xacro.xml:
/// ```xml
/// <xacro:property name="torso_xyzm" value="${[0.067, 0.023, 0.128, 0.1556]}"/>
/// <xacro:macro name="cuboid" params="xyzm">
///   <xacro:property name="x" value="${xyzm[0]}"/>
///   <xacro:property name="y" value="${xyzm[1]}"/>
///   <xacro:property name="z" value="${xyzm[2]}"/>
///   <xacro:property name="m" value="${xyzm[3]}"/>
/// </xacro:macro>
/// <xacro:cuboid xyzm="${torso_xyzm}"/>
/// ```
#[test]
fn test_array_parameter_indexing() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="my_list" value="${[1, 2, 3]}"/>

  <xacro:macro name="test_macro" params="arr">
    <link name="test">
      <value>${arr[0]}</value>
    </link>
  </xacro:macro>

  <xacro:test_macro arr="${my_list}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Array indexing in macro parameters should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<value>1</value>"#),
        "Array indexing should extract first element, got: {}",
        output
    );
}

/// Test array indexing with multiple elements
#[test]
fn test_array_parameter_multiple_indices() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="dimensions" value="${[0.5, 1.2, 0.8]}"/>

  <xacro:macro name="box" params="size">
    <geometry>
      <box size="${size[0]} ${size[1]} ${size[2]}"/>
    </geometry>
  </xacro:macro>

  <xacro:box size="${dimensions}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Multiple array indices should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"size="0.5 1.2 0.8""#),
        "Should extract all array elements, got: {}",
        output
    );
}

/// Test that array parameters work with arithmetic expressions
#[test]
fn test_array_parameter_with_arithmetic() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base_dims" value="${[1.0, 2.0, 3.0]}"/>

  <xacro:macro name="scaled_box" params="dims scale">
    <box x="${dims[0] * scale}"
         y="${dims[1] * scale}"
         z="${dims[2] * scale}"/>
  </xacro:macro>

  <xacro:scaled_box dims="${base_dims}" scale="2"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Array indexing with arithmetic should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"x="2""#),
        "Should scale first element: 1.0 * 2 = 2"
    );
    assert!(
        output.contains(r#"y="4""#),
        "Should scale second element: 2.0 * 2 = 4"
    );
    assert!(
        output.contains(r#"z="6""#),
        "Should scale third element: 3.0 * 2 = 6"
    );
}

/// Test nested array parameter passing
#[test]
fn test_nested_macro_with_array_parameter() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="color_rgb" value="${[1.0, 0.0, 0.5]}"/>

  <xacro:macro name="colored_component" params="rgb">
    <material>
      <color rgba="${rgb[0]} ${rgb[1]} ${rgb[2]} 1.0"/>
    </material>
  </xacro:macro>

  <xacro:macro name="part" params="color">
    <visual>
      <xacro:colored_component rgb="${color}"/>
    </visual>
  </xacro:macro>

  <xacro:part color="${color_rgb}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Nested macros with array parameters should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"rgba="1.0 0.0 0.5 1.0""#),
        "Array should pass through nested macro calls, got: {}",
        output
    );
}

/// Test that inline array literals work as macro parameters
#[test]
fn test_inline_array_literal_parameter() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="positioned_link" params="coords">
    <link name="test">
      <origin xyz="${coords[0]} ${coords[1]} ${coords[2]}"/>
    </link>
  </xacro:macro>

  <!-- Pass array literal directly, not via property -->
  <xacro:positioned_link coords="${[1.5, 2.0, 3.5]}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Inline array literal as parameter should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"xyz="1.5 2.0 3.5""#),
        "Should accept inline array literal, got: {}",
        output
    );
}

/// Test real-world pattern: multiple values packed in array parameter
///
/// This pattern is used in tiptap_xacro.xml where dimensions and mass
/// are packed together: [width, height, depth, mass]
#[test]
fn test_real_world_packed_array_parameter() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Pack related values together -->
  <xacro:property name="total_mass" value="0.778"/>
  <xacro:property name="torso_xyzm" value="${[0.067, 0.023, 0.128, total_mass * 0.2]}"/>

  <xacro:macro name="cuboid" params="xyzm">
    <link name="torso">
      <visual>
        <geometry>
          <box size="${xyzm[0]} ${xyzm[1]} ${xyzm[2]}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="${xyzm[3]}"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:cuboid xyzm="${torso_xyzm}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Real-world packed array pattern should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"size="0.067 0.023 0.128""#),
        "Should extract dimensions from array"
    );
    // Note: 0.778 * 0.2 = 0.15560000000000002 due to floating point precision
    assert!(
        output.contains(r#"value="0.1556"#),
        "Should extract computed mass starting with 0.1556, got: {}",
        output
    );
}

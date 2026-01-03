use xacro::{XacroError, XacroProcessor};

#[test]
fn test_xacro_namespace_removed() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="wheel_radius" value="0.1"/>
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor.run_from_string(input).unwrap();

    // Should not contain xmlns:xacro anywhere
    assert!(
        !output.contains("xmlns:xacro"),
        "Output should not contain xmlns:xacro"
    );

    // Should be valid XML
    let parsed = xmltree::Element::parse(output.as_bytes());
    assert!(parsed.is_ok(), "Output should be valid XML");
}

#[test]
fn test_other_namespaces_preserved() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       xmlns:gazebo="http://gazebo.org/schema"
       xmlns:ignition="http://ignitionrobotics.org/schema"
       name="test">
  <xacro:property name="wheel_radius" value="0.1"/>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <ignition:plugin filename="libMyPlugin.so" name="my_plugin">
    <param>value</param>
  </ignition:plugin>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor.run_from_string(input).unwrap();

    // xacro namespace should be removed
    assert!(
        !output.contains("xmlns:xacro"),
        "xacro namespace should be removed"
    );

    // Other namespaces should be preserved
    assert!(
        output.contains("xmlns:gazebo"),
        "gazebo namespace should be preserved"
    );
    assert!(
        output.contains("xmlns:ignition"),
        "ignition namespace should be preserved"
    );

    // Elements using those namespaces should be present
    assert!(
        output.contains("<gazebo"),
        "gazebo element should be present"
    );
    assert!(
        output.contains("ignition:plugin"),
        "ignition:plugin should be present"
    );

    // Should be valid XML
    let parsed = xmltree::Element::parse(output.as_bytes());
    assert!(
        parsed.is_ok(),
        "Output should be valid XML: {:?}",
        parsed.err()
    );
}

#[test]
fn test_unimplemented_feature_detection_arg() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:arg name="robot_name" default="my_robot"/>
  <link name="base_link"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(result.is_err(), "Should error on unimplemented xacro:arg");

    let err = result.unwrap_err().to_string();
    assert!(err.contains("xacro:arg"), "Error should mention xacro:arg");
    assert!(
        err.contains("not implemented"),
        "Error should say not implemented"
    );
}

#[test]
fn test_unimplemented_feature_detection_element() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:element xacro:name="link" name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_err(),
        "Should error on unimplemented xacro:element"
    );

    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("xacro:element"),
        "Error should mention xacro:element"
    );
}

#[test]
fn test_unimplemented_feature_detection_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:attribute name="some_attr" value="some_value"/>
  <link name="base_link"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_err(),
        "Should error on unimplemented xacro:attribute"
    );

    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("xacro:attribute"),
        "Error should mention xacro:attribute"
    );
    assert!(
        err.contains("not implemented"),
        "Error should say not implemented"
    );
}

#[test]
fn test_no_invalid_xml_from_unprocessed_elements() {
    // Simulates a bug where processor fails to remove a xacro:* element
    // The finalize_tree should catch this and error instead of producing invalid XML

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:unknown_feature/>
  <link name="base_link"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should error, not produce invalid XML
    assert!(result.is_err(), "Should error on unknown xacro element");

    // Verify it errors (either as undefined macro or unprocessed element)
    // Unknown xacro elements are caught by MacroProcessor as undefined macros
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("unknown_feature") || err.contains("This element was not processed"),
        "Error should mention the unknown xacro element: {}",
        err
    );
}

#[test]
fn test_implemented_features_work_without_macros() {
    // Test implemented features that don't hit the macro parameter scoping bug
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="length" value="0.5"/>

  <link name="my_link">
    <visual>
      <geometry>
        <box size="${length} 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <xacro:if value="true">
    <link name="conditional_link"/>
  </xacro:if>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor.run_from_string(input).unwrap();

    // Should process successfully
    assert!(output.contains("my_link"), "Link should be present");
    assert!(
        output.contains("conditional_link"),
        "Conditional should be processed"
    );
    assert!(output.contains("0.5"), "Property should be substituted");

    // xacro namespace should be removed
    assert!(
        !output.contains("xmlns:xacro"),
        "xacro namespace should be removed"
    );

    // No xacro:* elements should remain
    assert!(
        !output.contains("xacro:property"),
        "xacro:property should be removed"
    );
    assert!(!output.contains("xacro:if"), "xacro:if should be removed");

    // Verify output is valid XML (consistent with other tests)
    let parsed = xmltree::Element::parse(output.as_bytes());
    assert!(
        parsed.is_ok(),
        "Output should be valid XML: {:?}",
        parsed.err()
    );
}

#[test]
fn test_implemented_features_work_with_macros() {
    // Test that macros with parameters work correctly after parameter scoping bug fix
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       xmlns:gazebo="http://gazebo.org/schema"
       name="test">
  <xacro:property name="length" value="0.5"/>

  <xacro:macro name="box_link" params="name size">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${size} ${size} ${size}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:box_link name="macro_link" size="${length}"/>

  <gazebo reference="macro_link">
    <material>Gazebo/Blue</material>
  </gazebo>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor.run_from_string(input).unwrap();

    // Should process successfully with macros
    assert!(
        output.contains(r#"name="macro_link""#),
        "Macro should expand with name parameter"
    );
    assert!(
        output.contains(r#"0.5 0.5 0.5"#),
        "Macro should expand with size parameter (using global property)"
    );

    // xacro namespace should be removed
    assert!(
        !output.contains("xmlns:xacro"),
        "xacro namespace should be removed"
    );

    // Other namespaces should be preserved
    assert!(
        output.contains("xmlns:gazebo"),
        "gazebo namespace should be preserved"
    );

    // No xacro:* elements should remain
    assert!(
        !output.contains("xacro:property"),
        "xacro:property should be removed"
    );
    assert!(
        !output.contains("xacro:macro"),
        "xacro:macro should be removed"
    );
    assert!(
        !output.contains("xacro:box_link"),
        "xacro:box_link should be removed"
    );

    // Verify output is valid XML
    let parsed = xmltree::Element::parse(output.as_bytes());
    assert!(
        parsed.is_ok(),
        "Output should be valid XML: {:?}",
        parsed.err()
    );
}

#[test]
fn test_nonstandard_prefix_with_known_uri() {
    // Test that we can handle xmlns:foo="http://www.ros.org/wiki/xacro" (non-standard prefix)
    // This goes beyond Python xacro's capabilities (which only accepts "xacro" prefix)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:foo="http://www.ros.org/wiki/xacro" name="test">
  <foo:property name="width" value="2"/>
  <foo:property name="height" value="3"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="${width} ${height} 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let output = processor.run_from_string(input).unwrap();

    // Should process successfully
    assert!(output.contains(r#"name="base""#), "Link should be present");
    assert!(output.contains("2 3 1"), "Properties should be substituted");

    // The xmlns:foo should be removed since we detected it's a xacro namespace
    assert!(
        !output.contains("xmlns:foo"),
        "Non-standard xacro prefix should be removed, but got:\n{}",
        output
    );

    // No foo:property elements should remain
    assert!(
        !output.contains("foo:property"),
        "foo:property should be removed"
    );

    // Verify output is valid XML
    let parsed = xmltree::Element::parse(output.as_bytes());
    assert!(
        parsed.is_ok(),
        "Output should be valid XML: {:?}",
        parsed.err()
    );
}

#[test]
fn test_plain_urdf_without_xacro_namespace() {
    // Test backward compatibility: plain URDF files without xacro namespace should work
    let input = r#"<?xml version="1.0"?>
<robot name="simple">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Plain URDF without xacro namespace should process successfully (backward compatible)"
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="base_link""#),
        "Link should be present"
    );
    assert!(
        output.contains(r#"<box size="1 1 1""#),
        "Geometry should be preserved"
    );
}

#[test]
fn test_xacro_element_without_namespace_declaration_fails() {
    // Test that xacro elements WITH namespace URI but WITHOUT root declaration fail with helpful error
    let input = r#"<?xml version="1.0"?>
<robot name="invalid">
  <property xmlns="http://www.ros.org/wiki/xacro" name="x" value="1"/>
  <link name="base_link"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_err(),
        "Should fail when xacro elements are used without namespace declaration"
    );

    let err = result.unwrap_err();
    assert!(
        matches!(
            err,
            XacroError::MissingNamespace(ref msg) if msg.contains("no xacro namespace declared")
        ),
        "Expected MissingNamespace error with 'no xacro namespace declared' message, got: {:?}",
        err
    );
}

#[test]
fn test_invalid_xacro_namespace_uri_with_typo() {
    // Test that typos in xacro namespace URI are rejected (not silently accepted)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacr" name="test">
  <xacro:property name="x" value="1"/>
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should fail - the URI has a typo ("xacr" instead of "xacro")
    assert!(
        result.is_err(),
        "Should fail when xacro prefix is bound to invalid URI"
    );

    let err = result.unwrap_err();
    assert!(
        matches!(
            err,
            XacroError::MissingNamespace(ref msg) if msg.contains("unknown URI") || msg.contains("typo")
        ),
        "Expected MissingNamespace error mentioning invalid URI or typo, got: {:?}",
        err
    );
}

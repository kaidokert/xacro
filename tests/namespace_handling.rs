mod common;
use crate::common::*;
use xacro::XacroError;

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

    let output = run_xacro(input);

    // Should not contain xmlns:xacro anywhere
    assert_xacro_not_contains!(output, "xmlns:xacro");

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

    let output = run_xacro(input);

    // xacro namespace should be removed
    assert_xacro_not_contains!(output, "xmlns:xacro");

    // Other namespaces should be preserved
    assert_xacro_contains!(output, "xmlns:gazebo");
    assert_xacro_contains!(output, "xmlns:ignition");

    // Elements using those namespaces should be present
    assert_xacro_contains!(output, "<gazebo");
    assert_xacro_contains!(output, "ignition:plugin");

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

    let output = run_xacro(input);
    // Should have processed successfully and xacro:arg should be removed from output
    assert_xacro_not_contains!(output, "xacro:arg");
    assert_xacro_contains!(output, "base_link");
}

#[test]
fn test_unimplemented_feature_detection_element() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:element xacro:name="link" name="base"/>
</robot>"#;

    let result = test_xacro(input);
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

    let result = test_xacro(input);
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

    let result = test_xacro(input);

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

    let output = run_xacro(input);

    // Should process successfully
    assert_xacro_contains!(output, "my_link");
    assert_xacro_contains!(output, "conditional_link");
    assert_xacro_contains!(output, "0.5");

    // xacro namespace should be removed
    assert_xacro_not_contains!(output, "xmlns:xacro");

    // No xacro:* elements should remain
    assert_xacro_not_contains!(output, "xacro:property");
    assert_xacro_not_contains!(output, "xacro:if");

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

    let output = run_xacro(input);

    // Should process successfully with macros
    assert_xacro_contains!(output, r#"name="macro_link""#);
    assert_xacro_contains!(output, r#"0.5 0.5 0.5"#);

    // xacro namespace should be removed
    assert_xacro_not_contains!(output, "xmlns:xacro");

    // Other namespaces should be preserved
    assert_xacro_contains!(output, "xmlns:gazebo");

    // No xacro:* elements should remain
    assert_xacro_not_contains!(output, "xacro:property");
    assert_xacro_not_contains!(output, "xacro:macro");
    assert_xacro_not_contains!(output, "xacro:box_link");

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

    let output = run_xacro(input);

    // Should process successfully
    assert_xacro_contains!(output, r#"name="base""#);
    assert_xacro_contains!(output, "2 3 1");

    // The xmlns:foo should be removed since we detected it's a xacro namespace
    assert!(
        !output.contains("xmlns:foo"),
        "Non-standard xacro prefix should be removed, but got:\n{}",
        output
    );

    // No foo:property elements should remain
    assert_xacro_not_contains!(output, "foo:property");

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

    let output = run_xacro(input);
    assert_xacro_contains!(output, r#"name="base_link""#);
    assert_xacro_contains!(output, r#"<box size="1 1 1""#);
}

#[test]
fn test_xacro_element_without_namespace_declaration_fails() {
    // Test that xacro elements WITH namespace URI but WITHOUT root declaration fail with helpful error
    let input = r#"<?xml version="1.0"?>
<robot name="invalid">
  <property xmlns="http://www.ros.org/wiki/xacro" name="x" value="1"/>
  <link name="base_link"/>
</robot>"#;

    let result = test_xacro(input);
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

    let result = test_xacro(input);

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

#[test]
fn test_include_different_namespace_prefix() {
    use std::fs;
    use tempfile::TempDir;

    let temp_dir = TempDir::new().expect("Should create temp directory");
    let temp_path = temp_dir.path();

    // Create included file with different namespace prefix (xmlns:x instead of xmlns:xacro)
    let included_content = r#"<?xml version="1.0"?>
<part xmlns:x="http://www.ros.org/wiki/xacro">
  <x:property name="included_prop" value="from_include"/>
  <link name="included_link">
    <visual name="${included_prop}"/>
  </link>
</part>"#;

    let included_path = temp_path.join("included.xacro");
    fs::write(&included_path, included_content).expect("Should write included file");

    // Root file with standard xmlns:xacro prefix
    let root_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="root_prop" value="from_root"/>
  <xacro:include filename="{}"/>
  <link name="root_link">
    <visual name="${{root_prop}}"/>
  </link>
</robot>"#,
        included_path.display()
    );

    let output = run_xacro_expect(
        &root_content,
        "Should handle included file with different namespace prefix",
    );
    // Verify property from included file was evaluated
    assert_xacro_contains!(output, "from_include");
    // Verify property from root file still works
    assert_xacro_contains!(output, "from_root");
}

#[test]
fn test_include_different_xacro_uri() {
    use std::fs;
    use tempfile::TempDir;

    let temp_dir = TempDir::new().expect("Should create temp directory");
    let temp_path = temp_dir.path();

    // Create included file with different but valid xacro URI
    let included_content = r#"<?xml version="1.0"?>
<part xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="uri_test" value="different_uri"/>
</part>"#;

    let included_path = temp_path.join("different_uri.xacro");
    fs::write(&included_path, included_content).expect("Should write included file");

    // Root file with standard URI
    let root_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="{}"/>
  <link name="test" value="${{uri_test}}"/>
</robot>"#,
        included_path.display()
    );

    let output = run_xacro_expect(
        &root_content,
        "Should handle included file with different valid xacro URI",
    );
    assert_xacro_contains!(output, "different_uri");
}

#[test]
fn test_nested_includes_namespace_isolation() {
    use std::fs;
    use tempfile::TempDir;

    let temp_dir = TempDir::new().expect("Should create temp directory");
    let temp_path = temp_dir.path();

    // Create first included file with prefix "x"
    let include_a_content = r#"<?xml version="1.0"?>
<part_a xmlns:x="http://www.ros.org/wiki/xacro">
  <x:property name="from_a" value="value_a"/>
  <link name="link_a" value="${from_a}"/>
</part_a>"#;
    let include_a_path = temp_path.join("include_a.xacro");
    fs::write(&include_a_path, include_a_content).expect("Should write include_a file");

    // Create second included file with prefix "y"
    let include_b_content = r#"<?xml version="1.0"?>
<part_b xmlns:y="http://www.ros.org/wiki/xacro">
  <y:property name="from_b" value="value_b"/>
  <link name="link_b" value="${from_b}"/>
</part_b>"#;
    let include_b_path = temp_path.join("include_b.xacro");
    fs::write(&include_b_path, include_b_content).expect("Should write include_b file");

    // Root file includes both
    let root_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="from_root" value="value_root"/>
  <xacro:include filename="{}"/>
  <xacro:include filename="{}"/>
  <link name="root" value="${{from_root}}"/>
</robot>"#,
        include_a_path.display(),
        include_b_path.display()
    );

    let output = run_xacro_expect(
        &root_content,
        "Should handle nested includes with different namespace prefixes",
    );
    // Verify all three properties were evaluated correctly
    assert_xacro_contains!(output, "value_a");
    assert_xacro_contains!(output, "value_b");
    assert_xacro_contains!(output, "value_root");
}

#[test]
fn test_custom_namespace_attribute_prefix_preserved() {
    // Test that custom namespace prefixes on attributes are preserved
    // This was a bug where tesseract:make_convex became just make_convex
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       xmlns:tesseract="https://github.com/tesseract-robotics/tesseract"
       name="test"
       tesseract:make_convex="true">
  <xacro:property name="size" value="0.5"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="${size} ${size} ${size}"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let output = run_xacro(input);

    // Should preserve the tesseract: prefix on the attribute
    assert!(
        output.contains("tesseract:make_convex=\"true\""),
        "Custom namespace prefix should be preserved in attributes. Output: {}",
        output
    );
    // Ensure the prefix is NOT dropped
    assert!(
        !output.contains(" make_convex=\"true\""),
        "Attribute must not lose its tesseract: prefix in output. Output: {}",
        output
    );

    // Should still have the namespace declaration
    assert!(
        output.contains("xmlns:tesseract=\"https://github.com/tesseract-robotics/tesseract\""),
        "Custom namespace declaration should be preserved"
    );

    // Should NOT have xacro namespace (it should be removed)
    assert_xacro_not_contains!(output, "xmlns:xacro");

    // Property substitution should still work
    assert_xacro_contains!(output, "0.5 0.5 0.5");
}

/// Test that properties defined inside macros work across namespace variants
///
/// This tests lazy macro evaluation: when a macro from an included file with
/// a different namespace variant (e.g., http://ros.org/wiki/xacro without www)
/// is called from the main file (e.g., http://www.ros.org/wiki/xacro with www),
/// the properties inside that macro body should still be recognized and processed.
///
/// Timeline:
/// 1. Include file with macro (namespace: http://ros.org/wiki/xacro)
/// 2. Macro body stored as raw XML (lazy evaluation)
/// 3. Include ends, namespace stack pops to main file (http://www.ros.org/wiki/xacro)
/// 4. Macro called from main file
/// 5. During expansion, properties inside macro have original namespace
/// 6. Directive dispatch must check is_known_xacro_uri(), not just exact match
///
#[test]
fn test_include_macro_with_properties_across_namespace_variants() {
    use std::io::Write;

    // Create included file with different namespace variant
    let mut included_file = tempfile::Builder::new()
        .suffix(".xacro")
        .tempfile()
        .expect("Failed to create temp file");

    // Included file: namespace http://ros.org/wiki/xacro (different from main)
    // Defines macro with properties INSIDE macro body
    write!(
        included_file,
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="parent">
    <!-- Properties defined INSIDE macro body (lazy evaluation) -->
    <xacro:property name="foo" value="2.0"/>
    <xacro:property name="bar" value="3.0"/>

    <link name="${{parent}}_child">
      <visual>
        <geometry>
          <box size="${{foo}} ${{bar}} 0.5"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>
</robot>"#
    )
    .expect("Failed to write to temp file");
    included_file.flush().expect("Failed to flush temp file");

    let included_path = included_file
        .path()
        .to_str()
        .expect("Path is not valid UTF-8");

    // Main file: namespace http://www.ros.org/wiki/xacro (WITH www, different variant)
    let main_xml = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{}"/>

  <!-- Call macro from included file -->
  <xacro:test_macro parent="base"/>
</robot>"#,
        included_path
    );

    // This MUST work! Properties inside macro from included file should be processed
    let output = run_xacro_expect(
        &main_xml,
        "Properties inside macros from included files with different namespace variants should work",
    );

    // Verify the macro was expanded and properties were resolved
    assert_xacro_contains!(output, r#"name="base_child""#);
    assert_xacro_contains!(output, r#"size="2.0 3.0 0.5""#);

    // No xacro elements should remain
    assert_xacro_not_contains!(output, "xacro:property");
    assert_xacro_not_contains!(output, "xacro:macro");
}

// Integration tests for namespace URI validation with --compat=namespace
//
// Python xacro is prefix-centric and doesn't validate namespace URIs.
// Rust xacro is namespace-aware and rejects "typo" URIs by default,
// but accepts them with --compat=namespace flag.
//
// Real-world corpus has 52 files (70% of errors) with namespace "typos" like:
//   xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
// instead of the correct:
//   xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#xacro"

use xacro::{CompatMode, XacroError, XacroProcessor};

// Local test helpers for XML parsing (reduce boilerplate)
// TODO: Move to tests/common/mod.rs when generalizing later

/// Get attribute value by local name
fn get_attr<'a>(
    elem: &'a xmltree::Element,
    name: &str,
) -> &'a str {
    elem.attributes
        .iter()
        .find(|(attr_name, _)| attr_name.local_name == name)
        .map(|(_, value)| value.as_str())
        .unwrap_or_else(|| panic!("Expected '{}' attribute", name))
}

/// Find child element by name and prefix
fn find_element_by_prefix<'a>(
    parent: &'a xmltree::Element,
    name: &str,
    prefix: &str,
) -> &'a xmltree::Element {
    parent
        .children
        .iter()
        .find_map(|node| {
            node.as_element()
                .filter(|elem| elem.name == name && elem.prefix.as_deref() == Some(prefix))
        })
        .unwrap_or_else(|| panic!("Expected <{}:{}> element", prefix, name))
}

#[test]
fn test_namespace_typo_error_by_default() {
    // Namespace URI "typo" should error in strict mode
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" name="test">
  <xacro:property name="value" value="42"/>
  <link name="base">
    <visual>
      <origin xyz="0 0 ${value}"/>
    </visual>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should error with MissingNamespace variant
    match result {
        Err(XacroError::MissingNamespace(msg)) => {
            assert!(
                msg.contains("unknown URI"),
                "Error should mention unknown URI, got: {}",
                msg
            );
            assert!(
                msg.contains("#interface"),
                "Error should show the typo URI (#interface), got: {}",
                msg
            );
        }
        Err(other) => panic!("Expected MissingNamespace error, got: {:?}", other),
        Ok(_) => panic!("Expected error in strict mode, but processing succeeded"),
    }
}

#[test]
fn test_namespace_typo_accepted_with_compat_namespace() {
    // Namespace URI "typo" should be accepted with --compat=namespace
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" name="test">
  <xacro:property name="value" value="42"/>
  <link name="base">
    <visual>
      <origin xyz="0 0 ${value}"/>
    </visual>
  </link>
</robot>"#;

    let compat_mode = "namespace".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);

    // Should succeed in namespace compat mode
    assert!(
        result.is_ok(),
        "Namespace URI typo should be accepted with --compat=namespace, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Verify expression was evaluated
    assert!(
        output.contains(r#"xyz="0 0 42""#),
        "Should evaluate expression correctly, got: {}",
        output
    );

    // Note: In lenient mode, non-standard xacro namespace URIs are preserved in output
    // (finalize_tree only removes known xacro URIs). This is acceptable since the goal
    // is Python xacro compatibility, which also preserves these namespaces.
}

#[test]
fn test_namespace_typo_accepted_with_compat_all() {
    // Namespace URI "typo" should be accepted with --compat=all
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" name="test">
  <xacro:property name="value" value="42"/>
  <link name="base">
    <visual>
      <origin xyz="0 0 ${value}"/>
    </visual>
  </link>
</robot>"#;

    let compat_mode = "all".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);

    // Should succeed with all compat modes enabled
    assert!(
        result.is_ok(),
        "Namespace URI typo should be accepted with --compat=all, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"xyz="0 0 42""#),
        "Should evaluate expression correctly, got: {}",
        output
    );
}

#[test]
fn test_namespace_typo_still_errors_with_compat_duplicate_params() {
    // Namespace URI "typo" should still error with --compat=duplicate_params
    // (only namespace compat mode should affect this)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" name="test">
  <xacro:property name="value" value="42"/>
  <link name="base">
    <visual>
      <origin xyz="0 0 ${value}"/>
    </visual>
  </link>
</robot>"#;

    let compat_mode = "duplicate_params".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);

    // Should still error (namespace validation not disabled)
    match result {
        Err(XacroError::MissingNamespace(msg)) => {
            assert!(
                msg.contains("unknown URI"),
                "Error should mention unknown URI, got: {}",
                msg
            );
        }
        Err(other) => panic!("Expected MissingNamespace error, got: {:?}", other),
        Ok(_) => panic!("Expected error with --compat=duplicate_params, but processing succeeded"),
    }
}

#[test]
fn test_real_world_playerstage_interface_typo() {
    // Real-world case from corpus: playerstage.sourceforge.net with #interface
    // This is the most common namespace "typo" (52 of 74 errors, 70%)
    let input = r#"<?xml version="1.0"?>
<robot name='taurob_tracker'
       xmlns:xacro='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface'>
  <xacro:property name="wheel_radius" value="0.125"/>
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    // Strict mode: should error
    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);
    match result {
        Err(XacroError::MissingNamespace(msg)) => {
            assert!(
                msg.contains("#interface"),
                "Error should show the typo URI (#interface), got: {}",
                msg
            );
        }
        Err(other) => panic!("Expected MissingNamespace error, got: {:?}", other),
        Ok(_) => panic!("Real-world namespace typo should error in strict mode"),
    }

    // Compat namespace mode: should succeed
    let compat_mode = "namespace".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Real-world namespace typo should work with --compat=namespace, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // Verify expression was evaluated
    assert!(
        output.contains(r#"radius="0.125""#),
        "Should evaluate wheel_radius expression, got: {}",
        output
    );
}

#[test]
fn test_ros_wiki_xacro_interface_typo() {
    // Another common typo: ros.org/wiki/xacro/#interface instead of just /wiki/xacro
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro/#interface" name="test">
  <xacro:property name="pi" value="3.14159"/>
  <link name="base">
    <inertial>
      <mass value="${pi}"/>
    </inertial>
  </link>
</robot>"#;

    // Strict mode: should error
    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);
    assert!(
        matches!(result, Err(XacroError::MissingNamespace(_))),
        "Should error with MissingNamespace on wiki/xacro/#interface typo"
    );

    // Compat namespace mode: should succeed
    let compat_mode = "namespace".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Should accept wiki/xacro/#interface with --compat=namespace, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"value="3.14159""#),
        "Should evaluate pi expression, got: {}",
        output
    );
}

#[test]
fn test_multiple_compat_modes() {
    // Test comma-separated compat modes: namespace,duplicate_params
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" name="test">
  <xacro:macro name="test_macro" params="x:=1 x:=2">
    <link name="test">
      <param value="${x}"/>
    </link>
  </xacro:macro>

  <xacro:test_macro/>
</robot>"#;

    // Both namespace typo AND duplicate params - should error in strict mode
    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);
    assert!(
        result.is_err(),
        "Should error with both issues in strict mode"
    );

    // Only namespace compat: should fail on duplicate params
    let compat_mode = "namespace".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);
    assert!(
        result.is_err(),
        "Should still error on duplicate params with only namespace compat"
    );
    assert!(result.unwrap_err().to_string().contains("Duplicate"));

    // Both modes: should succeed
    let compat_mode = "namespace,duplicate_params".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Should succeed with both compat modes, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // Last duplicate wins (x=2)
    assert!(
        output.contains(r#"value="2""#),
        "Should use last duplicate param value, got: {}",
        output
    );
}

#[test]
fn test_namespace_collision_strict_mode() {
    // Namespace collision scenario: two prefixes bound to same unknown URI (#interface)
    // In strict mode, this errors during namespace extraction (unknown URI validation)
    // before the collision handling in finalize_tree is reached
    let input = r#"<?xml version="1.0"?>
<robot xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       name="test">
  <xacro:property name="value" value="42"/>
  <link name="base">
    <visual>
      <origin xyz="0 0 ${value}"/>
    </visual>
  </link>
  <interface:position name="position_iface_0"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should error: namespace URI validation happens before finalize_tree
    assert!(
        matches!(result, Err(XacroError::MissingNamespace(_))),
        "Should error on namespace collision in strict mode, got: {:?}",
        result
    );
}

#[test]
fn test_namespace_collision_compat_mode() {
    // Namespace collision should be accepted in compat mode
    // Python xacro checks prefix string, not namespace URI
    let input = r#"<?xml version="1.0"?>
<robot xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       name="test">
  <xacro:property name="value" value="42"/>
  <link name="base">
    <visual>
      <origin xyz="0 0 ${value}"/>
    </visual>
  </link>
  <interface:position name="position_iface_0"/>
</robot>"#;

    let compat_mode: CompatMode = "namespace".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);

    // Should succeed and pass through <interface:position>
    assert!(
        result.is_ok(),
        "Namespace collision should be accepted in compat mode, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Parse output XML to verify structure
    let root = xmltree::Element::parse(output.as_bytes()).expect("Should parse output XML");

    // Verify expression was evaluated
    let link = root.get_child("link").expect("Should have <link> element");
    let visual = link
        .get_child("visual")
        .expect("Should have <visual> element");
    let origin = visual
        .get_child("origin")
        .expect("Should have <origin> element");
    assert_eq!(
        get_attr(origin, "xyz"),
        "0 0 42",
        "Should evaluate expression correctly"
    );

    // Verify <interface:position> was preserved
    let interface_pos = find_element_by_prefix(&root, "position", "interface");
    assert_eq!(
        get_attr(interface_pos, "name"),
        "position_iface_0",
        "Should preserve position_iface_0 attribute"
    );
}

#[test]
fn test_namespace_collision_multiple_elements() {
    // Real-world case: multiple interface elements in r2d2.xacro
    let input = r#"<?xml version="1.0"?>
<robot xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       name="test">
  <xacro:property name="value" value="0.2"/>
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${value}"/>
    </visual>
  </link>
  <gazebo>
    <interface:position name="position_iface_0"/>
    <interface:audio name="audio_iface"/>
  </gazebo>
</robot>"#;

    // Strict mode: should error (namespace validation happens early)
    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);
    assert!(
        matches!(result, Err(XacroError::MissingNamespace(_))),
        "Should error on namespace collision in strict mode"
    );

    // Compat namespace mode: should succeed
    let compat_mode: CompatMode = "namespace".parse().unwrap();
    let processor =
        XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat_mode);
    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Real-world namespace collision should work with --compat=namespace, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Parse output XML to verify structure
    let root = xmltree::Element::parse(output.as_bytes()).expect("Should parse output XML");

    // Verify xacro processing worked
    let link = root.get_child("link").expect("Should have <link> element");
    let visual = link
        .get_child("visual")
        .expect("Should have <visual> element");
    let origin = visual
        .get_child("origin")
        .expect("Should have <origin> element");
    assert_eq!(
        get_attr(origin, "xyz"),
        "0 0 0.2",
        "Should evaluate expression"
    );

    // Verify both interface elements were preserved
    let gazebo = root
        .get_child("gazebo")
        .expect("Should have <gazebo> element");

    let position = find_element_by_prefix(gazebo, "position", "interface");
    assert_eq!(
        get_attr(position, "name"),
        "position_iface_0",
        "Should preserve interface:position name attribute"
    );

    let audio = find_element_by_prefix(gazebo, "audio", "interface");
    assert_eq!(
        get_attr(audio, "name"),
        "audio_iface",
        "Should preserve interface:audio name attribute"
    );
}

#[test]
fn test_namespace_removed_from_output_standard_uri() {
    // Test that standard xacro namespace URI is removed from output
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="width" value="0.5"/>
  <link size="${width}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // xmlns:xacro should NOT appear in output
    assert!(
        !result.contains("xmlns:xacro"),
        "xmlns:xacro should be removed from output"
    );

    // Verify expansion worked correctly
    let root = xmltree::Element::parse(result.as_bytes()).unwrap();
    let link = root.get_child("link").unwrap();
    assert_eq!(get_attr(link, "size"), "0.5");
}

#[test]
fn test_namespace_removed_from_output_nonstandard_uri() {
    // Test that non-standard xacro namespace URIs (accepted in compat mode)
    // are also removed from output
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       name="test">
  <xacro:property name="width" value="0.5"/>
  <link size="${width}"/>
</robot>"#;

    let compat = "namespace".parse().unwrap();
    let processor =
        xacro::XacroProcessor::new_with_compat_mode(std::collections::HashMap::new(), compat);
    let result = processor.run_from_string(input).unwrap();

    // xmlns:xacro should NOT appear in output
    assert!(
        !result.contains("xmlns:xacro"),
        "xmlns:xacro should be removed from output"
    );

    // But processing should have worked correctly
    let root = xmltree::Element::parse(result.as_bytes()).unwrap();
    let link = root.get_child("link").unwrap();
    assert_eq!(get_attr(link, "size"), "0.5");
}

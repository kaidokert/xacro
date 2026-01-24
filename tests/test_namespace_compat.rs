// Integration tests for namespace URI validation with --compat=namespace
//
// Python xacro is prefix-centric and doesn't validate namespace URIs.
// Rust xacro is namespace-aware and rejects "typo" URIs by default,
// but accepts them with --compat=namespace flag.
//
// Real-world has files with namespace "typos" like:
//   xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
// instead of the correct:
//   xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#xacro"

mod common;
use crate::common::*;
use xacro_rs::{CompatMode, XacroError};

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

    // Should error with MissingNamespace variant
    let result = test_xacro(input);
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
    let output = run_xacro_with_compat(input, compat_mode);

    // Verify expression was evaluated
    assert_xacro_contains!(output, r#"xyz="0 0 42""#);

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
    let output = run_xacro_with_compat(input, compat_mode);

    assert_xacro_contains!(output, r#"xyz="0 0 42""#);
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
    let result = test_xacro_with_compat(input, compat_mode);

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
    // Real-world case: playerstage.sourceforge.net with #interface
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
    let result = test_xacro(input);
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
    let output = run_xacro_with_compat(input, compat_mode);

    // Verify expression was evaluated
    assert_xacro_contains!(output, r#"radius="0.125""#);
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
    assert_xacro_error_variant!(
        input,
        |e| matches!(e, &XacroError::MissingNamespace(_)),
        "Should error with MissingNamespace on wiki/xacro/#interface typo"
    );

    // Compat namespace mode: should succeed
    let compat_mode = "namespace".parse().unwrap();
    let output = run_xacro_with_compat(input, compat_mode);

    assert_xacro_contains!(output, r#"value="3.14159""#);
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
    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Should error with both issues in strict mode"
    );

    // Only namespace compat: should fail on duplicate params
    let compat_mode = "namespace".parse().unwrap();
    let result = test_xacro_with_compat(input, compat_mode);
    assert!(
        result.is_err(),
        "Should still error on duplicate params with only namespace compat"
    );
    assert!(result.unwrap_err().to_string().contains("Duplicate"));

    // Both modes: should succeed
    let compat_mode = "namespace,duplicate_params".parse().unwrap();
    let output = run_xacro_with_compat(input, compat_mode);

    // Last duplicate wins (x=2)
    assert_xacro_contains!(output, r#"value="2""#);
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

    // Should error: namespace URI validation happens before finalize_tree
    assert_xacro_error_variant!(
        input,
        |e| matches!(e, &XacroError::MissingNamespace(_)),
        "Should error on namespace collision in strict mode"
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
    let output = run_xacro_with_compat(input, compat_mode);

    // Parse output XML to verify structure
    let root = parse_xml(&output);

    // Verify expression was evaluated
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let origin = find_child(visual, "origin");
    assert_eq!(
        get_attr(origin, "xyz"),
        "0 0 42",
        "Should evaluate expression correctly"
    );

    // Verify <interface:position> was preserved
    let interface_pos = find_child_prefixed(&root, "interface", "position");
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
    assert_xacro_error_variant!(
        input,
        |e| matches!(e, &XacroError::MissingNamespace(_)),
        "Should error on namespace collision in strict mode"
    );

    // Compat namespace mode: should succeed
    let compat_mode: CompatMode = "namespace".parse().unwrap();
    let output = run_xacro_with_compat(input, compat_mode);

    // Parse output XML to verify structure
    let root = parse_xml(&output);

    // Verify xacro processing worked
    let link = find_child(&root, "link");
    let visual = find_child(link, "visual");
    let origin = find_child(visual, "origin");
    assert_eq!(
        get_attr(origin, "xyz"),
        "0 0 0.2",
        "Should evaluate expression"
    );

    // Verify both interface elements were preserved
    let gazebo = find_child(&root, "gazebo");

    let position = find_child_prefixed(gazebo, "interface", "position");
    assert_eq!(
        get_attr(position, "name"),
        "position_iface_0",
        "Should preserve interface:position name attribute"
    );

    let audio = find_child_prefixed(gazebo, "interface", "audio");
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

    let output = run_xacro(input);

    // Verify expansion worked correctly and xmlns:xacro is removed
    let root = parse_xml(&output);

    // xmlns:xacro should NOT appear in output (check parsed structure)
    assert!(
        root.namespaces
            .as_ref()
            .map_or(true, |ns| !ns.0.contains_key("xacro")),
        "xmlns:xacro should be removed from output"
    );

    let link = find_child(&root, "link");
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
    let output = run_xacro_with_compat(input, compat);

    // Verify processing worked correctly and xmlns:xacro is removed
    let root = parse_xml(&output);

    // xmlns:xacro should NOT appear in output (check parsed structure)
    assert!(
        root.namespaces
            .as_ref()
            .map_or(true, |ns| !ns.0.contains_key("xacro")),
        "xmlns:xacro should be removed from output"
    );

    let link = find_child(&root, "link");
    assert_eq!(get_attr(link, "size"), "0.5");
}

#[test]
fn test_namespace_removed_aliased_prefix_standard_uri() {
    // Test defense-in-depth: non-"xacro" prefixes bound to standard xacro URIs
    // should also be removed (e.g., xmlns:foo="http://www.ros.org/wiki/xacro")
    let input = r#"<?xml version="1.0"?>
<robot xmlns:foo="http://www.ros.org/wiki/xacro" name="test">
  <foo:property name="width" value="0.5"/>
  <link size="${width}"/>
</robot>"#;

    let output = run_xacro(input);

    // Verify expansion worked correctly and xmlns:foo is removed
    let root = parse_xml(&output);

    // xmlns:foo should NOT appear in output (defense-in-depth for aliased prefixes)
    assert!(
        root.namespaces
            .as_ref()
            .map_or(true, |ns| !ns.0.contains_key("foo")),
        "xmlns:foo (bound to standard xacro URI) should be removed from output"
    );

    let link = find_child(&root, "link");
    assert_eq!(get_attr(link, "size"), "0.5");
}

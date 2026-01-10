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

use xacro::{CompatMode, XacroProcessor};

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

    // Should error with unknown URI message
    assert!(
        result.is_err(),
        "Namespace URI typo should error in strict mode"
    );

    let error = result.unwrap_err().to_string();
    assert!(
        error.contains("unknown URI"),
        "Error should mention unknown URI, got: {}",
        error
    );
    assert!(
        error.contains("#interface"),
        "Error should show the typo URI (#interface), got: {}",
        error
    );
    assert!(
        error.contains("Known xacro URIs"),
        "Error should list known URIs, got: {}",
        error
    );
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
    assert!(
        result.is_err(),
        "Namespace URI typo should still error with --compat=duplicate_params"
    );

    let error = result.unwrap_err().to_string();
    assert!(
        error.contains("unknown URI"),
        "Error should mention unknown URI, got: {}",
        error
    );
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
    assert!(
        result.is_err(),
        "Real-world namespace typo should error in strict mode"
    );
    assert!(result.unwrap_err().to_string().contains("#interface"));

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
        result.is_err(),
        "Should error on wiki/xacro/#interface typo"
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

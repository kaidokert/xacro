mod common;

use common::*;

/// Test that dictionary key access doesn't trigger false positive circular dependency
///
/// Bug: When a property value contains dictionary key access like `data['key']`,
/// the circular dependency detector was incorrectly treating the string literal 'key'
/// as a property reference, leading to false "circular dependency" errors when
/// the key name matches the property name.
///
/// Example: `initial_positions = data['initial_positions']` should NOT be circular
/// because 'initial_positions' in brackets is a string literal (dict key), not a
/// property reference.
#[test]
fn test_dict_key_not_circular_dependency() {
    // This test verifies the fix for the circular dependency false positive
    // The actual evaluation will fail because load_yaml is not implemented,
    // but it should NOT fail with a circular dependency error.

    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- This would trigger false positive before the fix:
         Property 'initial_positions' references 'initial_positions' in the dict key -->
    <xacro:property name="data_dict" value="{'initial_positions': [0, 1, 2]}" />
    <xacro:property name="initial_positions" value="${data_dict['initial_positions']}" />

    <link name="test">
        <origin xyz="${initial_positions[0]} 0 0"/>
    </link>
</robot>"#;

    // The error should be about undefined variable or dict syntax, NOT circular dependency
    let result = test_xacro(xacro);

    if let Err(e) = result {
        let error_msg = e.to_string();
        // Make sure it's NOT a circular dependency error
        assert!(
            !error_msg.contains("Circular property dependency"),
            "Should not report circular dependency, but got: {}",
            error_msg
        );
        // It's OK to fail for other reasons (e.g., dict literal not supported by pyisheval)
    }
}

#[test]
fn test_dict_key_different_name_no_error() {
    // When the dict key doesn't match the property name, it should work fine
    // (assuming dict literals were supported)

    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="data" value="{'x': 5, 'y': 10}" />
    <xacro:property name="my_x" value="${data['x']}" />

    <link name="test">
        <origin xyz="${my_x} 0 0"/>
    </link>
</robot>"#;

    let result = test_xacro(xacro);

    // Should not be circular dependency
    if let Err(e) = result {
        let error_msg = e.to_string();
        assert!(
            !error_msg.contains("Circular property dependency"),
            "Should not report circular dependency, but got: {}",
            error_msg
        );
    }
}

#[test]
fn test_actual_circular_dependency_still_detected() {
    // Verify that actual circular dependencies are still detected

    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- This IS a real circular dependency -->
    <xacro:property name="x" value="${y}" />
    <xacro:property name="y" value="${x}" />

    <link name="test">
        <origin xyz="${x} 0 0"/>
    </link>
</robot>"#;

    let result = test_xacro(xacro);

    // Should fail with circular dependency error
    assert!(result.is_err(), "Should detect circular dependency");
    let error_msg = result.unwrap_err().to_string();
    assert!(
        error_msg.contains("Circular property dependency"),
        "Should report circular dependency, but got: {}",
        error_msg
    );
}

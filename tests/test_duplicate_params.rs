// Integration tests for duplicate parameter handling
//
// Python xacro silently accepts duplicate parameters (buggy behavior).
// Rust xacro errors by default but accepts with --compat flag.

mod common;
use crate::common::*;

#[test]
fn test_duplicate_params_error_by_default() {
    // Duplicate parameter declarations should error in strict mode
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="test_macro" params="x:=1 y:=2 x:=3">
    <link name="test">
      <param_x value="${x}"/>
      <param_y value="${y}"/>
    </link>
  </xacro:macro>

  <xacro:test_macro/>
</robot>"#;

    let result = test_xacro(input);

    // Should error with duplicate parameter message
    assert!(
        result.is_err(),
        "Duplicate parameters should error in strict mode"
    );

    let error = result.unwrap_err().to_string();
    assert!(
        error.contains("Duplicate parameter declaration"),
        "Error should mention duplicate parameter, got: {}",
        error
    );
    assert!(
        error.contains("x"),
        "Error should mention parameter name 'x', got: {}",
        error
    );
    assert!(
        error.contains("--compat"),
        "Error should suggest --compat flag, got: {}",
        error
    );
}

#[test]
fn test_duplicate_params_accepted_in_compat_mode() {
    // Duplicate parameters should be accepted in compat mode (last wins)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="test_macro" params="x:=1 y:=2 x:=3">
    <link name="test">
      <param_x value="${x}"/>
      <param_y value="${y}"/>
    </link>
  </xacro:macro>

  <xacro:test_macro/>
</robot>"#;

    let compat = "all".parse().expect("Should parse compat mode");
    let output = run_xacro_with_compat(input, compat);

    // Should succeed in compat mode
    // Last declaration wins (x:=3)
    assert_xacro_contains!(
        output,
        r#"<param_x value="3""#,
        "Should use last declaration's default (x=3)"
    );
    assert_xacro_contains!(output, r#"<param_y value="2""#, "Should use y=2");
}

#[test]
fn test_duplicate_params_multiple_duplicates() {
    // Test multiple duplicate parameters
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="test_macro" params="a:=1 b:=2 a:=3 c:=4 b:=5">
    <link name="test">
      <param_a value="${a}"/>
      <param_b value="${b}"/>
      <param_c value="${c}"/>
    </link>
  </xacro:macro>

  <xacro:test_macro/>
</robot>"#;

    // Strict mode: should error
    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Multiple duplicate parameters should error in strict mode"
    );

    // Compat mode: should succeed with last values
    let compat = "all".parse().expect("Should parse compat mode");
    let output = run_xacro_with_compat(input, compat);

    assert_xacro_contains!(
        output,
        r#"<param_a value="3""#,
        "Should use last 'a' declaration (a=3)"
    );
    assert_xacro_contains!(
        output,
        r#"<param_b value="5""#,
        "Should use last 'b' declaration (b=5)"
    );
    assert_xacro_contains!(
        output,
        r#"<param_c value="4""#,
        "Should use 'c' declaration (c=4)"
    );
}

#[test]
fn test_duplicate_params_with_passed_values() {
    // Test that passed values are still respected (better than Python's behavior)
    // Python xacro has a bug where passed values are ignored when duplicates exist.
    // Our implementation accepts duplicates (last default wins) but still respects
    // passed values, which is more correct behavior.
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="test_macro" params="x:=1 x:=2">
    <link name="test">
      <param_x value="${x}"/>
    </link>
  </xacro:macro>

  <!-- Try to pass x=100 -->
  <xacro:test_macro x="100"/>
</robot>"#;

    // Compat mode: Accepts duplicates but still respects passed values
    let compat = "all".parse().expect("Should parse compat mode");
    let output = run_xacro_with_compat(input, compat);

    // Our implementation: Respects passed value (100), not last default (2)
    // This is BETTER than Python xacro's buggy behavior
    assert_xacro_contains!(
        output,
        r#"<param_x value="100""#,
        "Should use passed value (100) - better than Python's buggy behavior"
    );
}

#[test]
fn test_real_world_hk_camera_case() {
    // Has color_xyz_offset and color_rpy_offset duplicated
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="hk_ir_camera" params="
    color_xyz_offset:='0 0 0'
    color_rpy_offset:='0 0 0'
    color_xyz_offset:='0 0.015 0'
    color_rpy_offset:='0 0 0'">

    <link name="camera_color_frame">
      <origin xyz="${color_xyz_offset}" rpy="${color_rpy_offset}"/>
    </link>
  </xacro:macro>

  <xacro:hk_ir_camera/>
</robot>"#;

    // Strict mode: should error
    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Real-world duplicate params should error in strict mode"
    );
    assert!(result.unwrap_err().to_string().contains("color_xyz_offset"));

    // Compat mode: should succeed (last declarations win)
    let compat = "all".parse().expect("Should parse compat mode");
    let output = run_xacro_with_compat(input, compat);

    // Last declarations: color_xyz_offset:='0 0.015 0', color_rpy_offset:='0 0 0'
    assert_xacro_contains!(
        output,
        r#"xyz="0 0.015 0""#,
        "Should use last xyz declaration"
    );
    assert_xacro_contains!(output, r#"rpy="0 0 0""#, "Should use last rpy declaration");
}

// ============================================================================
// Block Parameter Duplicates
// ============================================================================

#[test]
fn test_duplicate_block_params_error_by_default() {
    // Duplicate *block parameter declarations should error in strict mode
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="block_dup" params="*body *body">
    <xacro:insert_block name="body"/>
  </xacro:macro>

  <xacro:block_dup>
    <link name="base"/>
  </xacro:block_dup>
</robot>"#;

    let result = test_xacro(input);

    assert!(
        result.is_err(),
        "Expected duplicate *block parameter error in strict mode"
    );

    let err = result.unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.to_lowercase().contains("duplicate"),
        "Error should mention duplicate: {}",
        msg
    );
}

#[test]
fn test_duplicate_block_params_compat_last_wins() {
    // Duplicate *block parameter declarations should be accepted in compat mode.
    // Even with duplicate declarations, there's still only ONE parameter, so only
    // ONE child element should be consumed.
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="block_dup" params="*body *body">
    <xacro:insert_block name="body"/>
  </xacro:macro>

  <xacro:block_dup>
    <link name="base"/>
  </xacro:block_dup>
</robot>"#;

    let compat = "all".parse().expect("Should parse compat mode");
    let output = run_xacro_with_compat(input, compat);

    // Should successfully insert the single block parameter
    assert_xacro_contains!(
        output,
        r#"name="base""#,
        "Compat mode should accept duplicate block param declarations"
    );
}

#[test]
fn test_block_and_nonblock_param_conflict_error_by_default() {
    // Conflicting block/non-block declarations of the same parameter
    // should error in strict mode
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="block_nonblock_conflict" params="body:=default *body">
    <xacro:insert_block name="body"/>
  </xacro:macro>

  <xacro:block_nonblock_conflict>
    <link name="from_block"/>
  </xacro:block_nonblock_conflict>
</robot>"#;

    let result = test_xacro(input);

    assert!(
        result.is_err(),
        "Expected block/non-block param conflict error in strict mode"
    );

    let err = result.unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.to_lowercase().contains("duplicate"),
        "Error should mention duplicate: {}",
        msg
    );
}

#[test]
fn test_block_and_nonblock_param_conflict_compat_last_wins() {
    // Conflicting block/non-block declarations of the same parameter
    // should be accepted in compat mode with "last declaration wins" semantics
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="block_nonblock_conflict" params="body:=default *body">
    <xacro:insert_block name="body"/>
  </xacro:macro>

  <xacro:block_nonblock_conflict>
    <link name="from_block"/>
  </xacro:block_nonblock_conflict>
</robot>"#;

    let compat = "all".parse().expect("Should parse compat mode");
    let output = run_xacro_with_compat(input, compat);

    // "last declaration wins": block declaration should be used
    assert_xacro_contains!(
        output,
        r#"name="from_block""#,
        "Compat mode should use last (block) declaration"
    );
}

#[test]
fn test_block_and_nonblock_param_conflict_compat_reverse_order() {
    // Reverse order: block declaration first, then non-block
    // "last declaration wins" should mean it's treated as non-block with default value
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="reverse_conflict" params="*body body:=default_value">
    <link name="${body}"/>
  </xacro:macro>

  <xacro:reverse_conflict/>
</robot>"#;

    let compat = "all".parse().expect("Should parse compat mode");
    let output = run_xacro_with_compat(input, compat);

    // "last declaration wins": non-block declaration should be used (with default)
    // Should NOT try to consume a child element
    assert_xacro_contains!(
        output,
        r#"name="default_value""#,
        "Compat mode should use last (non-block) declaration with default"
    );
}

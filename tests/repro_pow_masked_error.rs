mod common;
use common::*;

/// Reproduces the "Undefined variable: pow" red herring error
///
/// When a variable inside pow() is undefined, we should report the missing variable,
/// not that pow itself is undefined.
#[test]
fn test_pow_with_undefined_variable_reports_correct_error() {
    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="test">
    <inertial>
      <mass value="${pow(undefined_var, 2)}"/>
    </inertial>
  </link>
</robot>"#;

    let result = test_xacro(xacro);

    // Should report "undefined_var" is missing, NOT "pow" is missing
    assert!(
        result.is_err(),
        "Should fail when undefined variable is used"
    );
    let err_msg = result.unwrap_err().to_string();

    // The error should mention the undefined variable
    assert!(
        err_msg.contains("undefined_var"),
        "Expected error about 'undefined_var', but got: {}",
        err_msg
    );

    // The error should NOT blame pow
    assert!(
        !err_msg.contains("Undefined variable: pow"),
        "Error incorrectly blames 'pow' instead of the undefined argument: {}",
        err_msg
    );
}

#[test]
fn test_pow_with_valid_arguments_works() {
    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="base" value="2"/>
  <xacro:property name="result" value="${pow(base, 3)}"/>
  <link name="test">
    <inertial>
      <mass value="${result}"/>
    </inertial>
  </link>
</robot>"#;

    let xml = run_xacro(xacro);
    assert_xacro_contains!(xml, r#"<mass value="8"/>"#);
}

#[test]
fn test_cos_with_undefined_variable_reports_correct_error() {
    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="test">
    <inertial>
      <mass value="${cos(missing_angle)}"/>
    </inertial>
  </link>
</robot>"#;

    let result = test_xacro(xacro);

    assert!(
        result.is_err(),
        "Should fail when undefined variable is used"
    );
    let err_msg = result.unwrap_err().to_string();

    assert!(
        err_msg.contains("missing_angle"),
        "Expected error about 'missing_angle', but got: {}",
        err_msg
    );

    assert!(
        !err_msg.contains("Undefined variable: cos"),
        "Error incorrectly blames 'cos': {}",
        err_msg
    );
}

#[test]
fn test_log_with_undefined_first_arg_reports_correct_error() {
    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="test">
    <inertial>
      <mass value="${log(undefined_var, 10)}"/>
    </inertial>
  </link>
</robot>"#;

    let result = test_xacro(xacro);

    // Should report "undefined_var" is missing, NOT "log" is missing
    assert!(
        result.is_err(),
        "Should fail when undefined variable is used"
    );
    let err_msg = result.unwrap_err().to_string();

    assert!(
        err_msg.contains("undefined_var"),
        "Expected error about 'undefined_var', but got: {}",
        err_msg
    );

    assert!(
        !err_msg.contains("Undefined variable: log"),
        "Error incorrectly blames 'log': {}",
        err_msg
    );
}

#[test]
fn test_log_with_undefined_second_arg_reports_correct_error() {
    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="test">
    <inertial>
      <mass value="${log(10, undefined_base)}"/>
    </inertial>
  </link>
</robot>"#;

    let result = test_xacro(xacro);

    // Should report "undefined_base" is missing, NOT "log" is missing
    assert!(
        result.is_err(),
        "Should fail when undefined variable is used"
    );
    let err_msg = result.unwrap_err().to_string();

    assert!(
        err_msg.contains("undefined_base"),
        "Expected error about 'undefined_base', but got: {}",
        err_msg
    );

    assert!(
        !err_msg.contains("Undefined variable: log"),
        "Error incorrectly blames 'log': {}",
        err_msg
    );
}

#[test]
fn test_atan2_with_undefined_first_arg_reports_correct_error() {
    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="test">
    <inertial>
      <mass value="${atan2(undefined_y, 1)}"/>
    </inertial>
  </link>
</robot>"#;

    let result = test_xacro(xacro);

    // Should report "undefined_y" is missing, NOT "atan2" is missing
    assert!(
        result.is_err(),
        "Should fail when undefined variable is used"
    );
    let err_msg = result.unwrap_err().to_string();

    assert!(
        err_msg.contains("undefined_y"),
        "Expected error about 'undefined_y', but got: {}",
        err_msg
    );

    assert!(
        !err_msg.contains("Undefined variable: atan2"),
        "Error incorrectly blames 'atan2': {}",
        err_msg
    );
}

#[test]
fn test_atan2_with_undefined_second_arg_reports_correct_error() {
    let xacro = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="test">
    <inertial>
      <mass value="${atan2(1, undefined_x)}"/>
    </inertial>
  </link>
</robot>"#;

    let result = test_xacro(xacro);

    // Should report "undefined_x" is missing, NOT "atan2" is missing
    assert!(
        result.is_err(),
        "Should fail when undefined variable is used"
    );
    let err_msg = result.unwrap_err().to_string();

    assert!(
        err_msg.contains("undefined_x"),
        "Expected error about 'undefined_x', but got: {}",
        err_msg
    );

    assert!(
        !err_msg.contains("Undefined variable: atan2"),
        "Error incorrectly blames 'atan2': {}",
        err_msg
    );
}

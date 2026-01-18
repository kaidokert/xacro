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

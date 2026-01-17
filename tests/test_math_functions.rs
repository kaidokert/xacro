mod common;
use crate::common::*;

/// Helper function to run a math function test and verify the angle result
fn run_angle_test(
    input: &str,
    expected_value: f64,
    tolerance: f64,
    expect_msg: &str,
) {
    let output = run_xacro_expect(input, expect_msg);

    // Extract angle value and compare numerically
    let re = regex::Regex::new(r#"angle="([^"]+)""#).expect("Valid regex");
    let caps = re
        .captures(&output)
        .expect("angle attribute not found in output");
    let angle_val: f64 = caps[1].parse().expect("angle value is not a valid float");
    assert!(
        (angle_val - expected_value).abs() < tolerance,
        "Expected angle to be close to {}, got {}",
        expected_value,
        angle_val
    );
}

#[test]
fn test_radians_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${radians(90)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // radians(90) = 90 * pi / 180 = pi/2
    run_angle_test(
        input,
        std::f64::consts::FRAC_PI_2,
        1e-6,
        "Should process radians() function",
    );
}

#[test]
fn test_degrees_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${degrees(1.5707963267948966)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // degrees(pi/2) = 90
    run_angle_test(input, 90.0, 1e-9, "Should process degrees() function");
}

#[test]
fn test_radians_with_negative() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${radians(-111)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // radians(-111) = -111 * pi / 180 ≈ -1.9373154
    run_angle_test(
        input,
        -1.9373154,
        1e-6,
        "Should process radians() with negative value",
    );
}

#[test]
fn test_degrees_with_negative() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${degrees(-1.5707963267948966)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // degrees(-pi/2) = -90
    run_angle_test(
        input,
        -90.0,
        1e-9,
        "Should process degrees() with negative value",
    );
}

#[test]
fn test_cos_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${cos(0)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // cos(0) = 1.0
    run_angle_test(input, 1.0, 1e-9, "Should process cos() function");
}

#[test]
fn test_nested_math_functions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${cos(radians(0))}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // cos(radians(0)) = cos(0) = 1.0
    run_angle_test(
        input,
        1.0,
        1e-9,
        "Should process nested math functions cos(radians(0))",
    );
}

#[test]
fn test_sqrt_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${sqrt(4)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // sqrt(4) = 2.0
    run_angle_test(input, 2.0, 1e-9, "Should process sqrt() function");
}

#[test]
fn test_abs_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${abs(-3.7)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // abs(-3.7) = 3.7
    run_angle_test(input, 3.7, 1e-9, "Should process abs() function");
}

#[test]
fn test_floor_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${floor(3.7)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // floor(3.7) = 3.0
    run_angle_test(input, 3.0, 1e-9, "Should process floor() function");
}

#[test]
fn test_ceil_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${ceil(3.2)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // ceil(3.2) = 4.0
    run_angle_test(input, 4.0, 1e-9, "Should process ceil() function");
}

#[test]
fn test_acos_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${acos(1)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // acos(1) = 0.0
    run_angle_test(input, 0.0, 1e-9, "Should process acos() function");
}

#[test]
fn test_asin_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${asin(0)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // asin(0) = 0.0
    run_angle_test(input, 0.0, 1e-9, "Should process asin() function");
}

#[test]
fn test_atan_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${atan(0)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // atan(0) = 0.0
    run_angle_test(input, 0.0, 1e-9, "Should process atan() function");
}

#[test]
fn test_multiple_math_functions_in_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${sqrt(4) + cos(0) + abs(-1)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // sqrt(4) + cos(0) + abs(-1) = 2.0 + 1.0 + 1.0 = 4.0
    run_angle_test(
        input,
        4.0,
        1e-9,
        "Should process multiple math functions in expression",
    );
}

#[test]
fn test_deeply_nested_math_functions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${floor(abs(sqrt(cos(0))))}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // cos(0) = 1.0, sqrt(1.0) = 1.0, abs(1.0) = 1.0, floor(1.0) = 1.0
    run_angle_test(
        input,
        1.0,
        1e-9,
        "Should process deeply nested math functions",
    );
}

#[test]
fn test_math_functions_with_whitespace() {
    // Test with spaces inside parentheses
    let input_spaces = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${cos( 0 )}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;
    run_angle_test(
        input_spaces,
        1.0,
        1e-9,
        "Should handle spaces inside parentheses",
    );

    // Test with spaces before parentheses
    let input_before = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${cos  (0)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;
    run_angle_test(
        input_before,
        1.0,
        1e-9,
        "Should handle spaces before parentheses",
    );
}

#[test]
fn test_multiple_adjacent_math_functions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${cos(0) + sin(0)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // cos(0) + sin(0) = 1.0 + 0.0 = 1.0
    run_angle_test(
        input,
        1.0,
        1e-9,
        "Should process multiple adjacent math functions",
    );
}

#[test]
fn test_acos_domain_validation() {
    // Test that acos(2) fails (out of domain [-1, 1])
    // This matches Python xacro behavior which raises ValueError
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${acos(2)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);

    // Should fail - domain validation prevents replacement, then pyisheval fails
    // This matches Python xacro's ValueError for out-of-domain inputs
    assert!(result.is_err(), "Should error for out-of-domain acos(2)");
}

#[test]
fn test_asin_domain_validation() {
    // Test that asin(-1.5) fails (out of domain [-1, 1])
    // This matches Python xacro behavior which raises ValueError
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${asin(-1.5)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);

    // Should fail - domain validation prevents replacement, then pyisheval fails
    // This matches Python xacro's ValueError for out-of-domain inputs
    assert!(result.is_err(), "Should error for out-of-domain asin(-1.5)");
}

#[test]
fn test_acos_nan_domain() {
    // Test that acos(NaN) fails
    // NaN fails the domain check (!(-1.0..=1.0).contains(&NaN) == true)
    // This leaves the function call unresolved, causing pyisheval to fail
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${acos(float('nan'))}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);

    // Should fail - NaN fails domain check, function call remains unresolved
    assert!(result.is_err(), "Should error for acos(NaN)");
}

#[test]
fn test_asin_nan_domain() {
    // Test that asin(NaN) fails
    // NaN fails the domain check (!(-1.0..=1.0).contains(&NaN) == true)
    // This leaves the function call unresolved, causing pyisheval to fail
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${asin(float('nan'))}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);

    // Should fail - NaN fails domain check, function call remains unresolved
    assert!(result.is_err(), "Should error for asin(NaN)");
}

#[test]
fn test_atan2_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${atan2(0, 1)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // atan2(0, 1) = 0.0
    run_angle_test(input, 0.0, 1e-9, "Should process atan2() function");
}

#[test]
fn test_atan2_with_negative_values() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${atan2(1, -1)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // atan2(1, -1) = 3*pi/4 ≈ 2.356194490192345
    run_angle_test(
        input,
        2.356194490192345,
        1e-9,
        "Should process atan2() with negative x value",
    );
}

#[test]
fn test_atan2_with_expressions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="y" value="1"/>
  <xacro:property name="x" value="1"/>
  <xacro:property name="angle" value="${atan2(y, x)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // atan2(1, 1) = pi/4 ≈ 0.7853981633974483
    run_angle_test(
        input,
        std::f64::consts::FRAC_PI_4,
        1e-9,
        "Should process atan2() with property variables",
    );
}

#[test]
fn test_atan2_with_nested_expressions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${atan2(sqrt(3), 1)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    // atan2(sqrt(3), 1) = pi/3 ≈ 1.0471975511965979
    run_angle_test(
        input,
        std::f64::consts::FRAC_PI_3,
        1e-9,
        "Should process atan2() with nested math functions",
    );
}

#[test]
fn test_pow_function() {
    use std::collections::HashMap;
    use xacro::eval::interpreter::eval_text;

    let props = HashMap::new();
    let result = eval_text("${pow(2, 3)}", &props).expect("pow should work");
    assert_eq!(result, "8");

    let result = eval_text("${pow(10, 0.5)}", &props).expect("pow with fractional exp");
    let value: f64 = result.parse().expect("parse float");
    assert!((value - 10.0_f64.sqrt()).abs() < 1e-10, "sqrt(10) mismatch");
}

#[test]
fn test_log_function() {
    use std::collections::HashMap;
    use xacro::eval::interpreter::eval_text;

    let props = HashMap::new();
    let result = eval_text("${log(1)}", &props).expect("log should work");
    assert_eq!(result, "0", "ln(1) = 0");

    // ln(e) should be 1, using the built-in 'e' constant
    let result = eval_text("${log(e)}", &props).expect("log(e)");
    let value: f64 = result.parse().expect("parse float");
    assert!((value - 1.0).abs() < 1e-10, "ln(e) = 1");

    // Test log with base (log(100, 10) = 2)
    let result = eval_text("${log(100, 10)}", &props).expect("log(100, 10)");
    let value: f64 = result.parse().expect("parse float");
    assert!((value - 2.0).abs() < 1e-10, "log_10(100) = 2");

    // Test log with base 2 (log(8, 2) = 3)
    let result = eval_text("${log(8, 2)}", &props).expect("log(8, 2)");
    let value: f64 = result.parse().expect("parse float");
    assert!((value - 3.0).abs() < 1e-10, "log_2(8) = 3");
}

#[test]
fn test_math_prefix() {
    use std::collections::HashMap;
    use xacro::eval::interpreter::eval_text;

    let props = HashMap::new();

    // Test math.pow
    let result = eval_text("${math.pow(2, 3)}", &props).expect("math.pow");
    assert_eq!(result, "8");

    // Test math.log
    let result = eval_text("${math.log(1)}", &props).expect("math.log");
    assert_eq!(result, "0");

    // Test math.atan2
    let result = eval_text("${math.atan2(1, 0)}", &props).expect("math.atan2");
    let value: f64 = result.parse().expect("parse float");
    assert!(
        (value - std::f64::consts::FRAC_PI_2).abs() < 1e-10,
        "atan2(1,0) = π/2"
    );

    // Test math.sqrt
    let result = eval_text("${math.sqrt(4)}", &props).expect("math.sqrt");
    assert_eq!(result, "2");
}

#[test]
fn test_math_pi_constant() {
    use std::collections::HashMap;
    use xacro::eval::interpreter::eval_text;

    let props = HashMap::new();

    // Test math.pi
    let result = eval_text("${math.pi}", &props).expect("math.pi");
    let value: f64 = result.parse().expect("parse float");
    assert!((value - std::f64::consts::PI).abs() < 1e-10, "math.pi = π");

    // Test math.pi in expression (from corpus error)
    let result = eval_text("${-math.pi / 2}", &props).expect("-math.pi / 2");
    let value: f64 = result.parse().expect("parse float");
    assert!((value + std::f64::consts::FRAC_PI_2).abs() < 1e-10, "-π/2");
}

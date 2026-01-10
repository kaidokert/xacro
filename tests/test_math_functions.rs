use xacro::processor::XacroProcessor;

/// Helper function to run a math function test and verify the angle result
fn run_angle_test(
    input: &str,
    expected_value: f64,
    tolerance: f64,
    expect_msg: &str,
) {
    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).expect(expect_msg);

    // Extract angle value and compare numerically
    let re = regex::Regex::new(r#"angle="([^"]+)""#).expect("Valid regex");
    let caps = re
        .captures(&result)
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
    use xacro::processor::XacroProcessor;

    // Test that acos(2) fails (out of domain [-1, 1])
    // This matches Python xacro behavior which raises ValueError
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${acos(2)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should fail - domain validation prevents replacement, then pyisheval fails
    // This matches Python xacro's ValueError for out-of-domain inputs
    assert!(result.is_err(), "Should error for out-of-domain acos(2)");
}

#[test]
fn test_asin_domain_validation() {
    use xacro::processor::XacroProcessor;

    // Test that asin(-1.5) fails (out of domain [-1, 1])
    // This matches Python xacro behavior which raises ValueError
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${asin(-1.5)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should fail - domain validation prevents replacement, then pyisheval fails
    // This matches Python xacro's ValueError for out-of-domain inputs
    assert!(result.is_err(), "Should error for out-of-domain asin(-1.5)");
}

#[test]
fn test_acos_nan_domain() {
    use xacro::processor::XacroProcessor;

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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should fail - NaN fails domain check, function call remains unresolved
    assert!(result.is_err(), "Should error for acos(NaN)");
}

#[test]
fn test_asin_nan_domain() {
    use xacro::processor::XacroProcessor;

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

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should fail - NaN fails domain check, function call remains unresolved
    assert!(result.is_err(), "Should error for asin(NaN)");
}

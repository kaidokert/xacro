use xacro::processor::XacroProcessor;

#[test]
fn test_radians_function() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="angle" value="${radians(90)}"/>
  <link name="test">
    <joint angle="${angle}"/>
  </link>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Should process radians() function");

    // radians(90) = 90 * pi / 180 = pi/2
    // Extract angle value and compare numerically
    let re = regex::Regex::new(r#"angle="([^"]+)""#).expect("Valid regex");
    let caps = re
        .captures(&result)
        .expect("angle attribute not found in output");
    let angle_val: f64 = caps[1].parse().expect("angle value is not a valid float");
    assert!(
        (angle_val - std::f64::consts::FRAC_PI_2).abs() < 1e-6,
        "Expected angle to be close to pi/2 ({}), got {}",
        std::f64::consts::FRAC_PI_2,
        angle_val
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Should process degrees() function");

    // degrees(pi/2) = 90
    // Extract angle value and compare numerically
    let re = regex::Regex::new(r#"angle="([^"]+)""#).expect("Valid regex");
    let caps = re
        .captures(&result)
        .expect("angle attribute not found in output");
    let angle_val: f64 = caps[1].parse().expect("angle value is not a valid float");
    assert!(
        (angle_val - 90.0).abs() < 1e-9,
        "Expected angle to be close to 90.0, got {}",
        angle_val
    );
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

    let processor = XacroProcessor::new();
    let result = processor
        .run_from_string(input)
        .expect("Should process radians() with negative value");

    // radians(-111) = -111 * pi / 180 ≈ -1.9373154
    // Extract angle value and compare numerically
    let re = regex::Regex::new(r#"angle="([^"]+)""#).expect("Valid regex");
    let caps = re
        .captures(&result)
        .expect("angle attribute not found in output");
    let angle_val: f64 = caps[1].parse().expect("angle value is not a valid float");
    assert!(
        (angle_val - (-1.9373154)).abs() < 1e-6,
        "Expected angle to be close to -1.9373154, got {}",
        angle_val
    );
}

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

    // radians(90) = 90 * pi / 180 = pi/2 ≈ 1.5707963
    assert!(result.contains("1.5707963") || result.contains("1.57079"));
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
    assert!(result.contains("90"));
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

    // radians(-111) ≈ -1.9373
    assert!(result.contains("-1.937") || result.contains("-1.94"));
}

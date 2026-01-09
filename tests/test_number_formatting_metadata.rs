/// Integration test for number formatting with property metadata tracking
///
/// This test validates that we correctly format numbers to match Python xacro behavior:
/// - Properties with .0 in value string → always output with .0
/// - Properties without .0 → output without .0
/// - Division operations → always output with .0
/// - Float property used in expressions → propagate float-ness
use xacro::XacroProcessor;

#[test]
fn test_int_property_alone() {
    // Case: Property defined as "100" (no decimal)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="effort" value="100"/>
  <test value="${effort}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs: value="100" (no .0)
    assert!(
        result.contains(r#"value="100""#),
        "Int property alone should output without .0. Got: {}",
        result
    );
}

#[test]
fn test_float_property_alone() {
    // Case: Property defined as "100.0" (with decimal)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="effort" value="100.0"/>
  <test value="${effort}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs: value="100.0" (keeps .0)
    assert!(
        result.contains(r#"value="100.0""#),
        "Float property alone should output with .0. Got: {}",
        result
    );
}

#[test]
fn test_int_prop_times_int() {
    // Case: int * int → int
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="15"/>
  <test value="${x * 2}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs: value="30" (no .0)
    assert!(
        result.contains(r#"value="30""#),
        "Int * int should output without .0. Got: {}",
        result
    );
}

#[test]
fn test_float_prop_times_int() {
    // Case: float * int → float
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="1.5"/>
  <test value="${width * 2}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs: value="3.0" (keeps .0)
    assert!(
        result.contains(r#"value="3.0""#),
        "Float * int should output with .0. Got: {}",
        result
    );
}

#[test]
fn test_division_operation() {
    // Case: division always produces float
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <test value="${255/255}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs: value="1.0" (has .0 because division)
    assert!(
        result.contains(r#"value="1.0""#),
        "Division should output with .0. Got: {}",
        result
    );
}

#[test]
fn test_float_prop_propagation() {
    // Case: float-ness propagates through intermediate properties
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="1.5"/>
  <xacro:property name="w2" value="${width * 2}"/>
  <test1 value="${width}"/>
  <test2 value="${w2}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs:
    // test1 value="1.5"
    // test2 value="3.0" (w2 inherits float-ness from width)
    assert!(
        result.contains(r#"value="1.5""#),
        "Original float property should keep decimal. Got: {}",
        result
    );
    assert!(
        result.contains(r#"value="3.0""#),
        "Computed float property should have .0. Got: {}",
        result
    );
}

#[test]
fn test_int_division_float_result() {
    // Case: int / int → float
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="diameter" value="4"/>
  <test value="${diameter/2}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs: value="2.0" (division produces float)
    assert!(
        result.contains(r#"value="2.0""#),
        "Int/int should output with .0 (division produces float). Got: {}",
        result
    );
}

#[test]
fn test_float_property_with_zero() {
    // Case: "0.0" property
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="damping" value="0.0"/>
  <test value="${damping}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs: value="0.0" (keeps .0)
    assert!(
        result.contains(r#"value="0.0""#),
        "Float property 0.0 should keep decimal. Got: {}",
        result
    );
}

#[test]
fn test_complex_expression_with_division() {
    // Case: complex expression with division
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="mass" value="1"/>
  <xacro:property name="y" value="1"/>
  <xacro:property name="z" value="1"/>
  <test value="${(1/12) * mass * (y*y + z*z)}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs with .0 because expression contains division
    // (1/12) * 1 * (1*1 + 1*1) = 2/12 = 0.166666...
    assert!(
        result.contains(r#"value="0.166"#) || result.contains(r#"value="0.16666"#),
        "Expression with division should output 0.166... Got: {}",
        result
    );
}

#[test]
fn test_mixed_int_and_float_properties() {
    // Case: Multiple properties with different types
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="int_val" value="5"/>
  <xacro:property name="float_val" value="5.0"/>
  <test1 value="${int_val}"/>
  <test2 value="${float_val}"/>
  <test3 value="${int_val * 2}"/>
  <test4 value="${float_val * 2}"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input).unwrap();

    // Python xacro outputs:
    // test1: "5" (int)
    // test2: "5.0" (float)
    // test3: "10" (int * int)
    // test4: "10.0" (float * int)
    assert!(
        result.contains(r#"<test1 value="5"/>"#) || result.contains(r#"<test1 value="5"></test1>"#),
        "Int property should output without .0. Got: {}",
        result
    );
    assert!(
        result.contains(r#"<test2 value="5.0"/>"#)
            || result.contains(r#"<test2 value="5.0"></test2>"#),
        "Float property should output with .0. Got: {}",
        result
    );
    assert!(
        result.contains(r#"<test3 value="10"/>"#)
            || result.contains(r#"<test3 value="10"></test3>"#),
        "Int * 2 should output without .0. Got: {}",
        result
    );
    assert!(
        result.contains(r#"<test4 value="10.0"/>"#)
            || result.contains(r#"<test4 value="10.0"></test4>"#),
        "Float * 2 should output with .0. Got: {}",
        result
    );
}

/// Integration test for number formatting with property metadata tracking
///
/// This test validates that we correctly format numbers to match Python xacro behavior:
/// - Properties with .0 in value string → always output with .0
/// - Properties without .0 → output without .0
/// - Division operations → always output with .0
/// - Float property used in expressions → propagate float-ness
mod common;
use crate::common::*;

#[test]
fn test_int_property_alone() {
    // Case: Property defined as "100" (no decimal)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="effort" value="100"/>
  <test value="${effort}"/>
</robot>"#;

    let output = run_xacro(input);
    // Python xacro outputs: value="100" (no .0)
    assert_xacro_contains!(
        output,
        r#"value="100""#,
        "Int property alone should output without .0"
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

    let output = run_xacro(input);
    // Python xacro outputs: value="100.0" (keeps .0)
    assert_xacro_contains!(
        output,
        r#"value="100.0""#,
        "Float property alone should output with .0"
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

    let output = run_xacro(input);
    // Python xacro outputs: value="30" (no .0)
    assert_xacro_contains!(
        output,
        r#"value="30""#,
        "Int * int should output without .0"
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

    let output = run_xacro(input);
    // Python xacro outputs: value="3.0" (keeps .0)
    assert_xacro_contains!(
        output,
        r#"value="3.0""#,
        "Float * int should output with .0"
    );
}

#[test]
fn test_division_operation() {
    // Case: division always produces float
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <test value="${255/255}"/>
</robot>"#;

    let output = run_xacro(input);
    // Python xacro outputs: value="1.0" (has .0 because division)
    assert_xacro_contains!(output, r#"value="1.0""#, "Division should output with .0");
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

    let output = run_xacro(input);
    // Python xacro outputs:
    // test1 value="1.5"
    // test2 value="3.0" (w2 inherits float-ness from width)
    assert_xacro_contains!(
        output,
        r#"value="1.5""#,
        "Original float property should keep decimal"
    );
    assert_xacro_contains!(
        output,
        r#"value="3.0""#,
        "Propagated float property should keep .0"
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

    let output = run_xacro(input);
    // Python xacro outputs: value="2.0" (division produces float)
    assert_xacro_contains!(
        output,
        r#"value="2.0""#,
        "Int/int should output with .0 (division produces float)"
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

    let output = run_xacro(input);
    // Python xacro outputs: value="0.0" (keeps .0)
    assert_xacro_contains!(
        output,
        r#"value="0.0""#,
        "Float property 0.0 should keep decimal"
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

    let output = run_xacro(input);
    let root = parse_xml(&output);
    let test_elem = find_child(&root, "test");
    // (1/12) * 1 * (1*1 + 1*1) = 2/12 = 0.166666...
    let expected = (1.0 / 12.0) * 1.0 * (1.0 * 1.0 + 1.0 * 1.0);
    assert_attr_float!(test_elem, "value", expected, 1e-9);
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

    let output = run_xacro(input);
    let root = parse_xml(&output);

    // test1: "5" (int property)
    let test1 = find_child(&root, "test1");
    assert_xacro_attr!(test1, "value", "5");

    // test2: "5.0" (float property)
    let test2 = find_child(&root, "test2");
    assert_xacro_attr!(test2, "value", "5.0");

    // test3: "10" (int * int)
    let test3 = find_child(&root, "test3");
    assert_xacro_attr!(test3, "value", "10");

    // test4: "10.0" (float * int)
    let test4 = find_child(&root, "test4");
    assert_xacro_attr!(test4, "value", "10.0");
}

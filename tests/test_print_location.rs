mod common;
use crate::common::*;

#[test]
fn test_print_location_nested_macros() {
    // Test that print_location() doesn't crash with nested macro calls
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <!-- Test nested macro calls with print_location -->

  <xacro:macro name="inner" params="name">
    <link name="${name}">
      <!-- Print location from innermost macro -->
      <visual><origin xyz="0 0 ${xacro.print_location()}0"/></visual>
    </link>
  </xacro:macro>

  <xacro:macro name="middle" params="prefix">
    <xacro:inner name="${prefix}_inner"/>
  </xacro:macro>

  <xacro:macro name="outer" params="base">
    <xacro:middle prefix="${base}_middle"/>
  </xacro:macro>

  <!-- Call the outer macro -->
  <xacro:outer base="test"/>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Processing with nested macros and print_location() should succeed",
    );

    // Verify the output contains expected link (print_location returns empty string)
    assert_xacro_contains!(
        output,
        r#"<link name="test_middle_inner">"#,
        "Output should contain the expected link name"
    );

    // Verify xyz attribute has the expected value (0 0 0, since print_location returns empty)
    assert_xacro_contains!(
        output,
        r#"xyz="0 0 0""#,
        "xyz should be '0 0 0' (print_location returns empty string)"
    );
}

#[test]
fn test_print_location_no_context() {
    // Test print_location() at top level (no macro context)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <link name="base">
    <visual><origin xyz="1 2 ${xacro.print_location()}3"/></visual>
  </link>
</robot>"#;

    let output = run_xacro_expect(
        input,
        "Processing with print_location() at top level should succeed",
    );

    // Verify xyz attribute (print_location returns empty, so "1 2 3")
    assert_xacro_contains!(output, r#"xyz="1 2 3""#, "xyz should be '1 2 3'");
}

#[test]
fn test_print_location_in_property() {
    // Test print_location() used in property definition
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="debug" value="${xacro.print_location()}"/>
  <link name="base">
    <visual><origin xyz="1 2 3${debug}"/></visual>
  </link>
</robot>"#;

    let output = run_xacro_expect(input, "print_location() in property definition should work");

    // Verify xyz attribute (print_location returns empty, so "1 2 3")
    assert_xacro_contains!(
        output,
        r#"xyz="1 2 3""#,
        "xyz should be '1 2 3' (debug is empty string)"
    );
}

#[test]
fn test_print_location_multiple_calls() {
    // Test multiple calls to print_location() in same document
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="test_macro" params="name">
    <link name="${name}">
      <visual><origin xyz="${xacro.print_location()}1 2 3"/></visual>
    </link>
  </xacro:macro>

  <xacro:test_macro name="link1"/>
  <xacro:test_macro name="link2"/>
</robot>"#;

    let output = run_xacro_expect(input, "Multiple print_location() calls should work");

    // Both links should be present
    assert_xacro_contains!(output, r#"<link name="link1">"#, "link1 should exist");
    assert_xacro_contains!(output, r#"<link name="link2">"#, "link2 should exist");

    // Both should have xyz="1 2 3"
    let link1_count = output.matches(r#"xyz="1 2 3""#).count();
    assert!(
        link1_count >= 2,
        "Should have at least 2 occurrences of xyz='1 2 3', found {}",
        link1_count
    );
}

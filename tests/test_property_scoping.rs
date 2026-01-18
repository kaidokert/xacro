// Tests for property scoping behavior
//
// These tests document Python xacro's property scoping rules:
// 1. Default (no scope): Property is LOCAL to current macro
// 2. scope="parent": Property is visible in parent macro
// 3. scope="global": Property is globally accessible
//
// CURRENT STATUS: Most tests FAIL due to property scoping bug
// All properties leak to global scope instead of respecting scope rules

mod common;
use crate::common::*;

// ============================================================================
// Test 1: Local properties should NOT leak outside macro
// ============================================================================

#[test]
fn test_property_local_scope_not_accessible_outside() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="define_prop">
    <xacro:property name="my_prop" value="local"/>
  </xacro:macro>
  <xacro:define_prop/>
  <link name="${my_prop}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Local property should not be accessible outside macro - expected error, got Ok"
    );
}

// ============================================================================
// Test 2: Local properties should shadow outer properties
// ============================================================================

#[test]
fn test_property_shadowing_in_nested_macros() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="outer">
    <xacro:property name="x" value="outer"/>
    <outer>${x}</outer>
    <xacro:macro name="inner">
      <xacro:property name="x" value="inner"/>
      <inner>${x}</inner>
    </xacro:macro>
    <xacro:inner/>
    <outer2>${x}</outer2>
  </xacro:macro>
  <xacro:outer/>
</robot>"#;

    let result = run_xacro(input);

    // Inner macro should see its own "x" property
    assert_xacro_contains!(
        result,
        "<inner>inner</inner>",
        "Inner macro should see its local 'x' property"
    );

    // Outer macro should see its "x" property before and after inner expansion
    assert_xacro_contains!(
        result,
        "<outer>outer</outer>",
        "Outer macro should see its 'x' property before inner"
    );
    assert_xacro_contains!(
        result,
        "<outer2>outer</outer2>",
        "Outer macro should still see its 'x' property after inner"
    );
}

// ============================================================================
// Test 3: Parent scope attribute makes property visible to parent
// ============================================================================

#[test]
fn test_property_parent_scope_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="outer">
    <xacro:macro name="inner">
      <xacro:property name="x" value="from_inner" scope="parent"/>
    </xacro:macro>
    <xacro:inner/>
    <link name="${x}"/>
  </xacro:macro>
  <xacro:outer/>
</robot>"#;

    let result = run_xacro(input);
    assert_xacro_contains!(
        result,
        r#"<link name="from_inner""#,
        "Parent scope property should be visible in parent macro"
    );
}

// ============================================================================
// Test 4: Global scope attribute makes property globally accessible
// ============================================================================

#[test]
fn test_property_global_scope_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="define_global">
    <xacro:property name="global_prop" value="globally_visible" scope="global"/>
  </xacro:macro>
  <xacro:define_global/>
  <link name="${global_prop}"/>
</robot>"#;

    let result = run_xacro(input);
    assert_xacro_contains!(
        result,
        r#"<link name="globally_visible""#,
        "Global scope property should be accessible outside macro"
    );
}

// ============================================================================
// Test 5: Nested macros CAN access outer's local properties during expansion
// This is Python xacro's actual behavior - when inner macro is called during
// outer's expansion, it executes in the outer's scope and can see outer's
// local properties.
// ============================================================================

#[test]
fn test_nested_macro_can_access_outer_local_property() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="outer">
    <xacro:property name="outer_local" value="local_to_outer"/>
    <xacro:macro name="inner">
      <link name="${outer_local}"/>
    </xacro:macro>
    <xacro:inner/>
  </xacro:macro>
  <xacro:outer/>
</robot>"#;

    let result = run_xacro(input);
    assert_xacro_contains!(
        result,
        r#"<link name="local_to_outer""#,
        "Inner macro should see outer's local property during expansion"
    );
}

// ============================================================================
// Test 6: Property cleanup after macro expansion
// ============================================================================

#[test]
fn test_property_cleanup_after_macro_expansion() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="define_temp">
    <xacro:property name="temp" value="temporary"/>
    <link1 name="${temp}"/>
  </xacro:macro>
  <xacro:define_temp/>
  <link2 name="${temp}"/>
</robot>"#;

    let result = test_xacro(input);
    match result {
        Ok(output) => {
            panic!("Property 'temp' should be cleaned up after macro expansion, but got Ok. Output:\n{}", output);
        }
        Err(e) => {
            // Check error message contains "not defined" or similar
            let error_msg = format!("{:?}", e);
            assert!(
                error_msg.contains("not defined") || error_msg.contains("Undefined"),
                "Error should mention undefined property: {}",
                error_msg
            );
        }
    }
}

// ============================================================================
// Test 7: Multiple macro calls don't interfere
// ============================================================================

#[test]
fn test_multiple_macro_calls_isolated_properties() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="make_link" params="suffix">
    <xacro:property name="link_name" value="link_${suffix}"/>
    <link name="${link_name}"/>
  </xacro:macro>

  <xacro:make_link suffix="1"/>
  <xacro:make_link suffix="2"/>
</robot>"#;

    let result = run_xacro(input);

    // Each macro call should have its own local property
    assert_xacro_contains!(
        result,
        r#"<link name="link_1""#,
        "First macro call should produce link_1"
    );
    assert_xacro_contains!(
        result,
        r#"<link name="link_2""#,
        "Second macro call should produce link_2"
    );
}

// ============================================================================
// Test 8: Lazy properties (body-based) should also respect scoping
// ============================================================================

#[test]
fn test_lazy_property_local_scope() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="define_block">
    <xacro:property name="local_block">
      <link name="test_link"/>
    </xacro:property>
  </xacro:macro>
  <xacro:define_block/>
  <xacro:insert_block name="local_block"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Lazy property should not be accessible outside macro - expected error, got Ok"
    );
}

// ============================================================================
// Test 9: Global scope on lazy properties
// ============================================================================

#[test]
fn test_lazy_property_global_scope() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="define_block">
    <xacro:property name="global_block" scope="global">
      <link name="test_link"/>
    </xacro:property>
  </xacro:macro>
  <xacro:define_block/>
  <xacro:insert_block name="global_block"/>
</robot>"#;

    let result = run_xacro(input);
    assert_xacro_contains!(
        result,
        r#"<link name="test_link""#,
        "Global lazy property should be accessible outside macro"
    );
}

// ============================================================================
// Test 10: Parent scope on lazy properties
// ============================================================================

#[test]
fn test_lazy_property_parent_scope() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="outer">
    <xacro:macro name="inner">
      <xacro:property name="parent_block" scope="parent">
        <link name="test_link"/>
      </xacro:property>
    </xacro:macro>
    <xacro:inner/>
    <xacro:insert_block name="parent_block"/>
  </xacro:macro>
  <xacro:outer/>
</robot>"#;

    let result = run_xacro(input);
    assert_xacro_contains!(
        result,
        r#"<link name="test_link""#,
        "Parent scope lazy property should be accessible in parent macro"
    );
}

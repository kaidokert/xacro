// RefCell Safety Validation Tests
//
// These tests verify that the scope manipulation implementation doesn't trigger
// RefCell panics under various stress scenarios:
// - Deep macro nesting (10+ levels)
// - Complex property forwarding chains
// - Mixed scope attribute usage
// - Recursive macro expansion

mod common;
use common::*;

#[test]
fn test_deep_nesting_forward() {
    // Test deep nesting with property forwarding (10 levels)
    // Verifies no RefCell panic with many scope_depth() calls
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="value" value="deep"/>

  <xacro:macro name="level10" params="value:=^">
    <l10 v="${value}"/>
  </xacro:macro>

  <xacro:macro name="level9" params="value:=^">
    <xacro:level10/>
  </xacro:macro>

  <xacro:macro name="level8" params="value:=^">
    <xacro:level9/>
  </xacro:macro>

  <xacro:macro name="level7" params="value:=^">
    <xacro:level8/>
  </xacro:macro>

  <xacro:macro name="level6" params="value:=^">
    <xacro:level7/>
  </xacro:macro>

  <xacro:macro name="level5" params="value:=^">
    <xacro:level6/>
  </xacro:macro>

  <xacro:macro name="level4" params="value:=^">
    <xacro:level5/>
  </xacro:macro>

  <xacro:macro name="level3" params="value:=^">
    <xacro:level4/>
  </xacro:macro>

  <xacro:macro name="level2" params="value:=^">
    <xacro:level3/>
  </xacro:macro>

  <xacro:macro name="level1" params="value:=^">
    <xacro:level2/>
  </xacro:macro>

  <xacro:level1/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"v="deep""#);
}

#[test]
fn test_deep_nesting_scope_parent() {
    // Test deep nesting with scope="parent" (10 levels)
    // Verifies no RefCell panic with many define_property() calls
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="level10">
    <xacro:property name="v10" value="10" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level9">
    <xacro:level10/>
    <xacro:property name="v9" value="${v10}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level8">
    <xacro:level9/>
    <xacro:property name="v8" value="${v9}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level7">
    <xacro:level8/>
    <xacro:property name="v7" value="${v8}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level6">
    <xacro:level7/>
    <xacro:property name="v6" value="${v7}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level5">
    <xacro:level6/>
    <xacro:property name="v5" value="${v6}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level4">
    <xacro:level5/>
    <xacro:property name="v4" value="${v5}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level3">
    <xacro:level4/>
    <xacro:property name="v3" value="${v4}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level2">
    <xacro:level3/>
    <xacro:property name="v2" value="${v3}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level1">
    <xacro:level2/>
    <link name="${v2}"/>
  </xacro:macro>

  <xacro:level1/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"name="10""#);
}

#[test]
fn test_deep_nesting_scope_global() {
    // Test deep nesting with scope="global" from various depths
    // Verifies RefCell safety when writing to global from deep nesting
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="deep3">
    <xacro:property name="from_deep3" value="d3" scope="global"/>
  </xacro:macro>

  <xacro:macro name="deep2">
    <xacro:property name="from_deep2" value="d2" scope="global"/>
    <xacro:deep3/>
  </xacro:macro>

  <xacro:macro name="deep1">
    <xacro:property name="from_deep1" value="d1" scope="global"/>
    <xacro:deep2/>
  </xacro:macro>

  <xacro:deep1/>

  <!-- All should be accessible at global scope -->
  <link name="${from_deep1}_${from_deep2}_${from_deep3}"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"name="d1_d2_d3""#);
}

#[test]
fn test_mixed_forward_and_scope() {
    // Test mixing ^ operator and scope attributes in complex nesting
    // Verifies no RefCell conflicts between lookup_at_depth and define_property
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base" value="100"/>

  <xacro:macro name="inner" params="base:=^">
    <!-- Forward from parent, then define in parent scope -->
    <xacro:property name="computed" value="${base * 2}" scope="parent"/>
    <!-- Also define in global scope -->
    <xacro:property name="global_computed" value="${base * 3}" scope="global"/>
  </xacro:macro>

  <xacro:macro name="middle">
    <xacro:inner/>
    <!-- Use computed from inner (forwarded to this scope) -->
    <xacro:property name="middle_value" value="${computed}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="outer">
    <xacro:middle/>
    <!-- Use middle_value (forwarded here) -->
    <link v="${middle_value}"/>
  </xacro:macro>

  <xacro:outer/>

  <!-- Global computed should also be accessible -->
  <joint v="${global_computed}"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"v="200""#);
    assert_xacro_contains!(&output, r#"v="300""#);
}

#[test]
fn test_multiple_forwards_with_defaults() {
    // Test multiple parameters with ^ operator and defaults
    // Verifies no RefCell issues with multiple lookup_at_depth calls
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="p1" value="1"/>
  <xacro:property name="p2" value="2"/>
  <xacro:property name="p3" value="3"/>

  <xacro:macro name="test" params="p1:=^|10 p2:=^|20 p3:=^|30 p4:=^|40 p5:=^|50">
    <result v="${p1}_${p2}_${p3}_${p4}_${p5}"/>
  </xacro:macro>

  <xacro:test/>
</robot>"#;

    let output = run_xacro(input);
    // p1, p2, p3 from global; p4, p5 from defaults
    assert_xacro_contains!(&output, r#"v="1_2_3_40_50""#);
}

#[test]
fn test_scope_parent_chain() {
    // Test chaining scope="parent" through multiple levels
    // Verifies RefCell safety with cascading parent scope writes
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="level3">
    <xacro:property name="chain" value="start" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level2">
    <xacro:level3/>
    <!-- chain is now in level2 scope -->
    <xacro:property name="chain" value="${chain}_mid" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="level1">
    <xacro:level2/>
    <!-- chain is now in level1 scope -->
    <link name="${chain}_end"/>
  </xacro:macro>

  <xacro:level1/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"name="start_mid_end""#);
}

#[test]
fn test_concurrent_scope_operations() {
    // Test simultaneous forwards, parent writes, and global writes
    // Maximum stress on RefCell borrow checking
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="g1" value="global1"/>
  <xacro:property name="g2" value="global2"/>

  <xacro:macro name="worker" params="g1:=^ g2:=^">
    <!-- Forward from parent -->
    <xacro:property name="local" value="${g1}_${g2}"/>
    <!-- Write to parent -->
    <xacro:property name="to_parent" value="${local}_parent" scope="parent"/>
    <!-- Write to global -->
    <xacro:property name="to_global" value="${local}_global" scope="global"/>
    <!-- Local usage -->
    <item v="${local}"/>
  </xacro:macro>

  <xacro:macro name="wrapper">
    <xacro:worker/>
    <parent v="${to_parent}"/>
  </xacro:macro>

  <xacro:wrapper/>
  <global v="${to_global}"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"v="global1_global2""#);
    assert_xacro_contains!(&output, r#"v="global1_global2_parent""#);
    assert_xacro_contains!(&output, r#"v="global1_global2_global""#);
}

#[test]
fn test_forward_with_expression_evaluation() {
    // Test that forwarding works correctly with complex expressions
    // Verifies RefCell safety during substitute_text calls
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base" value="10"/>
  <xacro:property name="multiplier" value="5"/>

  <xacro:macro name="compute" params="base:=^ multiplier:=^">
    <xacro:property name="result" value="${base * multiplier + 100}" scope="parent"/>
  </xacro:macro>

  <xacro:macro name="outer">
    <xacro:compute/>
    <link v="${result}"/>
  </xacro:macro>

  <xacro:outer/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"v="150""#);
}

#[test]
fn test_nested_macro_calls_with_forwarding() {
    // Test macro calling another macro, both using ^ operator
    // Verifies depth tracking across macro boundaries
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="value" value="root"/>

  <xacro:macro name="inner" params="value:=^">
    <inner v="${value}"/>
  </xacro:macro>

  <xacro:macro name="middle" params="value:=^">
    <xacro:inner/>
  </xacro:macro>

  <xacro:macro name="outer" params="value:=^">
    <xacro:middle/>
  </xacro:macro>

  <xacro:outer/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(&output, r#"v="root""#);
}

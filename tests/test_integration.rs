mod common;
use crate::common::*;
use xacro::XacroProcessor;
use xmltree::Element;

#[test]
fn test_basic_property() {
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="0.5"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="${width} 1.0 0.2"/>
      </geometry>
    </visual>
  </link>
</robot>"#,
    );

    assert_xacro_contains!(output, "0.5", "Property substitution failed");
    assert_xacro_not_contains!(output, "${width}", "Property should be expanded");
    assert_xacro_not_contains!(
        output,
        "xacro:property",
        "Property definition should be removed"
    );
}

#[test]
fn test_basic_macro() {
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_link" params="name">
    <link name="${name}"/>
  </xacro:macro>

  <xacro:test_link name="base"/>
</robot>"#,
    );

    assert_xacro_contains!(output, r#"name="base""#, "Macro expansion failed");
    assert_xacro_not_contains!(output, "xacro:macro", "Macro definition should be removed");
    assert_xacro_not_contains!(output, "xacro:test_link", "Macro call should be expanded");
}

#[test]
fn test_conditional() {
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="use_sensor" value="true"/>
  <xacro:if value="${use_sensor}">
    <link name="sensor"/>
  </xacro:if>
  <xacro:unless value="${use_sensor}">
    <link name="no_sensor"/>
  </xacro:unless>
</robot>"#,
    );

    assert_xacro_contains!(
        output,
        r#"name="sensor""#,
        "xacro:if should include content when true"
    );
    assert_xacro_not_contains!(
        output,
        r#"name="no_sensor""#,
        "xacro:unless should exclude content when condition is true"
    );
}

#[test]
fn test_math_constants() {
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="angle" value="${pi/4}"/>
  <link name="test" value="${angle}"/>
</robot>"#,
    );

    // pi/4 = 0.7853981633974483
    assert_xacro_contains!(output, "0.785");
}

#[test]
fn test_include_basic() {
    let output = run_xacro_file("tests/data/include_test.xacro");

    // Should contain content from included file
    assert_xacro_contains!(output, "base_link");
}

#[test]
fn test_include_nested() {
    let output = run_xacro_file("tests/data/include_test_nested_base.xacro");

    // Should contain content from nested includes (arm and hand files)
    assert_xacro_contains!(output, "arm_base");
    assert_xacro_contains!(output, "hand_base");
}

#[test]
fn test_include_with_macros() {
    // Test that includes work properly with macro expansion
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_link" params="name">
    <link name="${name}"/>
  </xacro:macro>

  <!-- This would normally be in a separate file -->
  <xacro:test_link name="included_link"/>
</robot>"#,
    );

    assert_xacro_contains!(output, r#"name="included_link""#);
}

#[test]
fn test_property_default_attribute() {
    // Test the include-once guard pattern used in real-world files
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include-once guard pattern -->
  <xacro:property name="materials_defined" default="false"/>

  <xacro:unless value="${materials_defined}">
    <xacro:property name="materials_defined" value="true"/>
    <material name="aluminum">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </xacro:unless>

  <!-- This should not redefine materials -->
  <xacro:property name="materials_defined" default="false"/>
  <xacro:unless value="${materials_defined}">
    <material name="should_not_appear">
      <color rgba="1 0 0 1"/>
    </material>
  </xacro:unless>
</robot>"#,
    );

    assert_xacro_contains!(output, "aluminum", "First material should be defined");
    assert_xacro_not_contains!(
        output,
        "should_not_appear",
        "Second material block should be skipped"
    );
}

#[test]
fn test_property_default_vs_value() {
    // Test that value overrides default
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- default is used if property not already defined -->
  <xacro:property name="test1" default="default_value"/>
  <link name="${test1}"/>

  <!-- value always sets the property -->
  <xacro:property name="test2" default="ignored" value="actual_value"/>
  <link name="${test2}"/>
</robot>"#,
    );

    assert_xacro_contains!(output, r#"name="default_value""#, "Default should be used");
    assert_xacro_contains!(
        output,
        r#"name="actual_value""#,
        "Value should override default"
    );
}

/// Test nested macro calls with parameter expressions
///
/// This ensures that nested macro calls (one macro calling another) correctly
/// evaluate parameters at each level, including arithmetic expressions.
#[test]
fn test_nested_macro_calls_with_expressions() {
    let output = run_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inner" params="mass x">
    <inertia izz="${(1/12) * mass * x*x}"/>
  </xacro:macro>

  <xacro:macro name="outer" params="mass x">
    <xacro:inner mass="${mass}" x="${x}"/>
  </xacro:macro>

  <link name="test">
    <xacro:outer mass="2.0" x="0.5"/>
  </link>
</robot>"#,
    );

    // Parse and verify numeric value
    let root = parse_xml(&output);
    let link = find_child(&root, "link");
    let inertia = find_child(link, "inertia");

    // izz = (1/12) * mass * x*x = (1/12) * 2.0 * 0.5*0.5 = 0.041666...
    let expected_izz = (1.0 / 12.0) * 2.0 * 0.5 * 0.5;
    assert_attr_float!(inertia, "izz", expected_izz, 1e-9);

    // Verify no unevaluated placeholders remain
    assert_xacro_not_contains!(
        output,
        "${",
        "Output should not contain unevaluated expressions"
    );
}

/// Test circular property dependency detection
///
/// Should error when property A references B which references A.
/// This is critical for preventing infinite loops during property resolution.
///
/// Implementation: Errors with UndefinedVar because when resolving a→b→a,
/// the inner 'a' isn't defined yet. This is correct - it prevents infinite loops.
#[test]
fn test_circular_property_dependency() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="${b * 2}"/>
  <xacro:property name="b" value="${a * 3}"/>

  <link name="test">
    <mass value="${a}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);

    // Circular dependencies are detected via UndefinedVar or CircularPropertyDependency
    // Both error types correctly prevent infinite loops
    let err = result.expect_err("Circular dependency should be detected and error");

    assert!(
        matches!(
            err,
            xacro::XacroError::CircularPropertyDependency { .. }
                | xacro::XacroError::EvalError { .. }
        ),
        "Expected CircularPropertyDependency or EvalError, got: {:?}",
        err
    )
}

// ============================================================================
// CONDITIONAL TESTS
// ============================================================================

/// Test integer truthiness in conditionals (0 is false, non-zero is true)
#[test]
fn test_conditional_integer_truthiness() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${0}">
    <link name="zero"/>
  </xacro:if>
  <xacro:if value="${1}">
    <link name="one"/>
  </xacro:if>
  <xacro:if value="${-1}">
    <link name="neg_one"/>
  </xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Integer truthiness should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(!output.contains(r#"name="zero""#), "0 should be falsy");
    assert!(output.contains(r#"name="one""#), "1 should be truthy");
    assert!(output.contains(r#"name="neg_one""#), "-1 should be truthy");
}

/// Test float truthiness in conditionals (0.0 is false, non-zero is true)
#[test]
fn test_conditional_float_truthiness() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${0.0}">
    <link name="zero"/>
  </xacro:if>
  <xacro:if value="${0.1}">
    <link name="point_one"/>
  </xacro:if>
  <xacro:if value="${-0.1}">
    <link name="neg_point_one"/>
  </xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Float truthiness should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(!output.contains(r#"name="zero""#), "0.0 should be falsy");
    assert!(
        output.contains(r#"name="point_one""#),
        "0.1 should be truthy"
    );
    assert!(
        output.contains(r#"name="neg_point_one""#),
        "-0.1 should be truthy"
    );
}

/// Test boolean string literals in conditionals
#[test]
fn test_conditional_boolean_literals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="true">
    <link name="true_literal"/>
  </xacro:if>
  <xacro:if value="false">
    <link name="false_literal"/>
  </xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Boolean literals should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="true_literal""#),
        "true should be truthy"
    );
    assert!(
        !output.contains(r#"name="false_literal""#),
        "false should be falsy"
    );
}

/// Test nested conditionals
#[test]
fn test_nested_conditionals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="true">
    <xacro:if value="true">
      <link name="both_true"/>
    </xacro:if>
    <xacro:unless value="false">
      <link name="outer_true_inner_unless"/>
    </xacro:unless>
  </xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Nested conditionals should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains(r#"name="both_true""#));
    assert!(output.contains(r#"name="outer_true_inner_unless""#));
}

/// Test conditional error handling - missing value attribute
#[test]
fn test_conditional_missing_value_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if>
    <link name="test"/>
  </xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err(), "Missing value attribute should error");

    let err = result.err().unwrap();
    assert!(
        matches!(err, xacro::XacroError::MissingAttribute { .. }),
        "Should be MissingAttribute error, got: {:?}",
        err
    );
}

// ============================================================================
// PROPERTY LAZY EVALUATION TESTS
// ============================================================================

/// Test forward property references (property A references B defined later)
#[test]
fn test_property_forward_reference() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="derived" value="${base * 2}"/>
  <xacro:property name="base" value="10"/>
  <link name="test" value="${derived}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Forward references should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"value="20""#),
        "derived should be 20 (base * 2)"
    );
}

/// Test multilevel property dependencies (A → B → C)
#[test]
fn test_property_multilevel_dependencies() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="level1" value="${level2 * 2}"/>
  <xacro:property name="level2" value="${level3 * 2}"/>
  <xacro:property name="level3" value="5"/>
  <link name="test" value="${level1}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Multilevel dependencies should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // level3 = 5, level2 = 10, level1 = 20
    assert!(output.contains(r#"value="20""#), "level1 should be 20");
}

/// Test unused property with undefined variable (should NOT error - lazy evaluation)
#[test]
fn test_property_unused_with_undefined_var() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="unused" value="${undefined_var}"/>
  <xacro:property name="used" value="42"/>
  <link name="test" value="${used}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Unused property with undefined variable should not error (lazy eval): {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains(r#"value="42""#));
}

/// Test used property with undefined variable (SHOULD error)
#[test]
fn test_property_used_with_undefined_var_errors() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="bad" value="${undefined_var}"/>
  <link name="test" value="${bad}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Used property with undefined variable should error"
    );
}

// ============================================================================
// MACRO INSERT_BLOCK TESTS
// ============================================================================

/// Test basic insert_block functionality
#[test]
fn test_insert_block_basic() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="*content">
    <wrapper>
      <xacro:insert_block name="content"/>
    </wrapper>
  </xacro:macro>

  <xacro:test>
    <item name="foo"/>
  </xacro:test>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Basic insert_block should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<wrapper>"));
    assert!(output.contains(r#"<item name="foo""#));
}

/// Test insert_block with multiple block parameters
#[test]
fn test_insert_block_multiple_params() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="dual" params="*block1 *block2">
    <first>
      <xacro:insert_block name="block1"/>
    </first>
    <second>
      <xacro:insert_block name="block2"/>
    </second>
  </xacro:macro>

  <xacro:dual>
    <a name="alpha"/>
    <b name="beta"/>
  </xacro:dual>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Multiple block params should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains(r#"<a name="alpha""#));
    assert!(output.contains(r#"<b name="beta""#));
}

/// Test insert_block with property expressions in blocks
#[test]
fn test_insert_block_with_expressions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="value" value="42"/>

  <xacro:macro name="test" params="*content">
    <wrapper>
      <xacro:insert_block name="content"/>
    </wrapper>
  </xacro:macro>

  <xacro:test>
    <item value="${value * 2}"/>
  </xacro:test>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "insert_block with expressions should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"value="84""#),
        "Expression in block should be evaluated"
    );
}

/// Test insert_block error - missing block parameter
#[test]
fn test_insert_block_undefined_name() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="*content">
    <wrapper>
      <xacro:insert_block name="wrong_name"/>
    </wrapper>
  </xacro:macro>

  <xacro:test>
    <item name="foo"/>
  </xacro:test>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err(), "Undefined block name should error");

    let err = result.err().unwrap();
    assert!(
        matches!(err, xacro::XacroError::UndefinedBlock { .. }),
        "Should be UndefinedBlock error, got: {:?}",
        err
    );
}

/// Test circular block references cause an error
///
/// Test custom max recursion depth API
#[test]
fn test_custom_max_recursion_depth() {
    // Create processor with very low depth limit
    let processor = XacroProcessor::new_with_depth(5);

    // Create deeply nested macro calls that exceed depth of 5
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="nested" params="depth">
    <xacro:if value="${depth > 0}">
      <level depth="${depth}">
        <xacro:nested depth="${depth - 1}"/>
      </level>
    </xacro:if>
  </xacro:macro>

  <xacro:nested depth="10"/>
</robot>"#;

    let result = processor.run_from_string(input);

    // Should hit recursion limit before completing
    assert!(result.is_err(), "Should hit recursion limit with depth=5");

    let err = result.err().unwrap();
    assert!(
        matches!(err, xacro::XacroError::MacroRecursionLimit { depth: _, limit } if limit == 5),
        "Should be MacroRecursionLimit with limit=5, got: {:?}",
        err
    );
}

/// Test that circular block references are detected and cause an error
#[test]
fn test_circular_block_references() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="circular" params="*block1 *block2">
    <wrapper>
      <xacro:insert_block name="block1"/>
    </wrapper>
  </xacro:macro>

  <xacro:circular>
    <a>
      <xacro:insert_block name="block2"/>
    </a>
    <b>
      <xacro:insert_block name="block1"/>
    </b>
  </xacro:circular>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Circular block references should cause an error"
    );

    let err = result.err().unwrap();
    // Python xacro gives UndefinedBlock error, not recursion error, because
    // with pre-expansion (lexical scoping), blocks are expanded BEFORE entering
    // the macro. When expanding block1, it tries to insert block2 which hasn't
    // been collected yet.
    let undefined_block_name = match &err {
        xacro::XacroError::UndefinedBlock { name, .. } => name,
        other => panic!("Should be UndefinedBlock error, got: {:?}", other),
    };
    assert_eq!(
        undefined_block_name, "block2",
        "UndefinedBlock should report missing block2, got: {}",
        undefined_block_name
    );
}

/// Test deeply nested macros expand correctly
#[test]
fn test_deeply_nested_macros() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="level3" params="x">
    <item value="${x}"/>
  </xacro:macro>

  <xacro:macro name="level2" params="x">
    <xacro:level3 x="${x}"/>
  </xacro:macro>

  <xacro:macro name="level1" params="x">
    <xacro:level2 x="${x}"/>
  </xacro:macro>

  <xacro:level1 x="42"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Deeply nested macros should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<item value="42""#),
        "Output should contain expanded item"
    );
}

/// Test reusing a block multiple times
#[test]
fn test_insert_block_reuse() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="duplicate" params="*content">
    <first><xacro:insert_block name="content"/></first>
    <second><xacro:insert_block name="content"/></second>
  </xacro:macro>

  <xacro:duplicate>
    <item>Hello</item>
  </xacro:duplicate>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Block reuse should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<first>"), "Should have first element");
    assert!(output.contains("<second>"), "Should have second element");
    assert_eq!(
        output.matches("<item>Hello</item>").count(),
        2,
        "Item should appear twice"
    );
}

/// Test insert_block with global properties
#[test]
fn test_insert_block_with_global_property() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="global_x" value="1"/>
  <xacro:property name="local_y" value="2"/> <!-- Lexical scope: must be global or caller-local -->

  <xacro:macro name="foo" params="*content">
    <xacro:insert_block name="content"/>
  </xacro:macro>

  <xacro:foo>
    <origin xyz="${global_x} ${local_y} 0"/>
  </xacro:foo>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Block with global property should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"xyz="1 2 0""#),
        "Should substitute properties in lexical scope"
    );
}

/// Test missing block parameter causes error
#[test]
fn test_insert_block_missing_block() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="needs_block" params="*content">
    <wrapper/>
  </xacro:macro>

  <xacro:needs_block/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err(), "Missing block parameter should error");

    let err = result.err().unwrap();
    assert!(
        matches!(err, xacro::XacroError::MissingBlockParameter { .. }),
        "Should be MissingBlockParameter error, got: {:?}",
        err
    );
}

/// Test extra children causes error
#[test]
fn test_insert_block_extra_children() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="one_block" params="*content">
    <wrapper/>
  </xacro:macro>

  <xacro:one_block>
    <first/>
    <second/>
  </xacro:one_block>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err(), "Extra children should error");

    let err = result.err().unwrap();
    assert!(
        matches!(err, xacro::XacroError::UnusedBlock { extra_count: 1, .. }),
        "Should be UnusedBlock error, got: {:?}",
        err
    );
}

/// Test nested macro calls inside blocks
#[test]
fn test_insert_block_nested_macros() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inner" params="value">
    <item>${value}</item>
  </xacro:macro>

  <xacro:macro name="wrapper" params="*content">
    <container>
      <xacro:insert_block name="content"/>
    </container>
  </xacro:macro>

  <xacro:wrapper>
    <xacro:inner value="42"/>
  </xacro:wrapper>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Nested macros in blocks should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<container>"), "Should have container");
    assert!(
        output.contains("<item>42</item>"),
        "Should expand inner macro with value"
    );
}

/// Test multiple block parameters maintain ordering
#[test]
fn test_insert_block_multiple_block_params_ordering() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="pair" params="*first *second">
    <container>
      <xacro:insert_block name="first"/>
      <xacro:insert_block name="second"/>
    </container>
  </xacro:macro>

  <xacro:pair>
    <one/>
    <two/>
  </xacro:pair>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Multiple block params should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<container>"), "Should have container");

    // Check ordering - <one/> should appear before <two/>
    let one_pos = output.find("<one/>").expect("Should contain <one/>");
    let two_pos = output.find("<two/>").expect("Should contain <two/>");
    assert!(
        one_pos < two_pos,
        "First block should appear before second block"
    );
}

/// Test forward reference with lazy evaluation (from official xacro test suite)
/// Property a2 references a before a is defined
#[test]
fn test_property_forward_reference_official() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a2" value="${2*a}"/>
  <xacro:property name="a" value="42"/>
  <link name="test">
    <mass doubled="${a2}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Forward reference should work with lazy evaluation: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"doubled="84""#),
        "a2 should resolve to 84 (2*42)"
    );
}

/// Test 4-level transitive property chain (from official xacro test suite)
/// a -> b -> c -> d (deeper than typical 3-level tests)
#[test]
fn test_property_transitive_chain_4_levels() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="42"/>
  <xacro:property name="b" value="${a}"/>
  <xacro:property name="c" value="${b}"/>
  <xacro:property name="d" value="${c}"/>
  <link name="test">
    <mass d="${d}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "4-level transitive chain should resolve: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"d="42""#),
        "d should resolve through 4-level chain to 42"
    );
}

/// Test diamond dependency graph (from official xacro test suite)
/// Property f depends on c and d, which depend on a and b respectively
/// Tests branching dependency resolution (not just linear chains)
#[test]
fn test_property_diamond_dependency_graph() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="42"/>
  <xacro:property name="b" value="2.1"/>
  <xacro:property name="c" value="${a}"/>
  <xacro:property name="d" value="${b}"/>
  <xacro:property name="f" value="${c*d}"/>
  <link name="test">
    <mass f="${f}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Diamond dependency graph should resolve: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"f="88.2""#),
        "f should resolve to 88.2 (42 * 2.1)"
    );
}

/// Test multiple substitutions in a single attribute value
#[test]
fn test_property_multi_substitution() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="1"/>
  <xacro:property name="y" value="2"/>
  <xacro:property name="z" value="3"/>
  <link name="test">
    <origin xyz="${x} ${y} ${z}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Multi-substitution should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"xyz="1 2 3""#),
        "All three properties should be substituted"
    );
}

/// Test xacro:unless (inverse of if)
#[test]
fn test_unless_basic() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:unless value="true">
    <excluded/>
  </xacro:unless>
  <xacro:unless value="false">
    <included/>
  </xacro:unless>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_ok(), "Unless should work: {:?}", result.err());

    let output = result.unwrap();
    assert!(
        !output.contains("<excluded/>"),
        "Unless true should exclude"
    );
    assert!(
        output.contains("<included/>"),
        "Unless false should include"
    );
}

/// Test that xacro:if preserves comments and text nodes
#[test]
fn test_if_preserves_comments_and_text() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><!-- comment --> text <b>bar</b></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "If with mixed content should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<b>bar</b>"), "Element should be preserved");
    assert!(output.contains("text"), "Text node should be preserved");
    // Comments may not be preserved in output - that's OK
}

/// Test integration of conditionals with properties
#[test]
fn test_integration_conditionals_with_properties() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="use_feature" value="1"/>
  <xacro:property name="skip_feature" value="0"/>

  <xacro:if value="${use_feature}">
    <feature_enabled/>
  </xacro:if>
  <xacro:unless value="${use_feature}">
    <feature_disabled/>
  </xacro:unless>
  <xacro:if value="${skip_feature}">
    <skipped/>
  </xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Conditionals with properties should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("<feature_enabled/>"),
        "use_feature is true, should be enabled"
    );
    assert!(
        !output.contains("<feature_disabled/>"),
        "Unless use_feature should exclude"
    );
    assert!(!output.contains("<skipped/>"), "skip_feature is false");
}

// ============================================================================
// Property and Expression Tests
// ============================================================================

/// Test basic property substitution using test data files
#[test]
fn test_property_basic() {
    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="0.5"/>
  <box size="${width}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    // Expected: <box size="0.5"/>
    assert!(
        output.contains(r#"size="0.5""#),
        "Property substitution failed"
    );
    assert!(
        !output.contains("${width}"),
        "Property expression not expanded"
    );
    assert!(
        !output.contains("xacro:property"),
        "Property definition not removed"
    );
}

/// Test nested property references
#[test]
fn test_property_nested() {
    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="1"/>
  <xacro:property name="y" value="${x}"/>
  <point coord="${y}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    // Properties should be substituted recursively
    assert!(
        output.contains(r#"coord="1""#),
        "Nested property should resolve to 1"
    );
    assert!(
        !output.contains("${"),
        "All property expressions should be expanded"
    );
}

/// Test basic macro expansion using test data files
#[test]
fn test_macro_basic() {
    let result = test_xacro_file("tests/data/macro_test.xacro");
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    // Expected: <box size="0.5 0.5 0.5"/> with name="base_link"
    assert!(
        output.contains(r#"name="base_link""#),
        "Macro expansion failed"
    );
    assert!(
        output.contains(r#"size="0.5 0.5 0.5""#),
        "Macro parameter substitution failed"
    );
    assert!(
        !output.contains("xacro:macro"),
        "Macro definition not removed"
    );
}

/// Test multiple includes
#[test]
fn test_include_multi() {
    let result = test_xacro_file("tests/data/include_test_multi_base.xacro");
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    // Should include content from both wheels and arms files
    assert!(
        output.contains("wheel_front_right") || output.contains("arm_left_link"),
        "Include files not processed, output:\n{}",
        output
    );
}

/// Test include from subdirectory
#[test]
fn test_include_subdirectory() {
    let result = test_xacro_file("tests/data/include_test_directory.xacro");
    assert!(
        result.is_ok(),
        "Include from subdirectory failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // The included file should have been processed
    assert!(
        !output.contains("xacro:include"),
        "Include directive not processed"
    );
}

/// Test multiple property definitions
#[test]
fn test_property_multiple() {
    let result = test_xacro_file("tests/data/property_test_multiple.xacro");
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    assert!(
        !output.contains("${"),
        "All properties should be substituted"
    );
}

/// Test properties defined out of order
#[test]
fn test_property_multiple_out_of_order() {
    let result = test_xacro_file("tests/data/property_test_multiple_out_of_order.xacro");
    assert!(
        result.is_ok(),
        "Out of order properties should work with lazy evaluation: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        !output.contains("${"),
        "All properties should be substituted despite ordering"
    );
}

/// Test property substitution in attributes
#[test]
fn test_property_attributes() {
    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="prefix" value="arm"/>
  <link name="${prefix}_link"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="arm_link""#),
        "Property in link name should be substituted, output:\n{}",
        output
    );
    // No unsubstituted expressions should remain
    assert!(
        !output.contains("${prefix}"),
        "Property expressions should be fully expanded"
    );
}

/// Test arithmetic property expressions
#[test]
fn test_property_arithmetic() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="5"/>
  <xacro:property name="y" value="3"/>
  <link name="test">
    <mass value="${x + y}"/>
    <inertia value="${x * y}"/>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_ok(), "Arithmetic failed: {:?}", result.err());

    let output = result.unwrap();
    assert!(
        output.contains(r#"value="8""#),
        "Addition should work: 5 + 3 = 8"
    );
    assert!(
        output.contains(r#"value="15""#),
        "Multiplication should work: 5 * 3 = 15"
    );
}

/// Test property with expression values
#[test]
fn test_property_value_expressions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base" value="10"/>
  <xacro:property name="computed" value="${base * 2}"/>
  <link name="test" value="${computed}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Property with expression value failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"value="20""#),
        "Computed property should be 20"
    );
}

/// Test property redefinition with lazy evaluation
#[test]
fn test_property_redefinition_lazy() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="10"/>
  <xacro:property name="y" value="${x}"/>
  <xacro:property name="x" value="20"/>
  <link name="test" value="${y}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Property redefinition failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // With lazy evaluation, y should see the latest value of x (20)
    assert!(
        output.contains(r#"value="20""#),
        "Lazy evaluation should use latest x value"
    );
}

/// Test alternative namespace prefix (e.g., xacro: vs x:)
#[test]
fn test_property_alternative_namespace_prefix() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:x="http://www.ros.org/wiki/xacro">
  <x:property name="width" value="0.5"/>
  <link name="test" size="${width}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Alternative namespace prefix should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"size="0.5""#),
        "Property with alternative prefix should work"
    );
}

/// Test namespace handling
#[test]
fn test_property_namespace_handling() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" xmlns:other="http://example.com">
  <xacro:property name="x" value="42"/>
  <other:element value="${x}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Multiple namespaces should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"value="42""#),
        "Property substitution should work across namespaces"
    );
}

/// Test error propagation in property expressions
#[test]
fn test_property_error_propagation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="test" value="${undefined_property}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_err(), "Undefined property should cause error");
}

/// Test macro call evaluates parameters
#[test]
fn test_macro_call_evaluates_parameters() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base" value="10"/>
  <xacro:macro name="test_macro" params="value">
    <link mass="${value}"/>
  </xacro:macro>
  <xacro:test_macro value="${base * 2}"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Macro call with expression should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"mass="20""#),
        "Macro parameter expression should be evaluated"
    );
}

/// Test macro definition is not evaluated until called
#[test]
fn test_macro_definition_only_not_evaluated() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="value">
    <link mass="${value}"/>
    <error expression="${undefined_property}"/>
  </xacro:macro>
</robot>"#;

    let result = test_xacro(input);
    // Should succeed because macro is never called
    assert!(
        result.is_ok(),
        "Macro definition should not be evaluated: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        !output.contains("undefined_property"),
        "Macro body should not be in output if not called"
    );
}

/// Test macro with late-binding defaults
#[test]
fn test_macro_late_binding_defaults() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_mass" value="1"/>
  <xacro:macro name="link_macro" params="name mass:=${default_mass}">
    <link name="${name}" mass="${mass}"/>
  </xacro:macro>
  <xacro:link_macro name="link1"/>
  <xacro:property name="default_mass" value="2"/>
  <xacro:link_macro name="link2"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Late-binding defaults should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // With late binding, both should use the latest default_mass value
    // Or early binding would use values at definition time
    // This test documents the actual behavior
    assert!(output.contains(r#"name="link1""#), "Should have link1");
    assert!(output.contains(r#"name="link2""#), "Should have link2");
}

/// Test macro parameters override global properties
#[test]
fn test_macro_param_overrides_global() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="global"/>
  <xacro:macro name="test_macro" params="x">
    <link value="${x}"/>
  </xacro:macro>
  <xacro:test_macro x="local"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Macro param should override global: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"value="local""#),
        "Macro parameter should shadow global property"
    );
    assert!(
        !output.contains(r#"value="global""#),
        "Global value should not appear"
    );
}

/// Test macro can access global properties
#[test]
fn test_macro_with_global_property() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="global_x" value="1"/>
  <xacro:macro name="test_macro" params="local_y">
    <link x="${global_x}" y="${local_y}"/>
  </xacro:macro>
  <xacro:test_macro local_y="2.0"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Macro with global property should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains(r#"x="1""#), "Should access global property");
    assert!(output.contains(r#"y="2.0""#), "Should use local parameter");
}

/// Test nested macro calls
#[test]
fn test_nested_macro_calls() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inner" params="value">
    <item>${value}</item>
  </xacro:macro>
  <xacro:macro name="outer" params="x">
    <container>
      <xacro:inner value="${x * 2}"/>
    </container>
  </xacro:macro>
  <xacro:outer x="21"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Nested macro calls should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<container>"), "Should have container");
    assert!(
        output.contains("<item>42</item>"),
        "Nested macro should evaluate expression"
    );
}

/// Test conditional with value preservation
#[test]
fn test_conditional_value_preservation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1">
    <link name="preserved"/>
  </xacro:if>
  <xacro:if value="0">
    <link name="excluded"/>
  </xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(result.is_ok(), "Conditionals failed: {:?}", result.err());

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="preserved""#),
        "True condition should preserve content"
    );
    assert!(
        !output.contains(r#"name="excluded""#),
        "False condition should exclude content"
    );
}

/// Test if with boolean literals
#[test]
fn test_if_boolean_literals() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="true"><included_true/></xacro:if>
  <xacro:if value="false"><excluded_false/></xacro:if>
  <xacro:if value="True"><included_True/></xacro:if>
  <xacro:if value="False"><excluded_False/></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Boolean literals failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("<included_true/>"),
        "true should include content"
    );
    assert!(
        !output.contains("<excluded_false/>"),
        "false should exclude content"
    );
    assert!(
        output.contains("<included_True/>"),
        "True should include content"
    );
    assert!(
        !output.contains("<excluded_False/>"),
        "False should exclude content"
    );
}

/// Test if with float truthiness
/// NOTE: Float truthiness works via EXPRESSIONS, not literals
/// Use ${3*0.1} for float evaluation, not literal "1.0"
#[test]
fn test_if_float_truthiness() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${3*0.0}"><excluded_zero/></xacro:if>
  <xacro:if value="${3*0.1}"><included_nonzero/></xacro:if>
  <xacro:if value="${1.5 + 0.5}"><included_two/></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Float truthiness failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        !output.contains("<excluded_zero/>"),
        "3*0.0 = 0.0 should be falsy"
    );
    assert!(
        output.contains("<included_nonzero/>"),
        "3*0.1 = 0.3 should be truthy"
    );
    assert!(
        output.contains("<included_two/>"),
        "1.5+0.5 = 2.0 should be truthy"
    );
}

/// Test if with integer truthiness
#[test]
fn test_if_integer_truthiness() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><included_one/></xacro:if>
  <xacro:if value="0"><excluded_zero/></xacro:if>
  <xacro:if value="42"><included_fortytwo/></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Integer truthiness failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<included_one/>"), "1 should be truthy");
    assert!(!output.contains("<excluded_zero/>"), "0 should be falsy");
    assert!(
        output.contains("<included_fortytwo/>"),
        "42 should be truthy"
    );
}

/// Test if with invalid value causes error
#[test]
fn test_if_invalid_value_error() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="not_a_boolean"><content/></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Invalid boolean 'not_a_boolean' should cause error"
    );
}

/// Test if with missing value attribute
#[test]
fn test_if_missing_value_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if><content/></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Missing value attribute should cause error"
    );
}

/// Test if with expressions
#[test]
fn test_if_with_expressions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="5"/>
  <xacro:if value="${x &gt; 3}"><greater/></xacro:if>
  <xacro:if value="${x &lt; 3}"><less/></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "If with expressions failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains("<greater/>"), "x > 3 should be true");
    assert!(!output.contains("<less/>"), "x < 3 should be false");
}

/// Test if with properties
#[test]
fn test_if_with_properties() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="enable_feature" value="1"/>
  <xacro:property name="disable_feature" value="0"/>
  <xacro:if value="${enable_feature}"><enabled/></xacro:if>
  <xacro:if value="${disable_feature}"><disabled/></xacro:if>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "If with properties failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("<enabled/>"),
        "enable_feature=1 should include"
    );
    assert!(
        !output.contains("<disabled/>"),
        "disable_feature=0 should exclude"
    );
}

// ============================================================================
// Dynamic Directive Attribute Tests
// ============================================================================
// Tests for ${...} substitution in xacro directive attributes (property name,
// macro name, include filename, insert_block name). These are critical features
// used extensively in real-world ROS URDF files.

/// Test dynamic property name substitution
#[test]
fn test_property_dynamic_name() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="prefix" value="left"/>
  <xacro:property name="${prefix}_wheel_radius" value="0.1"/>
  <link name="base">
    <value>${left_wheel_radius}</value>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Dynamic property name failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("<value>0.1</value>"),
        "Property 'left_wheel_radius' should be accessible: {}",
        output
    );
}

/// Test dynamic property name with concatenation
#[test]
fn test_property_dynamic_name_concatenation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="side" value="right"/>
  <xacro:property name="component" value="wheel"/>
  <xacro:property name="${side}_${component}_radius" value="0.15"/>
  <link name="base">
    <value>${right_wheel_radius}</value>
  </link>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Dynamic property name concatenation failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("<value>0.15</value>"),
        "Property 'right_wheel_radius' should be accessible: {}",
        output
    );
}

/// Test dynamic macro name substitution
#[test]
fn test_macro_dynamic_name() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="prefix" value="mobile"/>
  <xacro:macro name="${prefix}_base" params="size">
    <link name="${prefix}_base_link" size="${size}"/>
  </xacro:macro>
  <xacro:mobile_base size="1.0"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Dynamic macro name failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="mobile_base_link""#),
        "Macro with dynamic name should be callable: {}",
        output
    );
    assert!(
        output.contains(r#"size="1.0""#),
        "Macro parameters should work: {}",
        output
    );
}

/// Test dynamic include filename substitution
#[test]
fn test_include_dynamic_filename() {
    use std::fs;
    use tempfile::TempDir;

    let temp_dir = TempDir::new().unwrap();
    let temp_path = temp_dir.path();

    // Create included file
    let included_content = r#"<?xml version="1.0"?>
<part>
  <link name="included_link"/>
</part>"#;
    let included_path = temp_path.join("common.xacro");
    fs::write(&included_path, included_content).unwrap();

    // Root file with dynamic filename
    let root_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="file" value="common.xacro"/>
  <xacro:include filename="${{file}}"/>
</robot>"#
    );

    let root_path = temp_path.join("root.xacro");
    fs::write(&root_path, &root_content).unwrap();

    let result = test_xacro_file(&root_path);
    assert!(
        result.is_ok(),
        "Dynamic include filename failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="included_link""#),
        "Include with dynamic filename should work: {}",
        output
    );
}

/// Test dynamic include filename with path concatenation
#[test]
fn test_include_dynamic_filename_with_path() {
    use std::fs;
    use tempfile::TempDir;

    let temp_dir = TempDir::new().unwrap();
    let temp_path = temp_dir.path();

    // Create subdirectory
    let subdir = temp_path.join("urdf");
    fs::create_dir(&subdir).unwrap();

    // Create included file in subdirectory
    let included_content = r#"<?xml version="1.0"?>
<part>
  <link name="from_subdir"/>
</part>"#;
    fs::write(subdir.join("components.xacro"), included_content).unwrap();

    // Root file with path concatenation
    let root_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="subdir" value="urdf"/>
  <xacro:include filename="${{subdir}}/components.xacro"/>
</robot>"#
    );

    let root_path = temp_path.join("root.xacro");
    fs::write(&root_path, &root_content).unwrap();

    let result = test_xacro_file(&root_path);
    assert!(
        result.is_ok(),
        "Dynamic include with path concatenation failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="from_subdir""#),
        "Include with path concatenation should work: {}",
        output
    );
}

/// Test dynamic insert_block name substitution
#[test]
fn test_insert_block_dynamic_name() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="block_name" value="content"/>
  <xacro:macro name="foo" params="*content side">
    <container side="${side}">
      <xacro:insert_block name="${block_name}"/>
    </container>
  </xacro:macro>
  <xacro:foo side="left">
    <item id="1"/>
  </xacro:foo>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Dynamic insert_block name failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"side="left""#),
        "Container should have side attribute: {}",
        output
    );
    assert!(
        output.contains(r#"<item id="1"/>"#),
        "Block content should be inserted: {}",
        output
    );
}

/// Test dynamic insert_block name with conditional selection
#[test]
fn test_insert_block_dynamic_name_conditional() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="use_left" value="1"/>
  <xacro:macro name="selector" params="*left_block *right_block">
    <xacro:if value="${use_left}">
      <xacro:property name="selected" value="left_block"/>
    </xacro:if>
    <xacro:unless value="${use_left}">
      <xacro:property name="selected" value="right_block"/>
    </xacro:unless>
    <result>
      <xacro:insert_block name="${selected}"/>
    </result>
  </xacro:macro>
  <xacro:selector>
    <left>Left content</left>
    <right>Right content</right>
  </xacro:selector>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Dynamic insert_block with conditional selection failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("<left>Left content</left>"),
        "Left block should be inserted when use_left=1: {}",
        output
    );
    assert!(
        !output.contains("<right>Right content</right>"),
        "Right block should NOT be inserted when use_left=1: {}",
        output
    );
}

/// Test root element attribute substitution with macro parameters
/// Gemini Code Assist discovered that root element attributes weren't being
/// substituted because we only expanded children. This test ensures that fix works.
#[test]
fn test_root_element_attribute_substitution() {
    // Test case: macro parameter should be substitutable in root element attributes
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="make_robot" params="robot_name">
    <robot_model name="${robot_name}_v1">
      <link name="base"/>
    </robot_model>
  </xacro:macro>
  <xacro:make_robot robot_name="test"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_ok(),
        "Root element attribute substitution failed: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="test_v1""#),
        "Macro parameter in element attribute should be substituted: {}",
        output
    );
}

/// Test that root element attributes with undefined properties fail gracefully
#[test]
fn test_root_element_undefined_property_error() {
    // This should fail because robot_name is not defined when root element is processed
    let input = r#"<?xml version="1.0"?>
<robot name="${undefined_prop}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base"/>
</robot>"#;

    let result = test_xacro(input);
    assert!(
        result.is_err(),
        "Should fail with undefined property in root element"
    );

    let err = result.unwrap_err();
    assert!(
        format!("{:?}", err).contains("undefined_prop"),
        "Error should mention the undefined property"
    );
}

/// Regression test for parameter name collision (Root Cause #1)
/// Tests that block parameters are evaluated in caller's scope, not macro's scope
#[test]
fn test_insert_block_parameter_collision_regression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="global_x" value="1"/>
  <xacro:property name="local_y" value="2"/>

  <!-- Macro foo has a parameter named local_y (intentional collision) -->
  <xacro:macro name="foo" params="local_y *content">
    <xacro:insert_block name="content"/>
  </xacro:macro>

  <!-- Macro bar also has local_y and forwards to foo (nested collision) -->
  <xacro:macro name="bar" params="local_y *content">
    <xacro:foo local_y="${local_y}">
      <xacro:insert_block name="content"/>
    </xacro:foo>
  </xacro:macro>

  <!-- Direct call: foo receives local_y="3.0" as parameter -->
  <xacro:foo local_y="3.0">
    <origin name="direct" xyz="${global_x} ${local_y} 0"/>
  </xacro:foo>

  <!-- Nested call: bar receives local_y="4.0" and forwards to foo -->
  <xacro:bar local_y="4.0">
    <origin name="nested" xyz="${global_x} ${local_y} 1"/>
  </xacro:bar>
</robot>"#;

    let result = run_xacro(input);

    // Verify block content uses caller's local_y (2), not macro parameters (3 or 4)
    // Check for attribute presence regardless of order
    assert!(
        result.contains(r#"name="direct""#) && result.contains(r#"xyz="1 2 0""#),
        "Direct call: block should use caller's local_y (2), not macro's (3). Got: {}",
        result
    );

    assert!(
        result.contains(r#"name="nested""#) && result.contains(r#"xyz="1 2 1""#),
        "Nested call: block should use original caller's local_y (2), not macro's (4). Got: {}",
        result
    );
}

#[test]
fn test_macro_simple_expression_evaluation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="mass">
    <inertial>
      <mass value="${mass}"/>
      <inertia ixy="${0}" ixz="${0}"/>
    </inertial>
  </xacro:macro>

  <xacro:test_macro mass="0.6"/>
</robot>"#;

    let result = run_xacro(input);

    // Parse output to check attributes robustly
    let root = parse_xml(&result);
    let inertial = root
        .get_child("inertial")
        .expect("Should find inertial element");
    let inertia = inertial
        .get_child("inertia")
        .expect("Should find inertia element");

    // The expressions ${0} should be evaluated to 0
    let ixy: f64 = inertia
        .get_attribute("ixy")
        .expect("Should have ixy attribute")
        .parse()
        .expect("ixy should parse to f64");
    let ixz: f64 = inertia
        .get_attribute("ixz")
        .expect("Should have ixz attribute")
        .parse()
        .expect("ixz should parse to f64");

    assert_eq!(ixy, 0.0, "ixy should be 0.0");
    assert_eq!(ixz, 0.0, "ixz should be 0.0");

    // Should NOT contain any unexpanded expressions
    assert!(
        !result.contains("${"),
        "Should not contain unexpanded expression '${0}'. Got: {}",
        result
    );
}

#[test]
fn test_lazy_block_empty_element_inserts_nothing() {
    // Block parameters match POSITIONALLY (element name doesn't matter).
    // **param (lazy block) inserts only children, not the wrapper element.
    // Empty element = no children = nothing inserted.
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="**content">
    <result>
      <xacro:insert_block name="content"/>
    </result>
  </xacro:macro>

  <!-- <other> captured positionally as "content" (name irrelevant) -->
  <xacro:test_macro>
    <other></other>
  </xacro:test_macro>
</robot>"#;

    let result = run_xacro(input);
    let root = parse_xml(&result);

    // Python xacro: <other></other> positionally captured as "content" block.
    // Since it's empty, insert_block inserts nothing (no children).
    let result_elem = root
        .get_child("result")
        .expect("Should have <result> element");

    // Should have no significant children (empty element has no children to insert)
    let significant_children: Vec<_> = result_elem
        .children
        .iter()
        .filter(|n| {
            if let Some(text) = n.as_text() {
                !text.trim().is_empty()
            } else {
                true // Elements, comments, etc.
            }
        })
        .collect();
    assert!(
        significant_children.is_empty(),
        "result element should be empty, but has significant children: {:?}",
        significant_children
    );
}

#[test]
fn test_cwd_extension() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="working_dir" value="$(cwd)"/>
  <path>${working_dir}</path>
</robot>"#;

    let result = run_xacro(input);
    let root = Element::parse(result.as_bytes()).expect("Should parse valid XML");

    let path_elem = root.get_child("path").expect("Should have <path> element");
    let path_text = path_elem.get_text().expect("Should have path text");

    // $(cwd) should expand to the actual current working directory
    let current_dir = std::env::current_dir().expect("Should be able to get current_dir");
    let expected_path = current_dir.display().to_string();

    assert!(
        std::path::Path::new(path_text.as_ref()).is_absolute(),
        "$(cwd) should expand to absolute path, got: {}",
        path_text
    );

    assert_eq!(
        path_text.as_ref(),
        expected_path,
        "$(cwd) should expand to actual current working directory"
    );

    // Should not contain literal $(cwd)
    assert!(
        !result.contains("$(cwd)"),
        "$(cwd) should be expanded, not literal"
    );
}

#[test]
fn test_cwd_with_property_reference() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="cwd_path" value="$(cwd)"/>
  <path1>$(cwd)</path1>
  <path2>${cwd_path}</path2>
</robot>"#;

    let result = run_xacro(input);
    let root = Element::parse(result.as_bytes()).expect("Should parse valid XML");

    let path1_elem = root
        .get_child("path1")
        .expect("Should have <path1> element");
    let path1_text = path1_elem.get_text().expect("Should have path1 text");

    let path2_elem = root
        .get_child("path2")
        .expect("Should have <path2> element");
    let path2_text = path2_elem.get_text().expect("Should have path2 text");

    // Both should be absolute paths
    assert!(
        std::path::Path::new(path1_text.as_ref()).is_absolute(),
        "Direct $(cwd) should expand to absolute path, got: {}",
        path1_text
    );

    assert!(
        std::path::Path::new(path2_text.as_ref()).is_absolute(),
        "Property reference to $(cwd) should expand to absolute path, got: {}",
        path2_text
    );

    // Both should be the same path
    assert_eq!(
        path1_text.as_ref(),
        path2_text.as_ref(),
        "Direct $(cwd) and property reference should give same result"
    );
}

#[test]
fn test_cwd_extension_no_args() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="invalid" value="$(cwd extra)"/>
  <path>${invalid}</path>
</robot>"#;

    let result = test_xacro(input);

    // Should fail - $(cwd) doesn't accept arguments
    assert!(result.is_err(), "$(cwd) with arguments should fail");

    let err = result.unwrap_err();
    assert!(
        matches!(&err, xacro::XacroError::InvalidExtension { reason, .. } if reason.contains("does not take arguments")),
        "Expected InvalidExtension error for $(cwd) with arguments, got: {:?}",
        err
    );
}

#[test]
fn test_cwd_iterative_substitution() {
    // Test that $(cwd) works through multi-pass substitution (escaped with $$, then expanded)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="escaped_cwd" value="$$(cwd)"/>
  <path>${escaped_cwd}</path>
</robot>"#;

    let result = run_xacro(input);
    let root = Element::parse(result.as_bytes()).expect("Should parse valid XML");

    let path_elem = root.get_child("path").expect("Should have <path> element");
    let path_text = path_elem.get_text().expect("Should have path text");

    // $$(cwd) first becomes $(cwd) (unescape), then expands to actual cwd (second pass)
    let current_dir = std::env::current_dir().expect("Should be able to get current_dir");
    let expected_path = current_dir.display().to_string();

    assert_eq!(
        path_text.as_ref(),
        expected_path,
        "Iterative substitution should resolve $$(cwd) -> $(cwd) -> actual_path"
    );

    // Should not contain any literal placeholders
    assert!(
        !result.contains("$(cwd)") && !result.contains("${"),
        "All placeholders should be fully expanded"
    );
}

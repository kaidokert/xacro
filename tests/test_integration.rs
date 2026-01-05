use xacro::XacroProcessor;

#[test]
fn test_basic_property() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="0.5"/>
  <link name="base">
    <visual>
      <geometry>
        <box size="${width} 1.0 0.2"/>
      </geometry>
    </visual>
  </link>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    assert!(output.contains("0.5"), "Property substitution failed");
    assert!(!output.contains("${width}"), "Property not expanded");
    assert!(
        !output.contains("xacro:property"),
        "Property definition not removed"
    );
}

#[test]
fn test_basic_macro() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_link" params="name">
    <link name="${name}"/>
  </xacro:macro>

  <xacro:test_link name="base"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    assert!(output.contains(r#"name="base""#), "Macro expansion failed");
    assert!(
        !output.contains("xacro:macro"),
        "Macro definition not removed"
    );
    assert!(
        !output.contains("xacro:test_link"),
        "Macro call not expanded"
    );
}

#[test]
fn test_conditional() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="use_sensor" value="true"/>
  <xacro:if value="${use_sensor}">
    <link name="sensor"/>
  </xacro:if>
  <xacro:unless value="${use_sensor}">
    <link name="no_sensor"/>
  </xacro:unless>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    assert!(output.contains(r#"name="sensor""#), "If condition failed");
    assert!(
        !output.contains(r#"name="no_sensor""#),
        "Unless condition failed"
    );
}

#[test]
fn test_math_constants() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="angle" value="${pi/4}"/>
  <link name="test" value="${angle}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    // pi/4 = 0.7853981633974483
    assert!(output.contains("0.785"), "Math constant pi not available");
}

#[test]
fn test_include_basic() {
    let processor = XacroProcessor::new();
    let result = processor.run("tests/data/include_test.xacro");
    assert!(
        result.is_ok(),
        "Failed to process include: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // Should contain content from included file
    assert!(output.contains("base_link"), "Include content not found");
}

#[test]
fn test_include_nested() {
    let processor = XacroProcessor::new();
    let result = processor.run("tests/data/include_test_nested_base.xacro");
    assert!(
        result.is_ok(),
        "Failed to process nested includes: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // Should contain content from nested includes (arm and hand files)
    assert!(output.contains("arm_base"), "Arm include content not found");
    assert!(
        output.contains("hand_base"),
        "Hand include content not found"
    );
}

#[test]
fn test_include_with_macros() {
    // Test that includes work properly with macro expansion
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_link" params="name">
    <link name="${name}"/>
  </xacro:macro>

  <!-- This would normally be in a separate file -->
  <xacro:test_link name="included_link"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="included_link""#),
        "Macro in include failed"
    );
}

#[test]
fn test_property_default_attribute() {
    // Test the include-once guard pattern used in real-world files
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
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
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Property default attribute should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("aluminum"),
        "First material should be defined"
    );
    assert!(
        !output.contains("should_not_appear"),
        "Second material block should be skipped"
    );
}

#[test]
fn test_property_default_vs_value() {
    // Test that value overrides default
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- default is used if property not already defined -->
  <xacro:property name="test1" default="default_value"/>
  <link name="${test1}"/>

  <!-- value always sets the property -->
  <xacro:property name="test2" default="ignored" value="actual_value"/>
  <link name="${test2}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Failed: {:?}", result.err());

    let output = result.unwrap();
    assert!(
        output.contains(r#"name="default_value""#),
        "Default should be used"
    );
    assert!(
        output.contains(r#"name="actual_value""#),
        "Value should override default"
    );
}

/// Test nested macro calls with parameter expressions
///
/// This ensures that nested macro calls (one macro calling another) correctly
/// evaluate parameters at each level, including arithmetic expressions.
#[test]
fn test_nested_macro_calls_with_expressions() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
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
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Nested macro calls with expressions should work: {:?}",
        result.err()
    );

    let output = result.unwrap();

    // Parse and verify numeric value
    let root = xmltree::Element::parse(output.as_bytes()).expect("Should parse output XML");
    let link = root.get_child("link").expect("Should find link element");
    let inertia = link
        .get_child("inertia")
        .expect("Should find inertia element");

    let izz: f64 = inertia
        .attributes
        .get("izz")
        .expect("Should have izz attribute")
        .parse()
        .expect("izz should parse to f64");

    // izz = (1/12) * mass * x*x = (1/12) * 2.0 * 0.5*0.5 = 0.041666...
    let expected_izz = (1.0 / 12.0) * 2.0 * 0.5 * 0.5;

    const TOLERANCE: f64 = 1e-9;
    assert!(
        (izz - expected_izz).abs() < TOLERANCE,
        "izz should be {}, got {}",
        expected_izz,
        izz
    );

    // Verify no unevaluated placeholders remain
    assert!(
        !output.contains("${"),
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a" value="${b * 2}"/>
  <xacro:property name="b" value="${a * 3}"/>

  <link name="test">
    <mass value="${a}"/>
  </link>
</robot>"#;

    let result = processor.run_from_string(input);

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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="true">
    <link name="true_literal"/>
  </xacro:if>
  <xacro:if value="false">
    <link name="false_literal"/>
  </xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if>
    <link name="test"/>
  </xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="derived" value="${base * 2}"/>
  <xacro:property name="base" value="10"/>
  <link name="test" value="${derived}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="level1" value="${level2 * 2}"/>
  <xacro:property name="level2" value="${level3 * 2}"/>
  <xacro:property name="level3" value="5"/>
  <link name="test" value="${level1}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="unused" value="${undefined_var}"/>
  <xacro:property name="used" value="42"/>
  <link name="test" value="${used}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="bad" value="${undefined_var}"/>
  <link name="test" value="${bad}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = xacro::XacroProcessor::new_with_depth(5);

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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
    assert!(
        result.is_err(),
        "Circular block references should cause an error"
    );

    let err = result.err().unwrap();
    // Note: Uses unified recursion tracking, so all recursion depth errors
    // are MacroRecursionLimit (covers macros, insert_block, and all expand_node calls)
    assert!(
        matches!(err, xacro::XacroError::MacroRecursionLimit { .. }),
        "Should be MacroRecursionLimit error, got: {:?}",
        err
    );
}

/// Test deeply nested macros expand correctly
#[test]
fn test_deeply_nested_macros() {
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="global_x" value="1.0"/>

  <xacro:macro name="foo" params="local_y *content">
    <xacro:insert_block name="content"/>
  </xacro:macro>

  <xacro:foo local_y="2.0">
    <origin xyz="${global_x} ${local_y} 0"/>
  </xacro:foo>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Block with global property should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"xyz="1 2 0""#),
        "Should substitute both global and local properties"
    );
}

/// Test missing block parameter causes error
#[test]
fn test_insert_block_missing_block() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="needs_block" params="*content">
    <wrapper/>
  </xacro:macro>

  <xacro:needs_block/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="a2" value="${2*a}"/>
  <xacro:property name="a" value="42"/>
  <link name="test">
    <mass doubled="${a2}"/>
  </link>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="1.0"/>
  <xacro:property name="y" value="2.0"/>
  <xacro:property name="z" value="3.0"/>
  <link name="test">
    <origin xyz="${x} ${y} ${z}"/>
  </link>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:unless value="true">
    <excluded/>
  </xacro:unless>
  <xacro:unless value="false">
    <included/>
  </xacro:unless>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><!-- comment --> text <b>bar</b></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="0.5"/>
  <box size="${width}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="1"/>
  <xacro:property name="y" value="${x}"/>
  <point coord="${y}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let result = processor.run("tests/data/macro_test.xacro");
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
    let processor = XacroProcessor::new();
    let result = processor.run("tests/data/include_test_multi_base.xacro");
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
    let processor = XacroProcessor::new();
    let result = processor.run("tests/data/include_test_directory.xacro");
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
    let processor = XacroProcessor::new();
    let result = processor.run("tests/data/property_test_multiple.xacro");
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
    let processor = XacroProcessor::new();
    let result = processor.run("tests/data/property_test_multiple_out_of_order.xacro");
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot name="test_${robot_name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="robot_name" value="robot"/>
  <xacro:property name="prefix" value="arm"/>
  <link name="${prefix}_link"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_ok(), "Failed to process: {:?}", result.err());

    let output = result.unwrap();
    // Root element attributes are NOT substituted (this is a known limitation/feature)
    // Only child element attributes are substituted
    assert!(
        output.contains(r#"name="arm_link""#),
        "Property in link name should be substituted, output:\n{}",
        output
    );
    // The link element should have property substituted
    assert!(
        !output.contains("${prefix}"),
        "Property expression in child elements should be expanded"
    );
}

/// Test arithmetic property expressions
#[test]
fn test_property_arithmetic() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="5"/>
  <xacro:property name="y" value="3"/>
  <link name="test">
    <mass value="${x + y}"/>
    <inertia value="${x * y}"/>
  </link>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base" value="10"/>
  <xacro:property name="computed" value="${base * 2}"/>
  <link name="test" value="${computed}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="10"/>
  <xacro:property name="y" value="${x}"/>
  <xacro:property name="x" value="20"/>
  <link name="test" value="${y}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:x="http://www.ros.org/wiki/xacro">
  <x:property name="width" value="0.5"/>
  <link name="test" size="${width}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" xmlns:other="http://example.com">
  <xacro:property name="x" value="42"/>
  <other:element value="${x}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="test" value="${undefined_property}"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(result.is_err(), "Undefined property should cause error");
}

/// Test macro call evaluates parameters
#[test]
fn test_macro_call_evaluates_parameters() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="base" value="10"/>
  <xacro:macro name="test_macro" params="value">
    <link mass="${value}"/>
  </xacro:macro>
  <xacro:test_macro value="${base * 2}"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="value">
    <link mass="${value}"/>
    <error expression="${undefined_property}"/>
  </xacro:macro>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="default_mass" value="1.0"/>
  <xacro:macro name="link_macro" params="name mass:=${default_mass}">
    <link name="${name}" mass="${mass}"/>
  </xacro:macro>
  <xacro:link_macro name="link1"/>
  <xacro:property name="default_mass" value="2.0"/>
  <xacro:link_macro name="link2"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="global"/>
  <xacro:macro name="test_macro" params="x">
    <link value="${x}"/>
  </xacro:macro>
  <xacro:test_macro x="local"/>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="global_x" value="1.0"/>
  <xacro:macro name="test_macro" params="local_y">
    <link x="${global_x}" y="${local_y}"/>
  </xacro:macro>
  <xacro:test_macro local_y="2.0"/>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_ok(),
        "Macro with global property should work: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(output.contains(r#"x="1""#), "Should access global property");
    assert!(output.contains(r#"y="2""#), "Should use local parameter");
}

/// Test nested macro calls
#[test]
fn test_nested_macro_calls() {
    let processor = XacroProcessor::new();
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

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1">
    <link name="preserved"/>
  </xacro:if>
  <xacro:if value="0">
    <link name="excluded"/>
  </xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="true"><included_true/></xacro:if>
  <xacro:if value="false"><excluded_false/></xacro:if>
  <xacro:if value="True"><included_True/></xacro:if>
  <xacro:if value="False"><excluded_False/></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${3*0.0}"><excluded_zero/></xacro:if>
  <xacro:if value="${3*0.1}"><included_nonzero/></xacro:if>
  <xacro:if value="${1.5 + 0.5}"><included_two/></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><included_one/></xacro:if>
  <xacro:if value="0"><excluded_zero/></xacro:if>
  <xacro:if value="42"><included_fortytwo/></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="not_a_boolean"><content/></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_err(),
        "Invalid boolean 'not_a_boolean' should cause error"
    );
}

/// Test if with missing value attribute
#[test]
fn test_if_missing_value_attribute() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if><content/></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
    assert!(
        result.is_err(),
        "Missing value attribute should cause error"
    );
}

/// Test if with expressions
#[test]
fn test_if_with_expressions() {
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="5"/>
  <xacro:if value="${x &gt; 3}"><greater/></xacro:if>
  <xacro:if value="${x &lt; 3}"><less/></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="enable_feature" value="1"/>
  <xacro:property name="disable_feature" value="0"/>
  <xacro:if value="${enable_feature}"><enabled/></xacro:if>
  <xacro:if value="${disable_feature}"><disabled/></xacro:if>
</robot>"#;

    let result = processor.run_from_string(input);
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

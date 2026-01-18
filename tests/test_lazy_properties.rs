// Tests for lazy properties (body-based property definitions)
//
// Lazy properties store XML content and expand it at insertion time,
// with expressions evaluated using the insertion scope.

mod common;
use crate::common::*;

/// Helper to normalize XML for comparison (remove extra whitespace)
fn normalize(xml: &str) -> String {
    xml.lines()
        .map(|line| line.trim())
        .filter(|line| !line.is_empty())
        .collect::<Vec<_>>()
        .join("")
}

// ============================================================================
// Test 1: Basic lazy property insertion
// ============================================================================

#[test]
fn test_lazy_property_basic() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="block1">
    <link name="test_link"/>
  </xacro:property>
  <xacro:insert_block name="block1"/>
</robot>"#;

    let output = run_xacro(input);
    let normalized = normalize(&output);

    assert_xacro_contains!(
        normalized,
        r#"<link name="test_link""#,
        "Should insert the link element from lazy property"
    );
    assert_xacro_not_contains!(
        normalized,
        "xacro:property",
        "Should not contain xacro:property directive"
    );
}

// ============================================================================
// Test 2: Insertion time evaluation (NOT definition time)
// ============================================================================

#[test]
fn test_lazy_property_insertion_time_eval() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="x" value="5"/>
  <xacro:property name="block1">
    <link name="link_${x}"/>
  </xacro:property>
  <xacro:property name="x" value="10"/>
  <xacro:insert_block name="block1"/>
</robot>"#;

    let output = run_xacro(input);
    let normalized = normalize(&output);

    assert_xacro_contains!(
        normalized,
        r#"<link name="link_10""#,
        "Should use x=10 (insertion time value), not x=5 (definition time value)"
    );
    assert_xacro_not_contains!(normalized, "link_5", "Should NOT use definition-time value");
}

// ============================================================================
// Test 3: Insertion scope (uses macro parameters)
// ============================================================================

#[test]
fn test_lazy_property_insertion_scope() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="block1">
    <link name="link_${x}"/>
  </xacro:property>

  <xacro:macro name="use_block" params="x">
    <xacro:insert_block name="block1"/>
  </xacro:macro>

  <xacro:use_block x="5"/>
</robot>"#;

    let output = run_xacro(input);
    let normalized = normalize(&output);

    assert_xacro_contains!(
        normalized,
        r#"<link name="link_5""#,
        "Should use macro parameter x=5 (insertion scope)"
    );
}

// ============================================================================
// Test 4: Nested directives are expanded
// ============================================================================

#[test]
fn test_lazy_property_nested_directives() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="flag" value="1"/>
  <xacro:property name="block1">
    <xacro:if value="${flag}">
      <link name="conditional_link"/>
    </xacro:if>
  </xacro:property>
  <xacro:insert_block name="block1"/>
</robot>"#;

    let output = run_xacro(input);
    let normalized = normalize(&output);

    assert_xacro_contains!(
        normalized,
        r#"<link name="conditional_link""#,
        "Should expand nested xacro:if and include the link"
    );
    assert_xacro_not_contains!(
        normalized,
        "xacro:if",
        "Should NOT contain xacro:if directive in output"
    );
}

// ============================================================================
// Test 5: Property precedence over block parameters
// ============================================================================

#[test]
fn test_lazy_property_precedence_over_block_param() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="block1">
    <link name="from_property"/>
  </xacro:property>

  <xacro:macro name="test_macro" params="*block1">
    <xacro:insert_block name="block1"/>
  </xacro:macro>

  <xacro:test_macro>
    <link name="from_param"/>
  </xacro:test_macro>
</robot>"#;

    let output = run_xacro(input);
    let normalized = normalize(&output);

    assert_xacro_contains!(
        normalized,
        r#"<link name="from_property""#,
        "Should use property (properties have precedence)"
    );
    assert_xacro_not_contains!(normalized, "from_param", "Should NOT use block parameter");
}

// ============================================================================
// Test 6a: Empty properties are valid
// ============================================================================

#[test]
fn test_lazy_property_empty_valid() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="empty"></xacro:property>
  <test>
    <xacro:insert_block name="empty"/>
  </test>
</robot>"#;

    let output = run_xacro(input);
    let normalized = normalize(&output);

    assert!(
        normalized.contains("<test")
            && (normalized.contains("</test>") || normalized.contains("<test/>")),
        "Should have test element (possibly empty)"
    );
}

// ============================================================================
// Test 6c: Text-only properties are NOT created (matches Python xacro)
// ============================================================================
//
// This behavior was verified against Python xacro:
//   $ xacro test_text_only.xacro
//   error: name 'greeting' is not defined when evaluating expression 'greeting'
//
// Python xacro only creates body-based properties when the body contains
// structural nodes (elements, comments, CDATA, or PIs). Pure text content
// is NOT treated as a property value.

#[test]
fn test_lazy_property_text_only_not_created() {
    let result = test_xacro(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="greeting">Hello</xacro:property>
  <link name="${greeting}"/>
</robot>"#,
    );

    assert!(
        result.is_err(),
        "Text-only property should not be created, causing undefined property error (matches Python xacro)"
    );
}

// ============================================================================
// Test 7: Mixed content (text + elements) works
// ============================================================================

#[test]
fn test_lazy_property_mixed_content() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="mixed">
    Start
    <middle/>
    End
  </xacro:property>
  <xacro:insert_block name="mixed"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(output, "Start", "Should preserve text before element");
    assert_xacro_contains!(output, "<middle", "Should contain the middle element");
    assert_xacro_contains!(output, "End", "Should preserve text after element");
}

// ============================================================================
// Test 9: Namespace preservation
// ============================================================================

#[test]
fn test_lazy_property_namespace_preservation() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"
       xmlns:custom="http://example.com/custom" name="test">
  <xacro:property name="namespaced_block">
    <custom:element custom:attr="value"/>
  </xacro:property>
  <xacro:insert_block name="namespaced_block"/>
</robot>"#;

    let output = run_xacro(input);
    assert!(
        output.contains("custom:element") || output.contains("custom:"),
        "Should preserve custom namespace prefix"
    );
    // Note: Namespace declaration might be moved to root, which is valid XML
}

// ============================================================================
// Test 10: Comments are preserved
// ============================================================================

#[test]
fn test_lazy_property_comments_preserved() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="with_comment">
    <!-- Comment before -->
    <link name="test"/>
    <!-- Comment after -->
  </xacro:property>
  <xacro:insert_block name="with_comment"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(output, "<!-- Comment before -->");
    assert_xacro_contains!(output, "<!-- Comment after -->");
}

// ============================================================================
// Test 11: CDATA and Processing Instructions preserved
// ============================================================================

#[test]
fn test_lazy_property_cdata_and_pi_preserved() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="block1">
    <![CDATA[some <raw> content]]>
    <?proc instruction?>
    <link name="test_link"/>
  </xacro:property>
  <xacro:insert_block name="block1"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(
        output,
        "some <raw> content",
        "Should preserve CDATA content"
    );
    assert_xacro_contains!(
        output,
        "<?proc instruction?>",
        "Should preserve processing instruction"
    );
}

// ============================================================================
// Test 12: Comment-only properties are valid
// ============================================================================

#[test]
fn test_lazy_property_comment_only_valid() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="comment_only">
    <!-- this is a comment-only lazy property -->
  </xacro:property>

  <test>
    <xacro:insert_block name="comment_only"/>
  </test>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(output, "<!-- this is a comment-only lazy property -->");
}

// ============================================================================
// Test 13: Local properties also have precedence
// ============================================================================

#[test]
fn test_lazy_property_local_precedence() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:macro name="test_local" params="*local_block">
    <xacro:property name="local_block">
      <link name="from_local_property"/>
    </xacro:property>
    <xacro:insert_block name="local_block"/>
  </xacro:macro>

  <xacro:test_local>
    <link name="from_block_param"/>
  </xacro:test_local>
</robot>"#;

    let output = run_xacro(input);
    let normalized = normalize(&output);

    assert_xacro_contains!(
        normalized,
        r#"<link name="from_local_property""#,
        "Should use local property (local properties have precedence)"
    );
    assert_xacro_not_contains!(
        normalized,
        "from_block_param",
        "Should NOT use block parameter"
    );
}

// ============================================================================
// Test 14: Value properties don't interfere with block parameters
// ============================================================================

#[test]
fn test_value_property_does_not_shadow_block_param() {
    // Test that value properties (defined with value="...") don't interfere
    // with block parameters. Only LAZY properties (body-based) should be
    // accessible via insert_block.
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="outer" params="*content">
    <xacro:property name="content" value="I AM A PROPERTY, NOT A BLOCK"/>
    <xacro:macro name="inner">
      <xacro:insert_block name="content"/>
    </xacro:macro>
    <xacro:inner/>
  </xacro:macro>
  <xacro:outer>
    <foo/>
  </xacro:outer>
</robot>"#;

    let result = run_xacro(input);
    assert!(
        !result.contains("I AM A PROPERTY"),
        "Output should NOT contain the property value. Output: {}",
        result
    );
    assert!(
        result.contains("<foo"),
        "Output should contain <foo/> from parent block parameter. Output: {}",
        result
    );
}

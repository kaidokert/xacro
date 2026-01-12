//! Integration tests for processing instruction preservation
//!
//! These tests verify that xacro:
//! - Preserves processing instructions (PIs) outside root element
//! - Preserves comments outside root element
//! - Only processes xacro-namespaced elements

mod common;
use crate::common::*;

/// Category 1: Corpus Reality Tests - Actual patterns from corpus

/// Test 1.1: Single xml-model PI (132 files in corpus)
#[test]
fn test_xml_model_single_line() {
    let input = r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>test_package</name>
</package>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(
        output,
        r#"<?xml-model href="http://download.ros.org/schema/package_format3.xsd"#,
        "xml-model PI must be preserved"
    );
    assert_xacro_contains!(
        output,
        r#"<name>test_package</name>"#,
        "Content must be unchanged"
    );
}

/// Test 1.2: Multi-line xml-model PI (~5 files in corpus)
#[test]
fn test_xml_model_multi_line() {
    let input = r#"<?xml version="1.0"?>
<?xml-model
  href="http://download.ros.org/schema/package_format2.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>test</name>
</package>"#;

    let output = run_xacro(input);
    // PI should be preserved (may be reformatted to single line by xmltree)
    assert!(
        output.contains(r#"<?xml-model"#) && output.contains(r#"package_format2.xsd"#),
        "Multi-line xml-model PI must be preserved, got:\n{}",
        output
    );
}

/// Test 1.3: No PIs (majority of corpus - regression test)
#[test]
fn test_no_processing_instructions() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="x" value="1"/>
  <link name="link_${x}"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(
        output,
        r#"<link name="link_1"/>"#,
        "Should work exactly as before"
    );
    assert_xacro_not_contains!(output, "xacro:property", "Xacro should be processed");
}

/// Test 1.4: xml-model + xacro expansion
#[test]
fn test_xml_model_with_xacro_expansion() {
    let input = r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package xmlns:xacro="http://www.ros.org/wiki/xacro" format="3">
  <xacro:property name="pkg_name" value="test_pkg"/>
  <name>${pkg_name}</name>
</package>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(output, r#"<?xml-model"#, "PI should be preserved");
    assert_xacro_contains!(
        output,
        r#"<name>test_pkg</name>"#,
        "Xacro expansion should work"
    );
    assert_xacro_not_contains!(
        output,
        "xacro:property",
        "xacro:property should be processed"
    );
}

/// Test 1.5: Comments outside root (Python xacro preserves)
#[test]
fn test_comment_outside_root_preserved() {
    let input = r#"<?xml version="1.0"?>
<!-- User comment about this package -->
<package format="3">
  <name>test</name>
</package>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(
        output,
        "<!-- User comment about this package -->",
        "User comments outside root should be preserved"
    );
}

/// Category 2: Edge Cases - Robustness tests

/// Test 2.1: Multiple PIs (not in corpus, but valid XML)
#[test]
fn test_multiple_processing_instructions() {
    let input = r#"<?xml version="1.0"?>
<?xml-stylesheet type="text/css" href="style.css"?>
<?xml-model href="schema.xsd"?>
<robot name="test"/>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(
        output,
        r#"<?xml-stylesheet"#,
        "xml-stylesheet PI should be preserved"
    );
    assert_xacro_contains!(output, r#"<?xml-model"#, "xml-model PI should be preserved");

    // Verify order (stylesheet appears before xml-model)
    let stylesheet_pos = output
        .find("<?xml-stylesheet")
        .expect("xml-stylesheet PI should be preserved");
    let model_pos = output
        .find("<?xml-model")
        .expect("xml-model PI should be preserved");
    assert!(stylesheet_pos < model_pos, "PI order must be preserved");
}

/// Test 2.2: PI + comment mixed (order matters)
#[test]
fn test_pi_and_comment_ordering() {
    let input = r#"<?xml version="1.0"?>
<!-- First comment -->
<?xml-model href="schema.xsd"?>
<!-- Second comment -->
<robot name="test"/>"#;

    let output = run_xacro(input);

    // Find positions
    let comment1_pos = output
        .find("<!-- First comment -->")
        .expect("First comment should be preserved");
    let pi_pos = output.find("<?xml-model").expect("PI should be preserved");
    let comment2_pos = output
        .find("<!-- Second comment -->")
        .expect("Second comment should be preserved");

    // Verify order preserved
    assert!(comment1_pos < pi_pos, "First comment before PI");
    assert!(pi_pos < comment2_pos, "PI before second comment");
}

/// Test 2.3: Empty PI data (valid but unusual)
#[test]
fn test_processing_instruction_no_data() {
    let input = r#"<?xml version="1.0"?>
<?target?>
<robot name="test"/>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(output, r#"<?target?>"#);
}

/// Test 2.4: PI inside root element (should work, already handled by xmltree)
#[test]
fn test_processing_instruction_inside_root() {
    let input = r#"<?xml version="1.0"?>
<robot name="test">
  <?processing-inside data="yes"?>
  <link name="base"/>
</robot>"#;

    let output = run_xacro(input);
    assert_xacro_contains!(output, r#"<?processing-inside data="yes"?>"#);
}

/// Category 3: Serialization Tests - Verify output format

/// Test 3.1: XML declaration is present
#[test]
fn test_xml_declaration_present() {
    let input = r#"<?xml version="1.0"?>
<robot name="test"/>"#;

    let output = run_xacro(input);
    assert!(
        output.trim_start().starts_with(r#"<?xml version="1.0""#),
        "Output should start with XML declaration"
    );
}

/// Test 3.2: XML declaration is emitted even when missing from input
#[test]
fn test_xml_declaration_added_when_missing() {
    let input = r#"<robot name="test"/>"#;

    let output = run_xacro(input);
    assert!(
        output.trim_start().starts_with(r#"<?xml version="1.0""#),
        "Output should start with XML declaration even when input omits it"
    );
}

/// Test 3.3: Preamble appears after declaration, before root
#[test]
fn test_preamble_position() {
    let input = r#"<?xml version="1.0"?>
<?xml-model href="schema.xsd"?>
<robot name="test"/>"#;

    let output = run_xacro(input);

    // Find positions
    let decl_pos = output
        .find(r#"<?xml version"#)
        .expect("XML declaration should be present");
    let pi_pos = output.find(r#"<?xml-model"#).expect("PI should be present");
    let root_pos = output
        .find(r#"<robot"#)
        .expect("Root element should be present");

    // Verify order: declaration → PI → root
    assert!(decl_pos < pi_pos, "Declaration before PI");
    assert!(pi_pos < root_pos, "PI before root");
}

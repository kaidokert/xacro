/// Block parameter stress tests (* vs **)
///
/// These tests verify the critical distinction between:
/// - *param (regular block): inserts element itself (wrapper + children)
/// - **param (lazy block): inserts children only (strips wrapper)
use xacro::XacroProcessor;

fn normalize(s: &str) -> String {
    s.replace(char::is_whitespace, "")
}

#[test]
fn test_single_star_preserves_wrapper() {
    // *param should insert the element itself (wrapper + children)
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="single_star" params="*blk">
    <out><xacro:insert_block name="blk"/></out>
  </xacro:macro>

  <xacro:single_star>
    <wrapper>content</wrapper>
  </xacro:single_star>
</robot>"#;

    let result = processor.run_from_string(input).unwrap();
    let normalized = normalize(&result);

    // Should preserve the wrapper element
    assert!(
        normalized.contains("<out><wrapper>content</wrapper></out>"),
        "Expected wrapper preserved with *param, got: {}",
        result
    );
}

#[test]
fn test_double_star_strips_wrapper() {
    // **param should insert only children (strip wrapper)
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="double_star" params="**blk">
    <out><xacro:insert_block name="blk"/></out>
  </xacro:macro>

  <xacro:double_star>
    <wrapper>content</wrapper>
  </xacro:double_star>
</robot>"#;

    let result = processor.run_from_string(input).unwrap();
    let normalized = normalize(&result);

    // Should strip the wrapper, insert only "content" text node
    assert!(
        normalized.contains("<out>content</out>"),
        "Expected wrapper stripped with **param, got: {}",
        result
    );
    assert!(
        !normalized.contains("<wrapper>"),
        "Wrapper should be stripped with **param, got: {}",
        result
    );
}

#[test]
fn test_mixed_star_positional() {
    // Mix of *param and **param in same macro
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="mixed" params="*a **b">
    <res_a><xacro:insert_block name="a"/></res_a>
    <res_b><xacro:insert_block name="b"/></res_b>
  </xacro:macro>

  <xacro:mixed>
    <wrap_a>A</wrap_a>
    <wrap_b>B</wrap_b>
  </xacro:mixed>
</robot>"#;

    let result = processor.run_from_string(input).unwrap();
    let normalized = normalize(&result);

    // First param (*a) should preserve wrapper
    assert!(
        normalized.contains("<res_a><wrap_a>A</wrap_a></res_a>"),
        "Expected *a to preserve wrapper, got: {}",
        result
    );

    // Second param (**b) should strip wrapper
    assert!(
        normalized.contains("<res_b>B</res_b>"),
        "Expected **b to strip wrapper, got: {}",
        result
    );
    assert!(
        !normalized.contains("<res_b><wrap_b>"),
        "Expected **b to strip wrap_b, got: {}",
        result
    );
}

#[test]
fn test_empty_single_star() {
    // *param with empty element should preserve the empty element
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="empty_single" params="*blk">
    <out><xacro:insert_block name="blk"/></out>
  </xacro:macro>

  <xacro:empty_single>
    <void/>
  </xacro:empty_single>
</robot>"#;

    let result = processor.run_from_string(input).unwrap();
    let normalized = normalize(&result);

    // Should preserve the empty void element
    assert!(
        normalized.contains("<out><void/></out>"),
        "Expected empty element preserved with *param, got: {}",
        result
    );
}

#[test]
fn test_empty_double_star() {
    // **param with empty element should insert nothing (no children)
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="empty_double" params="**blk">
    <out><xacro:insert_block name="blk"/></out>
  </xacro:macro>

  <xacro:empty_double>
    <void/>
  </xacro:empty_double>
</robot>"#;

    let result = processor.run_from_string(input).unwrap();
    let normalized = normalize(&result);

    // Should strip void element, leaving out empty (no children to insert)
    assert!(
        normalized.contains("<out></out>") || normalized.contains("<out/>"),
        "Expected empty <out> with **param on empty element, got: {}",
        result
    );
    assert!(
        !normalized.contains("<void"),
        "Expected void element stripped with **param, got: {}",
        result
    );
}

#[test]
fn test_single_star_with_attributes() {
    // *param should preserve wrapper with attributes
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="*blk">
    <out><xacro:insert_block name="blk"/></out>
  </xacro:macro>

  <xacro:test>
    <wrapper attr="value" id="123">
      <inner>text</inner>
    </wrapper>
  </xacro:test>
</robot>"#;

    let result = processor.run_from_string(input).unwrap();
    let normalized = normalize(&result);

    // Should preserve wrapper with all attributes
    assert!(
        normalized.contains("<wrapper") && normalized.contains("attr=\"value\""),
        "Expected wrapper with attributes preserved, got: {}",
        result
    );
    assert!(
        normalized.contains("<inner>text</inner>"),
        "Expected children preserved, got: {}",
        result
    );
}

#[test]
fn test_double_star_strips_attributes() {
    // **param should strip wrapper and its attributes, keep only children
    let processor = XacroProcessor::new();
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="**blk">
    <out><xacro:insert_block name="blk"/></out>
  </xacro:macro>

  <xacro:test>
    <wrapper attr="value" id="123">
      <inner>text</inner>
    </wrapper>
  </xacro:test>
</robot>"#;

    let result = processor.run_from_string(input).unwrap();
    let normalized = normalize(&result);

    // Should strip wrapper and all its attributes
    assert!(
        !normalized.contains("<wrapper"),
        "Expected wrapper stripped with **param, got: {}",
        result
    );
    assert!(
        !normalized.contains("attr=\"value\""),
        "Expected attributes stripped with **param, got: {}",
        result
    );
    assert!(
        normalized.contains("<out><inner>text</inner></out>"),
        "Expected only children preserved, got: {}",
        result
    );
}

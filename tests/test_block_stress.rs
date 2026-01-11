/// Block parameter stress tests (* vs **)
///
/// These tests verify the critical distinction between:
/// - *param (regular block): inserts element itself (wrapper + children)
/// - **param (lazy block): inserts children only (strips wrapper)
use xacro::XacroProcessor;
use xmltree::Element;

fn parse_result(xml: &str) -> Element {
    Element::parse(xml.as_bytes()).expect("Should parse valid XML")
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
    let root = parse_result(&result);

    let out = root.get_child("out").expect("Should have <out> element");
    let wrapper = out
        .get_child("wrapper")
        .expect("Should have <wrapper> child (preserved by *param)");
    assert_eq!(
        wrapper.get_text().as_deref(),
        Some("content"),
        "Wrapper should contain 'content' text"
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
    let root = parse_result(&result);

    let out = root.get_child("out").expect("Should have <out> element");

    // Should NOT have wrapper element (stripped by **param)
    assert!(
        out.get_child("wrapper").is_none(),
        "Wrapper should be stripped with **param"
    );

    // Should have text node directly
    assert_eq!(
        out.get_text().as_deref(),
        Some("content"),
        "Should have text content directly (wrapper stripped)"
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
    let root = parse_result(&result);

    // First param (*a) should preserve wrapper
    let res_a = root.get_child("res_a").expect("Should have <res_a>");
    let wrap_a = res_a
        .get_child("wrap_a")
        .expect("Should have <wrap_a> preserved by *a");
    assert_eq!(wrap_a.get_text().as_deref(), Some("A"));

    // Second param (**b) should strip wrapper
    let res_b = root.get_child("res_b").expect("Should have <res_b>");
    assert!(
        res_b.get_child("wrap_b").is_none(),
        "Should NOT have <wrap_b> (stripped by **b)"
    );
    assert_eq!(
        res_b.get_text().as_deref(),
        Some("B"),
        "Should have text directly"
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
    let root = parse_result(&result);

    let out = root.get_child("out").expect("Should have <out>");
    let void = out
        .get_child("void")
        .expect("Should have <void/> preserved by *param");
    assert!(void.children.is_empty(), "<void> should be empty");
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
    let root = parse_result(&result);

    let out = root.get_child("out").expect("Should have <out>");

    // Should NOT have void element (stripped by **param)
    assert!(
        out.get_child("void").is_none(),
        "Should NOT have <void> (stripped by **param)"
    );

    // Should have no children (empty element had no children to insert)
    let significant_children: Vec<_> = out
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
        "<out> should be empty (no children to insert from empty element)"
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
    let root = parse_result(&result);

    let out = root.get_child("out").expect("Should have <out>");
    let wrapper = out
        .get_child("wrapper")
        .expect("Should have <wrapper> preserved by *param");

    // Check attributes preserved
    use xmltree::AttributeName;
    assert_eq!(
        wrapper
            .attributes
            .get(&AttributeName::local("attr"))
            .map(|s| s.as_str()),
        Some("value"),
        "attr attribute should be preserved"
    );
    assert_eq!(
        wrapper
            .attributes
            .get(&AttributeName::local("id"))
            .map(|s| s.as_str()),
        Some("123"),
        "id attribute should be preserved"
    );

    // Check children preserved
    let inner = wrapper
        .get_child("inner")
        .expect("Should have <inner> child");
    assert_eq!(inner.get_text().as_deref(), Some("text"));
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
    let root = parse_result(&result);

    let out = root.get_child("out").expect("Should have <out>");

    // Should NOT have wrapper (stripped by **param)
    assert!(
        out.get_child("wrapper").is_none(),
        "Wrapper should be stripped by **param"
    );

    // Should have inner element directly (children preserved)
    let inner = out
        .get_child("inner")
        .expect("Should have <inner> child directly (wrapper stripped)");
    assert_eq!(inner.get_text().as_deref(), Some("text"));
}

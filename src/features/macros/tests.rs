#[cfg(test)]
mod macro_tests {
    use crate::{features::macros::MacroProcessor, utils::print_diff, XacroError, XacroProcessor};
    use log::error;
    use std::collections::HashMap;
    use std::path::Path;

    /// Standard xacro namespace for tests
    const XACRO_NS: &str = "http://www.ros.org/wiki/xacro";

    #[test]
    fn test_macro_basic() -> Result<(), XacroError> {
        env_logger::try_init().ok();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let path = Path::new("tests/data/macro_test.xacro");
        let data = XacroProcessor::parse_file(path)?;
        let expected = XacroProcessor::parse_file("tests/data/macro_test_expected.xacro")?;

        // For this test, no global properties needed
        let global_properties = HashMap::new();
        let actual = macro_processor.process(data, &global_properties, XACRO_NS)?;

        let expected_str =
            XacroProcessor::serialize(&expected).expect("failed to serialize expected XML");
        let actual_str =
            XacroProcessor::serialize(&actual).expect("failed to serialize actual XML");

        if actual != expected {
            error!("\nXML Difference (actual vs expected):");
            print_diff(&actual_str, &expected_str);
            panic!("XML documents do not match expected output");
        }

        Ok(())
    }

    #[test]
    fn test_nested_macro_calls() {
        // Test Problem 1 fix: nested macros now expand
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inner" params="x">
    <item value="${x}"/>
  </xacro:macro>

  <xacro:macro name="outer" params="">
    <xacro:inner x="5"/>
  </xacro:macro>

  <xacro:outer/>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Check that nested macro was expanded
        assert_eq!(result.children.len(), 1);
        let item = result.children[0]
            .as_element()
            .expect("Expected 'item' element");
        assert_eq!(item.name, "item");
        assert_eq!(item.attributes.get("value"), Some(&"5".to_string()));
    }

    #[test]
    fn test_macro_with_global_property() {
        // Test Problem 2 fix: macros can access global properties
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="pi" value="3.14159"/>

  <xacro:macro name="circle" params="radius">
    <area value="${pi * radius * radius}"/>
  </xacro:macro>

  <xacro:circle radius="2"/>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();

        // Simulate global properties from PropertyProcessor
        let mut global_properties = HashMap::new();
        global_properties.insert("pi".to_string(), "3.14159".to_string());

        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Check that macro referenced global property
        // Note: xacro:property elements are still present (PropertyProcessor removes them)
        let area_elem = result
            .children
            .iter()
            .find_map(|child| {
                if let xmltree::XMLNode::Element(e) = child {
                    if e.name == "area" {
                        return Some(e);
                    }
                }
                None
            })
            .expect("Expected area element");

        let value = area_elem.attributes.get("value").unwrap();
        // pi * 2 * 2 = 12.56636
        assert!(value.starts_with("12.56"), "Expected ~12.56, got {}", value);
    }

    #[test]
    fn test_macro_param_overrides_global() {
        // Test scope precedence: macro param should override global
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="x" value="1"/>

  <xacro:macro name="foo" params="x">
    <item value="${x}"/>
  </xacro:macro>

  <xacro:foo x="2"/>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();

        // Global property x=1
        let mut global_properties = HashMap::new();
        global_properties.insert("x".to_string(), "1".to_string());

        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Macro param (x="2") should override global property (x="1")
        // Note: xacro:property elements are still present (PropertyProcessor removes them)
        let item_elem = result
            .children
            .iter()
            .find_map(|child| {
                if let xmltree::XMLNode::Element(e) = child {
                    if e.name == "item" {
                        return Some(e);
                    }
                }
                None
            })
            .expect("Expected item element");

        assert_eq!(item_elem.attributes.get("value"), Some(&"2".to_string()));
    }

    #[test]
    fn test_deeply_nested_macros() {
        // Test recursion works at multiple levels
        env_logger::try_init().ok();
        let input = r#"
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
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Test 3 levels of nesting all expanded correctly
        assert_eq!(result.children.len(), 1);
        let item = result.children[0]
            .as_element()
            .expect("Expected 'item' element");
        assert_eq!(item.name, "item");
        assert_eq!(item.attributes.get("value"), Some(&"42".to_string()));
    }

    // ========== insert_block tests ==========

    #[test]
    fn test_insert_block_basic() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="container" params="*content">
    <wrapper>
      <xacro:insert_block name="content"/>
    </wrapper>
  </xacro:macro>

  <xacro:container>
    <item id="1"/>
  </xacro:container>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Should have wrapper with item inside
        assert_eq!(result.children.len(), 1);

        let wrapper = result.children[0]
            .as_element()
            .expect("Expected 'wrapper' element");
        assert_eq!(wrapper.name, "wrapper");
        assert_eq!(wrapper.children.len(), 1);

        let item = wrapper.children[0]
            .as_element()
            .expect("Expected 'item' element");
        assert_eq!(item.name, "item");
        assert_eq!(item.attributes.get("id"), Some(&"1".to_string()));
    }

    #[test]
    fn test_insert_block_multiple_params() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="joint_with_origin" params="name type *origin">
    <joint name="${name}" type="${type}">
      <xacro:insert_block name="origin"/>
    </joint>
  </xacro:macro>

  <xacro:joint_with_origin name="elbow" type="revolute">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </xacro:joint_with_origin>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Should produce joint with origin
        assert_eq!(result.children.len(), 1);

        let joint = result.children[0]
            .as_element()
            .expect("Expected 'joint' element");
        assert_eq!(joint.name, "joint");
        assert_eq!(joint.attributes.get("name"), Some(&"elbow".to_string()));
        assert_eq!(joint.attributes.get("type"), Some(&"revolute".to_string()));
        assert_eq!(joint.children.len(), 1);

        let origin = joint.children[0]
            .as_element()
            .expect("Expected 'origin' element");
        assert_eq!(origin.name, "origin");
        assert_eq!(origin.attributes.get("xyz"), Some(&"0 0 0.3".to_string()));
    }

    #[test]
    fn test_insert_block_reuse() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="duplicate" params="*content">
    <first><xacro:insert_block name="content"/></first>
    <second><xacro:insert_block name="content"/></second>
  </xacro:macro>

  <xacro:duplicate>
    <item>Hello</item>
  </xacro:duplicate>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Should duplicate the block
        assert_eq!(result.children.len(), 2);

        let first = result.children[0]
            .as_element()
            .expect("Expected 'first' element");
        assert_eq!(first.name, "first");
        assert_eq!(first.children.len(), 1);

        let second = result.children[1]
            .as_element()
            .expect("Expected 'second' element");
        assert_eq!(second.name, "second");
        assert_eq!(second.children.len(), 1);
    }

    #[test]
    fn test_insert_block_with_expressions() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="*content">
    <xacro:insert_block name="content"/>
  </xacro:macro>

  <xacro:foo>
    <origin xyz="${x} 0 0"/>
  </xacro:foo>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let mut global_properties = HashMap::new();
        global_properties.insert("x".to_string(), "0.5".to_string());
        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // ${x} should be evaluated in the block
        assert_eq!(result.children.len(), 1);

        let origin = result.children[0]
            .as_element()
            .expect("Expected 'origin' element");
        assert_eq!(origin.name, "origin");
        assert_eq!(origin.attributes.get("xyz"), Some(&"0.5 0 0".to_string()));
    }

    #[test]
    fn test_insert_block_with_global_property() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="foo" params="local_y *content">
    <xacro:insert_block name="content"/>
  </xacro:macro>

  <xacro:foo local_y="2.0">
    <origin xyz="${global_x} ${local_y} 0"/>
  </xacro:foo>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let mut global_properties = HashMap::new();
        global_properties.insert("global_x".to_string(), "1.0".to_string());
        let result = macro_processor
            .process(xml, &global_properties, XACRO_NS)
            .unwrap();

        // Should produce origin with both global and local props substituted
        // Note: pyisheval evaluates 1.0 and 2.0 as numbers, which convert to "1" and "2" (not "1.0", "2.0")
        assert_eq!(result.children.len(), 1);

        let origin = result.children[0]
            .as_element()
            .expect("Expected 'origin' element");
        assert_eq!(origin.name, "origin");
        assert_eq!(origin.attributes.get("xyz"), Some(&"1 2 0".to_string()));
    }

    #[test]
    fn test_insert_block_missing_block() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="needs_block" params="*content">
    <wrapper/>
  </xacro:macro>

  <xacro:needs_block/>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor.process(xml, &global_properties, XACRO_NS);

        // Should error - missing block parameter
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::MissingBlockParameter { ref param, .. } if param == "content"
        ));
    }

    #[test]
    fn test_insert_block_extra_children() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="one_block" params="*content">
    <wrapper/>
  </xacro:macro>

  <xacro:one_block>
    <first/>
    <second/>
  </xacro:one_block>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor.process(xml, &global_properties, XACRO_NS);

        // Should error - too many children (provided 2, expected 1)
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::UnusedBlock { extra_count: 1, .. }
        ));
    }

    #[test]
    fn test_insert_block_undefined_name() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="bad_insert" params="*content">
    <xacro:insert_block name="wrong_name"/>
  </xacro:macro>

  <xacro:bad_insert>
    <item/>
  </xacro:bad_insert>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor.process(xml, &global_properties, XACRO_NS);

        // Should error - undefined block name
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::UndefinedBlock { ref name } if name == "wrong_name"
        ));
    }

    #[test]
    fn test_insert_block_with_default_param() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="invalid" params="*content:=default">
    <wrapper/>
  </xacro:macro>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor.process(xml, &global_properties, XACRO_NS);

        // Should error during macro collection - block params can't have defaults
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::BlockParameterWithDefault { ref param } if param.contains("content")
        ));
    }

    #[test]
    fn test_insert_block_nested_macros() {
        env_logger::try_init().ok();
        let input = r#"
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
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor
            .process(xml, &HashMap::new(), XACRO_NS)
            .unwrap();

        // The nested macro inside the block should be fully expanded
        assert_eq!(result.children.len(), 1);

        let container = result.children[0]
            .as_element()
            .expect("Expected 'container' element");
        assert_eq!(container.name, "container");
        assert_eq!(container.children.len(), 1);

        let item = container.children[0]
            .as_element()
            .expect("Expected 'item' element");
        assert_eq!(item.name, "item");
        assert_eq!(item.children.len(), 1);

        let text = item.children[0]
            .as_text()
            .expect("Expected text content in 'item'");
        assert_eq!(text.trim(), "42");
    }

    #[test]
    fn test_insert_block_multiple_block_params_ordering() {
        env_logger::try_init().ok();
        let input = r#"
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
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor
            .process(xml, &HashMap::new(), XACRO_NS)
            .unwrap();

        // Verify positional ordering: first child -> "first" param, second child -> "second" param
        assert_eq!(result.children.len(), 1);

        let container = result.children[0]
            .as_element()
            .expect("Expected 'container' element");
        assert_eq!(container.name, "container");
        assert_eq!(container.children.len(), 2);

        let first_elem = container.children[0]
            .as_element()
            .expect("Expected first element");
        assert_eq!(first_elem.name, "one");

        let second_elem = container.children[1]
            .as_element()
            .expect("Expected second element");
        assert_eq!(second_elem.name, "two");
    }

    #[test]
    fn test_insert_block_empty_param_name() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="invalid" params="*">
    <wrapper/>
  </xacro:macro>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor.process(xml, &global_properties, XACRO_NS);

        // Should error - empty parameter name (just "*")
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::InvalidParameterName { ref param } if param == "*"
        ));
    }

    // ========== Parameter validation error tests ==========

    #[test]
    fn test_empty_param_name_with_default() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="invalid" params=":=foo">
    <wrapper/>
  </xacro:macro>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor.process(xml, &HashMap::new(), XACRO_NS);

        // Should error - empty parameter name with ":=foo"
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::InvalidParameterName { ref param } if param == ":=foo"
        ));
    }

    #[test]
    fn test_duplicate_param_block_and_regular() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="invalid" params="*foo foo">
    <wrapper/>
  </xacro:macro>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor.process(xml, &HashMap::new(), XACRO_NS);

        // Should error - duplicate parameter "foo" (once as block, once as regular)
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::DuplicateParamDeclaration { ref param } if param == "foo"
        ));
    }

    #[test]
    fn test_duplicate_param_with_defaults() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="invalid" params="foo foo:=1">
    <wrapper/>
  </xacro:macro>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor.process(xml, &HashMap::new(), XACRO_NS);

        // Should error - duplicate parameter "foo"
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::DuplicateParamDeclaration { ref param } if param == "foo"
        ));
    }

    #[test]
    fn test_block_param_attribute_collision() {
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="*content">
    <wrapper>
      <xacro:insert_block name="content"/>
    </wrapper>
  </xacro:macro>

  <xacro:test content="bar">
    <item/>
  </xacro:test>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor.process(xml, &HashMap::new(), XACRO_NS);

        // Should error - "content" is a block param but specified as attribute
        let err = result.unwrap_err();
        assert!(matches!(
            err,
            XacroError::BlockParameterAttributeCollision { ref param } if param == "content"
        ));
    }

    #[test]
    fn test_circular_block_references() {
        env_logger::try_init().ok();
        let input = r#"
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
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor.process(xml, &HashMap::new(), XACRO_NS);

        // Should error - circular block references (block1 -> block2 -> block1)
        let err = result.unwrap_err();
        assert!(matches!(err, XacroError::BlockInsertRecursionLimit { .. }));
    }

    #[test]
    fn test_parameterless_macro() {
        // Test that macros without a params attribute work correctly
        // This was failing because collect_macros required params attribute to exist
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="simple_item">
    <item value="42"/>
  </xacro:macro>

  <xacro:simple_item/>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let macro_processor: MacroProcessor = MacroProcessor::new();
        let result = macro_processor
            .process(xml, &HashMap::new(), XACRO_NS)
            .expect("Parameterless macro should work");

        // Check that macro was expanded
        assert_eq!(result.children.len(), 1);
        let item = result.get_child("item").expect("Expected 'item' element");
        assert_eq!(item.attributes.get("value"), Some(&"42".to_string()));
    }
}

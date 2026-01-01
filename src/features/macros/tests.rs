#[cfg(test)]
mod macro_tests {
    use crate::{
        features::macros::MacroProcessor,
        utils::{pretty_print_xml, print_diff},
        XacroProcessor,
    };
    use log::error;
    use std::collections::HashMap;
    use std::path::Path;

    #[test]
    fn test_macro_basic() {
        env_logger::try_init().ok();
        let macro_processor = MacroProcessor::new();
        let path = Path::new("tests/data/macro_test.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected = XacroProcessor::parse_file("tests/data/macro_test_expected.xacro").unwrap();

        // For this test, no global properties needed
        let global_properties = HashMap::new();
        let result = macro_processor.process(data, &global_properties);

        match result {
            Ok(actual) => {
                let expected_str = pretty_print_xml(&expected);
                let actual_str = pretty_print_xml(&actual);

                if actual != expected {
                    error!("\nXML Difference (actual vs expected):");
                    print_diff(&actual_str, &expected_str);
                    panic!("XML documents are different");
                }
            }
            Err(e) => {
                error!("Processing failed: {:?}", e);
                panic!("Macro processing failed");
            }
        }
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
        let macro_processor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor.process(xml, &global_properties).unwrap();

        // Check that nested macro was expanded
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(item) = &result.children[0] {
            assert_eq!(item.name, "item");
            assert_eq!(item.attributes.get("value"), Some(&"5".to_string()));
        } else {
            panic!("Expected item element");
        }
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
        let macro_processor = MacroProcessor::new();

        // Simulate global properties from PropertyProcessor
        let mut global_properties = HashMap::new();
        global_properties.insert("pi".to_string(), "3.14159".to_string());

        let result = macro_processor.process(xml, &global_properties).unwrap();

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
        let macro_processor = MacroProcessor::new();

        // Global property x=1
        let mut global_properties = HashMap::new();
        global_properties.insert("x".to_string(), "1".to_string());

        let result = macro_processor.process(xml, &global_properties).unwrap();

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
        let macro_processor = MacroProcessor::new();
        let global_properties = HashMap::new();
        let result = macro_processor.process(xml, &global_properties).unwrap();

        // Test 3 levels of nesting all expanded correctly
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(item) = &result.children[0] {
            assert_eq!(item.name, "item");
            assert_eq!(item.attributes.get("value"), Some(&"42".to_string()));
        } else {
            panic!("Expected item element");
        }
    }
}

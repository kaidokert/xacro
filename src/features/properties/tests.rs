#[cfg(test)]
mod property_tests {
    use crate::{
        features::properties::PropertyProcessor, utils::xml::is_xacro_element, XacroProcessor,
    };
    use log::error;
    use std::path::Path;

    #[test]
    fn test_property_basic() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_test.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/property_test_expected.xacro").unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    #[test]
    fn test_property_nested() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_test_nested.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/property_test_nested_expected.xacro").unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    #[test]
    fn test_property_multiple() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_test_multiple.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/property_test_multiple_expected.xacro").unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    #[test]
    fn test_property_multiple_out_of_order() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_test_multiple_out_of_order.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected = XacroProcessor::parse_file(
            "tests/data/property_test_multiple_out_of_order_expected.xacro",
        )
        .unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    #[test]
    fn test_property_attributes() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_test_attributes.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/property_test_attributes_expected.xacro")
                .unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    #[test]
    fn test_property_multi_substitution() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_test_multi_substitution.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected = XacroProcessor::parse_file(
            "tests/data/property_test_multi_substitution_expected.xacro",
        )
        .unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    #[test]
    fn test_property_arithmetic() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/eval_arithmetic.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/eval_arithmetic_expected.urdf").unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    /// Test 3.1: Expressions in property values with recursive evaluation
    /// Property values can contain expressions that reference other properties,
    /// and those properties should be evaluated recursively.
    #[test]
    fn test_property_value_expressions() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_value_expressions.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/property_value_expressions_expected.urdf")
                .unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();
        assert_eq!(output, expected);
    }

    /// Test 3.2: Error propagation from property value expressions
    /// A failing expression inside a property value should surface as
    /// XacroError::EvalError from PropertyProcessor::process
    #[test]
    fn test_property_error_propagation() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_error_in_value.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();

        let result = property_processor.process(data);

        assert!(result.is_err(), "expected eval error in property value");
        let err = result.unwrap_err();

        // Ensure the error variant is EvalError so error propagation is covered
        assert!(
            matches!(
                err,
                crate::error::XacroError::EvalError { ref expr, .. }
                    if expr.contains("unknown_function")
            ),
            "Expected EvalError with 'unknown_function' in expr, got: {:?}",
            err
        );
    }

    /// Test 3.3: Namespace handling for property elements
    /// Only <property> and <xacro:property> should be processed.
    /// Elements like <foo:property> should be preserved in output.
    #[test]
    fn test_property_namespace_handling() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_namespace_handling.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/property_namespace_handling_expected.urdf")
                .unwrap();

        let result = property_processor.process(data);

        if result.is_err() {
            error!("{:?}", result);
        }

        assert!(result.is_ok());
        let (output, _properties) = result.unwrap();

        // Verify that foo:property was NOT removed (should remain in output)
        let mut buf = Vec::new();
        output.write(&mut buf).unwrap();
        let output_str = String::from_utf8(buf).unwrap();
        assert!(
            output_str.contains("foo:property"),
            "foo:property should be preserved in output"
        );

        assert_eq!(output, expected);
    }

    /// Test 3.4: Property flow - HashMap is returned from process()
    /// PropertyProcessor should return both the processed Element and the HashMap
    /// so subsequent processors can use the properties.
    #[test]
    fn test_property_flow_returns_hashmap() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="width" value="0.5"/>
    <xacro:property name="height" value="1.0"/>
    <xacro:property name="computed" value="${width * 2}"/>
</robot>
        "#;
        let data = xmltree::Element::parse(input.as_bytes()).unwrap();

        let result = property_processor.process(data);
        assert!(result.is_ok());

        let (_output, properties) = result.unwrap();

        // Verify properties HashMap contains all properties
        assert_eq!(properties.get("width"), Some(&"0.5".to_string()));
        // Note: pyisheval may format 1.0 as "1.0" or "1" depending on how it's computed
        assert!(
            properties.get("height") == Some(&"1".to_string())
                || properties.get("height") == Some(&"1.0".to_string())
        );
        assert!(
            properties.get("computed") == Some(&"1".to_string())
                || properties.get("computed") == Some(&"1.0".to_string())
        ); // 0.5 * 2 = 1.0
    }

    /// Test 3.5: Conditional value preservation
    /// PropertyProcessor must NOT substitute the 'value' attribute on xacro:if/unless
    /// elements, preserving raw expressions for ConditionProcessor to handle.
    #[test]
    fn test_conditional_value_preservation() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="x" value="5"/>
    <xacro:if value="${x > 3}">
        <foo/>
    </xacro:if>
    <xacro:unless value="${x &lt; 0}">
        <bar/>
    </xacro:unless>
</robot>
        "#;
        let data = xmltree::Element::parse(input.as_bytes()).unwrap();

        let result = property_processor.process(data);
        assert!(result.is_ok());

        let (output, _properties) = result.unwrap();

        // Find the xacro:if element in output
        let if_elem = output
            .children
            .iter()
            .find_map(|child| {
                if let xmltree::XMLNode::Element(elem) = child {
                    if is_xacro_element(elem, "if") {
                        return Some(elem);
                    }
                }
                None
            })
            .expect("Should find xacro:if element");

        // CRITICAL: The 'value' attribute should still contain "${x > 3}"
        // NOT "5 > 3" or "True" or any substituted value
        assert_eq!(
            if_elem.attributes.get("value"),
            Some(&"${x > 3}".to_string()),
            "xacro:if value attribute should preserve raw expression"
        );

        // Find the xacro:unless element
        let unless_elem = output
            .children
            .iter()
            .find_map(|child| {
                if let xmltree::XMLNode::Element(elem) = child {
                    if is_xacro_element(elem, "unless") {
                        return Some(elem);
                    }
                }
                None
            })
            .expect("Should find xacro:unless element");

        // Verify unless also preserves expression (with XML entity escaped)
        assert_eq!(
            unless_elem.attributes.get("value"),
            Some(&"${x < 0}".to_string()),
            "xacro:unless value attribute should preserve raw expression"
        );
    }

    #[test]
    fn test_property_alternative_namespace_prefix() {
        // Test namespace-based detection with alternative prefix (xmlns:x instead of xmlns:xacro)
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:x="http://www.ros.org/wiki/xacro">
  <x:property name="width" value="2"/>
  <x:property name="height" value="3"/>

  <link name="base">
    <box size="${width} ${height} 1"/>
  </link>

  <x:if value="${width > 1}">
    <link name="conditional"/>
  </x:if>
</robot>
        "#;

        let xml = xmltree::Element::parse(input.as_bytes()).unwrap();
        let property_processor = PropertyProcessor::new();
        let (result, properties) = property_processor.process(xml).unwrap();

        // Verify properties were collected despite using 'x:' prefix
        assert_eq!(properties.get("width"), Some(&"2".to_string()));
        assert_eq!(properties.get("height"), Some(&"3".to_string()));

        // Verify property elements were removed (x:property)
        let has_property_elem = result.children.iter().any(|child| {
            if let xmltree::XMLNode::Element(elem) = child {
                elem.name == "property"
            } else {
                false
            }
        });
        assert!(!has_property_elem, "x:property elements should be removed");

        // Verify x:if element is still present (not processed by PropertyProcessor)
        let has_if_elem = result.children.iter().any(|child| {
            if let xmltree::XMLNode::Element(elem) = child {
                is_xacro_element(elem, "if")
            } else {
                false
            }
        });
        assert!(has_if_elem, "x:if element should still be present");

        // Verify expression substitution worked in box element
        let link = result
            .children
            .iter()
            .find_map(|child| {
                if let xmltree::XMLNode::Element(elem) = child {
                    if elem.name == "link"
                        && elem.attributes.get("name") == Some(&"base".to_string())
                    {
                        return Some(elem);
                    }
                }
                None
            })
            .expect("Should find base link");

        let box_elem = link
            .children
            .iter()
            .find_map(|child| {
                if let xmltree::XMLNode::Element(elem) = child {
                    if elem.name == "box" {
                        return Some(elem);
                    }
                }
                None
            })
            .expect("Should find box element");

        assert_eq!(box_elem.attributes.get("size"), Some(&"2 3 1".to_string()));
    }
}

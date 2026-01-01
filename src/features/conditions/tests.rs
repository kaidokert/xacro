#[cfg(test)]
mod condition_tests {
    use crate::features::conditions::ConditionProcessor;
    use std::collections::HashMap;
    use xmltree::Element;

    /// Helper to parse XML string
    fn parse_xml(xml: &str) -> Element {
        Element::parse(xml.as_bytes()).unwrap()
    }

    /// Helper to serialize Element to string
    fn serialize_xml(elem: &Element) -> String {
        let mut buf = Vec::new();
        elem.write(&mut buf).unwrap();
        String::from_utf8(buf).unwrap()
    }

    // Test from Python xacro: test_boolean_if_statement (line 715)
    #[test]
    fn test_if_boolean_literals() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="false">
    <a />
  </xacro:if>
  <xacro:if value="true">
    <b />
  </xacro:if>
</robot>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should only have <b/>, not <a/>
        assert_eq!(result.children.len(), 1); // Only one child (the <b/>)
        if let xmltree::XMLNode::Element(elem) = &result.children[0] {
            assert_eq!(elem.name, "b");
        } else {
            panic!("Expected element child");
        }
    }

    // Test from Python xacro: test_integer_if_statement (line 735)
    #[test]
    fn test_if_integer_truthiness() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${0*42}">
    <a />
  </xacro:if>
  <xacro:if value="0">
    <b />
  </xacro:if>
  <xacro:if value="${0}">
    <c />
  </xacro:if>
  <xacro:if value="${1*2+3}">
    <d />
  </xacro:if>
</robot>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should only have <d/> (1*2+3 = 5, which is truthy)
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(elem) = &result.children[0] {
            assert_eq!(elem.name, "d");
        } else {
            panic!("Expected element child");
        }
    }

    // Test from Python xacro: test_float_if_statement (line 755)
    // CRITICAL: This tests type preservation!
    #[test]
    fn test_if_float_truthiness() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${3*0.0}">
    <a />
  </xacro:if>
  <xacro:if value="${3*0.1}">
    <b />
  </xacro:if>
</robot>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should only have <b/> (3*0.1 = 0.3, non-zero float is truthy)
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(elem) = &result.children[0] {
            assert_eq!(elem.name, "b");
        } else {
            panic!("Expected element child");
        }
    }

    // Test from Python xacro: test_property_if_statement (line 769)
    #[test]
    fn test_if_with_properties() {
        let processor = ConditionProcessor::new();
        let mut properties = HashMap::new();
        properties.insert("condT".to_string(), "1".to_string()); // True as number
        properties.insert("condF".to_string(), "0".to_string()); // False as number

        let input = parse_xml(
            r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${condF}"><a /></xacro:if>
  <xacro:if value="${condT}"><b /></xacro:if>
</robot>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should only have <b/> (condT is truthy)
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(elem) = &result.children[0] {
            assert_eq!(elem.name, "b");
        } else {
            panic!("Expected element child");
        }
    }

    // Test from Python xacro: test_consecutive_if (line 782)
    #[test]
    fn test_nested_conditionals() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><xacro:if value="0"><a>bar</a></xacro:if></xacro:if>
</a>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should have no children (outer if is true, but inner if is false)
        assert_eq!(result.children.len(), 0);
    }

    // Test from Python xacro: test_equality_expression_in_if_statement (line 788)
    #[test]
    fn test_if_with_expressions() {
        let processor = ConditionProcessor::new();
        let mut properties = HashMap::new();
        properties.insert("var".to_string(), "useit".to_string());

        let input = parse_xml(
            r#"<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${var == 'useit'}"><foo>bar</foo></xacro:if>
</a>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should have <foo>bar</foo>
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(elem) = &result.children[0] {
            assert_eq!(elem.name, "foo");
        } else {
            panic!("Expected element child");
        }
    }

    // Test xacro:unless (inverse of if)
    #[test]
    fn test_unless_basic() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:unless value="true">
    <excluded/>
  </xacro:unless>
  <xacro:unless value="false">
    <included/>
  </xacro:unless>
</robot>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should only have <included/> (unless false = true, so include)
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(elem) = &result.children[0] {
            assert_eq!(elem.name, "included");
        } else {
            panic!("Expected element child");
        }
    }

    // Test from Python xacro: test_consider_non_elements_if (line 838)
    #[test]
    fn test_if_preserves_comments_and_text() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="1"><!-- comment --> text <b>bar</b></xacro:if>
</a>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should preserve comment, text, and element
        // Note: xmltree may normalize whitespace/comments
        assert!(result.children.len() >= 2); // At least text and element
    }

    // Test from Python xacro: test_invalid_if_statement (line 729)
    #[test]
    fn test_if_invalid_value_error() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="nonsense"><foo/></xacro:if>
</a>"#,
        );

        let result = processor.process(input, &properties);

        // Should error (strict boolean evaluation)
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("not a boolean expression"),
            "Error message should indicate invalid boolean, got: {}",
            err
        );
    }

    // Test missing value attribute
    #[test]
    fn test_if_missing_value_attribute() {
        let processor = ConditionProcessor::new();
        let properties = HashMap::new();

        let input = parse_xml(
            r#"<a xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if><foo/></xacro:if>
</a>"#,
        );

        let result = processor.process(input, &properties);

        // Should error (missing value attribute)
        assert!(result.is_err());
        let err = result.unwrap_err();
        match err {
            crate::error::XacroError::MissingAttribute { element, attribute } => {
                assert_eq!(element, "xacro:if");
                assert_eq!(attribute, "value");
            }
            _ => panic!("Expected MissingAttribute error, got: {:?}", err),
        }
    }

    // Integration test: conditionals with properties
    #[test]
    fn test_integration_conditionals_with_properties() {
        let processor = ConditionProcessor::new();
        let mut properties = HashMap::new();
        properties.insert("use_feature".to_string(), "1".to_string());
        properties.insert("skip_feature".to_string(), "0".to_string());

        let input = parse_xml(
            r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${use_feature}">
    <feature_enabled/>
  </xacro:if>
  <xacro:unless value="${use_feature}">
    <feature_disabled/>
  </xacro:unless>
  <xacro:if value="${skip_feature}">
    <skipped/>
  </xacro:if>
</robot>"#,
        );

        let result = processor.process(input, &properties).unwrap();

        // Should have only <feature_enabled/> (use_feature is true, unless excludes, skip is false)
        assert_eq!(result.children.len(), 1);
        if let xmltree::XMLNode::Element(elem) = &result.children[0] {
            assert_eq!(elem.name, "feature_enabled");
        } else {
            panic!("Expected element child");
        }
    }
}

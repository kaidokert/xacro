#[cfg(test)]
mod property_tests {
    use crate::{
        features::properties::PropertyProcessor, utils::xml::is_xacro_element, XacroProcessor,
    };
    use log::error;
    use std::path::Path;

    /// Standard xacro namespace for tests
    const XACRO_NS: &str = "http://www.ros.org/wiki/xacro";

    #[test]
    fn test_property_basic() {
        env_logger::try_init().ok();
        let property_processor = PropertyProcessor::new();
        let path = Path::new("tests/data/property_test.xacro");
        let data = XacroProcessor::parse_file(path).unwrap();
        let expected =
            XacroProcessor::parse_file("tests/data/property_test_expected.xacro").unwrap();

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);

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

        let result = property_processor.process(data, XACRO_NS);
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

        let result = property_processor.process(data, XACRO_NS);
        assert!(result.is_ok());

        let (output, _properties) = result.unwrap();

        // Find the xacro:if element in output
        let if_elem = output
            .children
            .iter()
            .find_map(|child| {
                if let xmltree::XMLNode::Element(elem) = child {
                    if is_xacro_element(elem, "if", XACRO_NS) {
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
                    if is_xacro_element(elem, "unless", XACRO_NS) {
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
        let (result, properties) = property_processor.process(xml, XACRO_NS).unwrap();

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
                is_xacro_element(elem, "if", XACRO_NS)
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

    #[test]
    fn test_math_constants() {
        // Test that built-in math constants (pi, e, tau, M_PI, inf, nan) are available
        env_logger::try_init().ok();
        let input = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Test all math constants in attributes -->
  <link name="test">
    <origin rpy="${pi} ${-pi/2} ${tau/4}"/>
    <inertia ixx="${e}" iyy="${M_PI/4}" izz="${tau}"/>
    <meta name="inf_test" value="${inf}"/>
    <meta name="neg_inf_test" value="${-inf}"/>
    <meta name="nan_test" value="${nan}"/>
  </link>
</robot>
"#;
        let processor = XacroProcessor::new();
        let result = processor.run_from_string(input);

        assert!(
            result.is_ok(),
            "Math constants should be available: {:?}",
            result.err()
        );

        let output = result.unwrap();

        // Parse the output XML to verify numeric values
        let root = xmltree::Element::parse(output.as_bytes()).expect("Failed to parse output XML");
        let link = root.get_child("link").expect("Should find link element");

        // Verify pi, -pi/2, tau/4 in origin rpy attribute
        let origin = link
            .get_child("origin")
            .expect("Should find origin element");
        let rpy_str = origin
            .attributes
            .get("rpy")
            .expect("Should have rpy attribute");
        let rpy_values: Vec<f64> = rpy_str
            .split_whitespace()
            .map(|s| s.parse().expect("Should parse as f64"))
            .collect();

        const TOLERANCE: f64 = 1e-9;
        assert!(
            (rpy_values[0] - core::f64::consts::PI).abs() < TOLERANCE,
            "rpy[0] should be pi"
        );
        assert!(
            (rpy_values[1] - (-core::f64::consts::PI / 2.0)).abs() < TOLERANCE,
            "rpy[1] should be -pi/2"
        );
        assert!(
            (rpy_values[2] - (core::f64::consts::TAU / 4.0)).abs() < TOLERANCE,
            "rpy[2] should be tau/4"
        );

        // Verify e, M_PI/4, tau in inertia
        let inertia = link
            .get_child("inertia")
            .expect("Should find inertia element");

        let get_attr_f64 = |elem: &xmltree::Element, name: &str| -> f64 {
            elem.attributes
                .get(name)
                .unwrap_or_else(|| panic!("Should have attribute '{}'", name))
                .parse()
                .unwrap_or_else(|_| panic!("Attribute '{}' should parse to f64", name))
        };

        let ixx = get_attr_f64(inertia, "ixx");
        let iyy = get_attr_f64(inertia, "iyy");
        let izz = get_attr_f64(inertia, "izz");

        assert!(
            (ixx - core::f64::consts::E).abs() < TOLERANCE,
            "ixx should be e"
        );
        assert!(
            (iyy - (core::f64::consts::PI / 4.0)).abs() < TOLERANCE,
            "iyy should be M_PI/4 (verifies M_PI alias)"
        );
        assert!(
            (izz - core::f64::consts::TAU).abs() < TOLERANCE,
            "izz should be tau"
        );

        // Verify inf and nan in meta elements
        let meta_elements: Vec<_> = link
            .children
            .iter()
            .filter_map(|node| node.as_element())
            .filter(|elem| elem.name == "meta")
            .collect();

        let get_meta_value = |name: &str| -> f64 {
            meta_elements
                .iter()
                .find(|e| e.attributes.get("name").map_or(false, |val| val == name))
                .unwrap_or_else(|| panic!("Should find meta element with name '{}'", name))
                .attributes
                .get("value")
                .unwrap_or_else(|| panic!("Meta element '{}' should have a value attribute", name))
                .parse()
                .unwrap_or_else(|_| panic!("Value of meta element '{}' should parse to f64", name))
        };

        let inf_val = get_meta_value("inf_test");
        assert!(
            inf_val.is_infinite() && inf_val.is_sign_positive(),
            "Should be +inf"
        );

        let neg_inf_val = get_meta_value("neg_inf_test");
        assert!(
            neg_inf_val.is_infinite() && neg_inf_val.is_sign_negative(),
            "Should be -inf"
        );

        let nan_val = get_meta_value("nan_test");
        assert!(nan_val.is_nan(), "Should be NaN");
    }

    #[test]
    fn test_macro_body_not_evaluated_during_definition() {
        // This test verifies that expressions inside macro definitions
        // are NOT evaluated during the definition phase, only during expansion.
        //
        // The bug: PropertyProcessor was recursing into macro bodies and trying to
        // evaluate expressions like ${mass * x*x} where mass, x are macro parameters
        // that don't exist until macro expansion time.

        // Test 1: Macro definition only (no call) - should not error
        let input1 = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inertial_box" params="mass x y z">
    <inertia ixx="${(1/12) * mass * (y*y + z*z)}"
             iyy="${(1/12) * mass * (x*x + z*z)}"
             izz="${(1/12) * mass * (x*x + y*y)}"/>
  </xacro:macro>
</robot>"#;

        let processor = XacroProcessor::new();
        let result1 = processor.run_from_string(input1);
        result1.expect("Macro definition only should succeed");
        // Success means we didn't try to evaluate ${mass}, ${x}, etc. during definition
        // (If we had, we'd get "Undefined variable" errors)

        // Test 2: Macro with call - parameters should work during expansion
        let input2 = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inertial_box" params="mass x y z">
    <inertia ixx="${(1/12) * mass * (y*y + z*z)}"
             iyy="${(1/12) * mass * (x*x + z*z)}"
             izz="${(1/12) * mass * (x*x + y*y)}"/>
  </xacro:macro>

  <link name="test">
    <xacro:inertial_box mass="1.0" x="0.1" y="0.2" z="0.3"/>
  </link>
</robot>"#;

        let processor2 = XacroProcessor::new();
        let result2 = processor2.run_from_string(input2);
        let output2 = result2.expect("Macro with call should succeed");

        // Verify the expressions were actually evaluated during expansion
        assert!(
            output2.contains("ixx="),
            "Should have expanded ixx attribute"
        );
        assert!(
            output2.contains("0.01"),
            "Should have computed numeric values"
        );

        // Test 3: Nested macro calls (one macro calling another with parameters)
        let input3 = r#"<?xml version="1.0"?>
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

        let processor3 = XacroProcessor::new();
        let result3 = processor3.run_from_string(input3);
        let output3 = result3.expect("Nested macro calls should succeed");

        // Verify nested macro expansion computed correctly
        // izz = (1/12) * mass * x*x = (1/12) * 2.0 * 0.5*0.5 = 0.041666...
        assert!(
            output3.contains("0.04166"),
            "Expected nested macro expansion to compute izz correctly, got:\n{}",
            output3
        );
        assert!(
            !output3.contains("${"),
            "Expected no unevaluated ${{...}} placeholders in nested macro expansion, got:\n{}",
            output3
        );
    }
}

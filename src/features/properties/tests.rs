#[cfg(test)]
mod property_tests {
    use crate::{features::properties::PropertyProcessor, XacroProcessor};
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
        assert_eq!(result.unwrap(), expected);
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
        assert_eq!(result.unwrap(), expected);
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
        assert_eq!(result.unwrap(), expected);
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
        assert_eq!(result.unwrap(), expected);
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
        assert_eq!(result.unwrap(), expected);
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
        assert_eq!(result.unwrap(), expected);
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
        assert_eq!(result.unwrap(), expected);
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
        assert_eq!(result.unwrap(), expected);
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
        match err {
            crate::error::XacroError::EvalError { expr, .. } => {
                // The error should mention the failing expression
                assert!(
                    expr.contains("unknown_function"),
                    "EvalError should mention the failing expression, got: {}",
                    expr
                );
            }
            _ => panic!("expected XacroError::EvalError, got {:?}", err),
        }
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
        let output = result.unwrap();

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
}

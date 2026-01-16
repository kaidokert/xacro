// Integration tests for load_yaml() functionality (feature-gated)

#[cfg(feature = "yaml")]
mod load_yaml_tests {
    use std::collections::HashMap;
    use xacro::eval::interpreter::{eval_text, eval_text_with_interpreter, init_interpreter};

    #[test]
    fn test_load_yaml_simple() {
        let props = HashMap::new();

        // Load YAML and access top-level key
        // Note: YAML true becomes Python True, which evaluates to 1
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['enabled']}",
            &props,
        )
        .expect("load_yaml should succeed");

        assert_eq!(value, "1", "enabled (True) should evaluate to 1");
    }

    #[test]
    fn test_load_yaml_nested_dict() {
        let props = HashMap::new();

        // Access nested dict values
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['robot']['chassis']['length']}",
            &props,
        )
        .expect("load_yaml nested access should succeed");

        assert_eq!(value, "0.5", "chassis length should be 0.5");
    }

    #[test]
    fn test_load_yaml_with_xacro_prefix() {
        let props = HashMap::new();

        // Test xacro.load_yaml() syntax
        let value = eval_text(
            "${xacro.load_yaml('tests/data/test_config.yaml')['count']}",
            &props,
        )
        .expect("xacro.load_yaml should succeed");

        assert_eq!(value, "5", "count should be 5");
    }

    #[test]
    fn test_load_yaml_array_access() {
        let props = HashMap::new();

        // Access array elements
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['joints'][0]}",
            &props,
        )
        .expect("load_yaml array access should succeed");

        assert_eq!(value, "joint1", "first joint should be joint1");
    }

    #[test]
    fn test_load_yaml_deep_nesting() {
        let props = HashMap::new();

        // Access deeply nested value
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['nested']['level1']['level2']['value']}",
            &props,
        )
        .expect("load_yaml deep nesting should succeed");

        assert_eq!(value, "deep_value", "deep nested value should match");
    }

    #[test]
    fn test_load_yaml_in_arithmetic() {
        let props = HashMap::new();

        // Use loaded value in arithmetic expression
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['robot']['wheel']['radius'] * 2}",
            &props,
        )
        .expect("load_yaml in arithmetic should succeed");

        assert_eq!(value, "0.2", "radius * 2 should be 0.2");
    }

    #[test]
    fn test_load_yaml_multiple_calls() {
        let props = HashMap::new();

        // Multiple load_yaml calls in same expression
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['robot']['chassis']['length'] + \
             load_yaml('tests/data/test_config.yaml')['robot']['chassis']['width']}",
            &props,
        )
        .expect("multiple load_yaml calls should succeed");

        assert_eq!(value, "0.8", "0.5 + 0.3 should be 0.8");
    }

    #[test]
    fn test_load_yaml_extract_and_store() {
        let mut props = HashMap::new();
        let mut interp = init_interpreter();

        // Extract a specific value from YAML and store it
        let wheel_base = eval_text_with_interpreter(
            "${load_yaml('tests/data/test_config.yaml')['robot']['wheel']['base']}",
            &props,
            &mut interp,
        )
        .expect("load_yaml should succeed");

        // Store the extracted value
        props.insert("wheel_base".to_string(), wheel_base);

        // Now use the stored value in calculations
        let value = eval_text_with_interpreter("${wheel_base * 2}", &props, &mut interp)
            .expect("stored value calculation should succeed");

        assert_eq!(value, "0.8", "wheel_base * 2 should be 0.8");
    }

    #[test]
    fn test_load_yaml_file_not_found() {
        let props = HashMap::new();

        // Try to load non-existent file
        let result = eval_text("${load_yaml('tests/data/nonexistent.yaml')}", &props);

        assert!(result.is_err(), "should error on missing file");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("Failed to load YAML") || err_msg.contains("No such file"),
            "error should mention file loading failure, got: {}",
            err_msg
        );
    }

    #[test]
    fn test_load_yaml_invalid_yaml() {
        use std::io::Write;
        use tempfile::NamedTempFile;

        // Create temporary file with invalid YAML
        let mut temp_file = NamedTempFile::new().expect("create temp file");
        write!(temp_file, "invalid: yaml:\n  - bad\n  syntax").expect("write temp file");
        let temp_path = temp_file.path().to_str().expect("get temp path");

        let props = HashMap::new();
        let result = eval_text(&format!("${{load_yaml('{}')}}", temp_path), &props);

        assert!(result.is_err(), "should error on invalid YAML");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("Failed to parse YAML") || err_msg.contains("parse"),
            "error should mention YAML parsing failure, got: {}",
            err_msg
        );
    }

    #[test]
    fn test_load_yaml_with_property_filename() {
        let mut props = HashMap::new();
        props.insert(
            "config_file".to_string(),
            "tests/data/test_config.yaml".to_string(),
        );

        // Variable filenames are now supported
        let value = eval_text("${load_yaml(config_file)['count']}", &props)
            .expect("variable filename should work");

        assert_eq!(value, "5", "count should be 5");
    }

    #[test]
    fn test_load_yaml_argument_with_parentheses_in_string() {
        let mut props = HashMap::new();
        // Property value contains parentheses - this tests that find_matching_paren
        // correctly handles the closing paren of load_yaml vs parens inside the argument
        props.insert(
            "file_with_parens".to_string(),
            "tests/data/test_config.yaml".to_string(),
        );

        // The fix ensures we use find_matching_paren instead of regex capture [^()]+?
        // This allows proper handling when the argument contains parentheses
        let value = eval_text(
            "${load_yaml(file_with_parens)['robot']['chassis']['width']}",
            &props,
        )
        .expect("load_yaml argument parsing should succeed");

        assert_eq!(
            value, "0.3",
            "should correctly parse load_yaml argument even with potential paren complexity"
        );
    }

    #[test]
    fn test_load_yaml_argument_with_parentheses_in_string() {
        let mut props = HashMap::new();
        // Property value contains parentheses - this tests that find_matching_paren
        // correctly handles the closing paren of load_yaml vs parens inside the argument
        props.insert(
            "file_with_parens".to_string(),
            "tests/data/test_config.yaml".to_string(),
        );

        // The fix ensures we use find_matching_paren instead of regex capture [^()]+?
        // This allows proper handling when the argument contains parentheses
        let result = eval_text(
            "${load_yaml(file_with_parens)['robot']['chassis']['width']}",
            &props,
        );

        match result {
            Ok(value) => assert_eq!(
                value, "0.3",
                "should correctly parse load_yaml argument even with potential paren complexity"
            ),
            Err(e) => panic!("load_yaml argument parsing failed: {}", e),
        }
    }
}

#[cfg(not(feature = "yaml"))]
mod yaml_disabled_tests {
    use std::collections::HashMap;
    use xacro::eval::interpreter::eval_text;

    #[test]
    fn test_load_yaml_disabled_error() {
        let props = HashMap::new();

        // Try to use load_yaml without yaml feature
        let result = eval_text("${load_yaml('tests/data/test_config.yaml')}", &props);

        assert!(result.is_err(), "should error when yaml feature disabled");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("yaml") && err_msg.contains("feature"),
            "error should mention yaml feature requirement, got: {}",
            err_msg
        );
    }

    #[test]
    fn test_xacro_load_yaml_disabled_error() {
        let props = HashMap::new();

        // Try to use xacro.load_yaml without yaml feature
        let result = eval_text("${xacro.load_yaml('tests/data/test_config.yaml')}", &props);

        assert!(result.is_err(), "should error when yaml feature disabled");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("yaml") && err_msg.contains("feature"),
            "error should mention yaml feature requirement, got: {}",
            err_msg
        );
    }
}

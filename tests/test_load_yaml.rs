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
        let temp_path = temp_file.path().to_string_lossy().replace('\\', "/");

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
        use std::io::Write;
        use tempfile::Builder;

        let props = HashMap::new();

        // Create a temp YAML file whose path literal includes parentheses
        // This truly tests that find_matching_paren handles parens in the argument
        let mut temp = Builder::new()
            .prefix("config(")
            .suffix(").yaml")
            .tempfile()
            .expect("create temp yaml");
        write!(temp, "robot:\n  chassis:\n    width: 0.3\n").expect("write temp yaml");
        let path = temp.path().to_string_lossy().replace('\\', "/");

        // The fix ensures we use find_matching_paren instead of regex capture [^()]+?
        // This allows proper handling when the argument contains parentheses
        let expr = format!("${{load_yaml('{}')['robot']['chassis']['width']}}", path);
        let value = eval_text(&expr, &props).expect("load_yaml argument parsing should succeed");

        assert_eq!(
            value, "0.3",
            "should correctly parse load_yaml argument even with potential paren complexity"
        );
    }

    #[test]
    fn test_load_yaml_null_value() {
        use std::io::Write;
        use tempfile::NamedTempFile;

        let mut temp_file = NamedTempFile::new().expect("create temp file");
        write!(temp_file, "~").expect("write temp file");
        let temp_path = temp_file.path().to_string_lossy().replace('\\', "/");

        let props = HashMap::new();
        let value = eval_text(&format!("${{load_yaml('{}') + 5}}", &temp_path), &props)
            .expect("load_yaml with null should succeed");
        assert_eq!(value, "5", "null (None) + 5 should be 5");
    }

    #[test]
    fn test_load_yaml_null_in_dict() {
        use std::io::Write;
        use tempfile::NamedTempFile;

        let mut temp_file = NamedTempFile::new().expect("create temp file");
        write!(temp_file, "value: null\nother: 10").expect("write temp file");
        let temp_path = temp_file.path().to_string_lossy().replace('\\', "/");

        let props = HashMap::new();
        let value = eval_text(
            &format!("${{load_yaml('{}')['value']}}", &temp_path),
            &props,
        )
        .expect("load_yaml null value access should succeed");
        assert_eq!(value, "0", "null value should evaluate to 0 (None)");
    }

    #[test]
    fn test_load_yaml_inf_nan_values() {
        let props = HashMap::new();

        // Test positive infinity - evaluates to Python inf
        let result = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['positive_inf']}",
            &props,
        );
        // Python's float('inf') evaluates to inf, which is a valid value
        assert!(
            result.is_ok(),
            "positive_inf should evaluate successfully, got: {:?}",
            result
        );

        // Test negative infinity
        let result = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['negative_inf']}",
            &props,
        );
        assert!(result.is_ok(), "negative_inf should evaluate successfully");

        // Test NaN
        let result = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['not_a_number']}",
            &props,
        );
        assert!(result.is_ok(), "not_a_number should evaluate successfully");

        // Test normal float still works
        let value = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['normal_float']}",
            &props,
        )
        .expect("normal_float should succeed");
        assert_eq!(value, "3.14", "normal float should be '3.14'");
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
            err_msg.contains("load_yaml() requires 'yaml' feature"),
            "error should mention explicit load_yaml yaml feature requirement, got: {}",
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
            err_msg.contains("load_yaml() requires 'yaml' feature"),
            "error should mention explicit load_yaml yaml feature requirement, got: {}",
            err_msg
        );
    }
}

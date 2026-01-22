// Integration tests for load_yaml() functionality (feature-gated)

mod common;
use common::*;

#[cfg(feature = "yaml")]
mod load_yaml_tests {
    use super::*;

    #[test]
    fn test_load_yaml_simple() {
        let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="config" value="${load_yaml('tests/data/test_config.yaml')}"/>
  <link name="test" enabled="${config['enabled']}"/>
</robot>"#;

        let output = run_xacro_expect(input, "load_yaml should succeed");

        // YAML true becomes Python True, which evaluates to 1.0 (displayed as "1.0")
        assert!(
            output.contains(r#"enabled="1.0""#) || output.contains(r#"enabled="1""#),
            "enabled (True) should evaluate to 1.0 or 1. Output:\n{}",
            output
        );
    }

    // NOTE: Most load_yaml() tests are unit tests in src/eval/interpreter/core.rs
    // where they can directly use the eval_text() helper
}

#[cfg(not(feature = "yaml"))]
mod yaml_disabled_tests {
    use super::*;

    #[test]
    fn test_load_yaml_disabled_error() {
        let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="config" value="${load_yaml('tests/data/test_config.yaml')}"/>
</robot>"#;

        let result = test_xacro(input);
        assert!(result.is_err(), "should error when yaml feature disabled");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("load_yaml() requires 'yaml' feature"),
            "error should mention yaml feature requirement, got: {}",
            err_msg
        );
    }

    #[test]
    fn test_xacro_load_yaml_disabled_error() {
        let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="config" value="${xacro.load_yaml('tests/data/test_config.yaml')}"/>
</robot>"#;

        let result = test_xacro(input);
        assert!(result.is_err(), "should error when yaml feature disabled");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("load_yaml() requires 'yaml' feature"),
            "error should mention yaml feature requirement, got: {}",
            err_msg
        );
    }
}

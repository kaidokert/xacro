pub mod interpreter;
pub mod lexer;
pub mod scope;

#[cfg(feature = "yaml")]
pub mod yaml_tag_handler;

pub(crate) use interpreter::*;
pub(crate) use scope::*;
#[cfg(test)]
mod property_tests {

    use super::*;
    use std::collections::HashMap;

    #[test]
    fn test_scope_basic_shadowing() {
        let processor: EvalContext = EvalContext::new();

        // Set up global property
        processor.add_raw_property("x".to_string(), "10".to_string());

        // Verify global resolution through substitute_text
        let result1 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result1, "10", "Global x should be 10");

        // Push macro scope with shadowing
        let mut scope = HashMap::new();
        scope.insert("x".to_string(), "5".to_string());
        processor.push_scope(scope);

        // Verify shadowed resolution
        let result2 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result2, "5", "Scoped x should be 5 (shadowing global)");

        // Pop scope
        processor.pop_scope();

        // Verify global restoration
        let result3 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result3, "10", "After pop, x should be 10 again");
    }

    #[test]
    fn test_scope_fallback_to_global() {
        let processor: EvalContext = EvalContext::new();

        // Set up global properties
        processor.add_raw_property("x".to_string(), "10".to_string());
        processor.add_raw_property("y".to_string(), "20".to_string());

        // Push scope that only shadows x
        let mut scope = HashMap::new();
        scope.insert("x".to_string(), "5".to_string());
        processor.push_scope(scope);

        // x should be shadowed, y should fall back to global
        let result_x = processor.substitute_text("${x}").unwrap();
        let result_y = processor.substitute_text("${y}").unwrap();
        assert_eq!(result_x, "5", "Scoped x should be 5");
        assert_eq!(result_y, "20", "y should fall back to global value 20");

        processor.pop_scope();
    }

    #[test]
    fn test_scope_nested_shadowing() {
        let processor: EvalContext = EvalContext::new();

        // Global property
        processor.add_raw_property("x".to_string(), "10".to_string());

        // First scope level
        let mut scope1 = HashMap::new();
        scope1.insert("x".to_string(), "20".to_string());
        processor.push_scope(scope1);

        let result1 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result1, "20", "First scope: x should be 20");

        // Second scope level (nested)
        let mut scope2 = HashMap::new();
        scope2.insert("x".to_string(), "30".to_string());
        processor.push_scope(scope2);

        let result2 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result2, "30", "Nested scope: x should be 30");

        // Pop innermost scope
        processor.pop_scope();
        let result3 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result3, "20", "After pop, x should be 20 again");

        // Pop outer scope
        processor.pop_scope();
        let result4 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result4, "10", "After second pop, x should be global 10");
    }

    #[test]
    fn test_scope_cache_bypass() {
        let processor: EvalContext = EvalContext::new();

        // Set up global property
        processor.add_raw_property("x".to_string(), "10".to_string());

        // Evaluate to populate cache
        let result1 = processor.substitute_text("${x}").unwrap();
        assert_eq!(result1, "10");

        // Push scope with different value
        let mut scope = HashMap::new();
        scope.insert("x".to_string(), "5".to_string());
        processor.push_scope(scope);

        // Should get scoped value, not cached global value
        let result2 = processor.substitute_text("${x}").unwrap();
        assert_eq!(
            result2, "5",
            "Scoped value should bypass cache and return 5, not cached 10"
        );

        processor.pop_scope();
    }

    #[test]
    fn test_scope_with_expressions() {
        let processor: EvalContext = EvalContext::new();

        // Global property
        processor.add_raw_property("base".to_string(), "10".to_string());

        // Push scope with multiplier
        let mut scope = HashMap::new();
        scope.insert("multiplier".to_string(), "3".to_string());
        processor.push_scope(scope);

        // Expression should use both scoped and global properties
        let result = processor.substitute_text("${base * multiplier}").unwrap();
        assert_eq!(result, "30", "Should compute 10 * 3 = 30");

        processor.pop_scope();
    }

    #[test]
    fn test_scope_undefined_property() {
        let processor: EvalContext = EvalContext::new();

        // Push scope with only x defined
        let mut scope = HashMap::new();
        scope.insert("x".to_string(), "5".to_string());
        processor.push_scope(scope);

        // Trying to resolve undefined property should error
        let result = processor.substitute_text("${y}");
        assert!(
            result.is_err(),
            "Undefined property should error even in scope"
        );

        processor.pop_scope();
    }

    // ========== Additional Unit Tests for substitute_text ==========

    #[test]
    fn test_substitute_text_simple_arithmetic() {
        let processor: EvalContext = EvalContext::new();

        processor.add_raw_property("x".to_string(), "10".to_string());
        processor.add_raw_property("y".to_string(), "5".to_string());

        let add = processor.substitute_text("${x + y}").unwrap();
        assert_eq!(add, "15");

        let multiply = processor.substitute_text("${x * y}").unwrap();
        assert_eq!(multiply, "50");
    }

    #[test]
    fn test_substitute_text_with_functions() {
        let processor: EvalContext = EvalContext::new();

        let result = processor.substitute_text("${abs(-5)}").unwrap();
        assert_eq!(result, "5");

        let result2 = processor.substitute_text("${max(10, 20)}").unwrap();
        assert_eq!(result2, "20");
    }

    #[test]
    fn test_substitute_text_multiple_variables() {
        let processor: EvalContext = EvalContext::new();

        processor.add_raw_property("prefix".to_string(), "robot_".to_string());
        processor.add_raw_property("name".to_string(), "arm".to_string());

        let result = processor.substitute_text("${prefix}${name}").unwrap();
        assert_eq!(result, "robot_arm");
    }

    // ========== Additional Unit Tests for eval_boolean ==========

    #[test]
    fn test_eval_boolean_literals() {
        let processor: EvalContext = EvalContext::new();

        assert_eq!(processor.eval_boolean("True").unwrap(), true);
        assert_eq!(processor.eval_boolean("False").unwrap(), false);
        assert_eq!(processor.eval_boolean("1").unwrap(), true);
        assert_eq!(processor.eval_boolean("0").unwrap(), false);
    }

    #[test]
    fn test_eval_boolean_comparisons() {
        let processor: EvalContext = EvalContext::new();

        processor.add_raw_property("x".to_string(), "10".to_string());
        processor.add_raw_property("y".to_string(), "20".to_string());

        assert_eq!(processor.eval_boolean("${x < y}").unwrap(), true);
        assert_eq!(processor.eval_boolean("${x > y}").unwrap(), false);
        assert_eq!(processor.eval_boolean("${x == 10}").unwrap(), true);
    }

    // ========== Additional Unit Tests for has_property ==========

    #[test]
    fn test_has_property_basic() {
        let processor: EvalContext = EvalContext::new();

        processor.add_raw_property("x".to_string(), "10".to_string());

        assert_eq!(processor.has_property("x"), true);
        assert_eq!(processor.has_property("y"), false);
    }

    #[test]
    fn test_has_property_with_scope() {
        let processor: EvalContext = EvalContext::new();

        processor.add_raw_property("global".to_string(), "1".to_string());

        let mut scope = HashMap::new();
        scope.insert("local".to_string(), "2".to_string());
        processor.push_scope(scope);

        assert_eq!(processor.has_property("global"), true);
        assert_eq!(processor.has_property("local"), true);

        processor.pop_scope();

        assert_eq!(processor.has_property("global"), true);
        assert_eq!(processor.has_property("local"), false);
    }

    #[test]
    fn test_substitute_literal_zero() {
        let processor: EvalContext = EvalContext::new();

        let result = processor.substitute_text("${0}").unwrap();
        assert_eq!(result, "0", "Should evaluate literal 0 expression");
    }
}

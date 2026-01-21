//! Unit tests for directive handlers
//!
//! These tests verify directive handlers in isolation without running the full
//! xacro processing pipeline. They use mock contexts to test specific behaviors
//! and error conditions.

#[cfg(test)]
mod directive_unit_tests {
    use super::super::*;
    use crate::error::XacroError;
    use crate::expand::XacroContext;
    use std::path::PathBuf;
    use xmltree::Element;

    // =============================================================================
    // Test Helpers
    // =============================================================================

    /// Create a minimal XacroContext for testing
    fn mock_xacro_context() -> XacroContext {
        XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        )
    }

    /// Parse XML string to Element for testing
    fn element_from_xml(xml: &str) -> Element {
        Element::parse(xml.as_bytes()).expect("Test XML should be valid")
    }

    // =============================================================================
    // Tests for handle_property_directive
    // =============================================================================

    #[test]
    fn test_property_directive_simple() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<property name="x" value="42"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(result.is_ok(), "Simple property definition should succeed");
        assert_eq!(
            result.unwrap().len(),
            0,
            "Property definitions produce no output"
        );
        assert_eq!(
            ctx.properties.lookup_raw_value("x"),
            Some("42".to_string()),
            "Property should be stored"
        );
    }

    #[test]
    fn test_property_directive_missing_name() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<property value="42"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(result.is_err(), "Property without name should fail");
        assert!(
            matches!(
                result,
                Err(XacroError::MissingAttribute { element, attribute })
                if element == "xacro:property" && attribute == "name"
            ),
            "Should report missing name attribute"
        );
    }

    #[test]
    fn test_property_directive_with_expression() {
        let ctx = mock_xacro_context();
        // Set up a property to reference
        ctx.properties
            .add_raw_property("base".to_string(), "10".to_string());

        let elem = element_from_xml(r#"<property name="doubled" value="${base * 2}"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(result.is_ok(), "Property with expression should succeed");
        // Note: Expression is stored as-is (lazy evaluation)
        assert_eq!(
            ctx.properties.lookup_raw_value("doubled"),
            Some("${base * 2}".to_string()),
            "Expression should be stored without evaluation"
        );
    }

    #[test]
    fn test_property_directive_default_not_set_when_exists() {
        let ctx = mock_xacro_context();
        // Set up existing property
        ctx.properties
            .add_raw_property("x".to_string(), "original".to_string());

        let elem = element_from_xml(r#"<property name="x" default="should_not_appear"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(
            result.is_ok(),
            "Default on existing property should succeed"
        );
        assert_eq!(
            ctx.properties.lookup_raw_value("x"),
            Some("original".to_string()),
            "Existing property should not be overwritten by default"
        );
    }

    #[test]
    fn test_property_directive_default_set_when_missing() {
        let ctx = mock_xacro_context();

        let elem = element_from_xml(r#"<property name="new_prop" default="default_value"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(result.is_ok(), "Default on missing property should succeed");
        assert_eq!(
            ctx.properties.lookup_raw_value("new_prop"),
            Some("default_value".to_string()),
            "Default should be set when property doesn't exist"
        );
    }

    #[test]
    fn test_property_directive_invalid_scope() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<property name="x" value="42" scope="invalid"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(result.is_err(), "Invalid scope should fail");
        assert!(
            matches!(result, Err(XacroError::InvalidScopeAttribute { .. })),
            "Should report invalid scope"
        );
    }

    #[test]
    fn test_property_directive_parent_scope() {
        let ctx = mock_xacro_context();
        let initial_depth = ctx.properties.scope_depth();

        // Push a new scope
        ctx.properties.push_scope(std::collections::HashMap::new());
        let new_depth = ctx.properties.scope_depth();
        assert_eq!(new_depth, initial_depth + 1, "Should have pushed one scope");

        let elem = element_from_xml(r#"<property name="x" value="42" scope="parent"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(result.is_ok(), "Parent scope property should succeed");
        assert_eq!(
            result.unwrap().len(),
            0,
            "Property directive produces no output"
        );

        // Verify property was stored and can be looked up
        assert_eq!(
            ctx.properties.lookup_raw_value("x"),
            Some("42".to_string()),
            "Property should be stored with correct value"
        );

        // Verify it's accessible from current scope
        assert!(
            ctx.properties.has_property("x"),
            "Property should be accessible from current scope"
        );
    }

    #[test]
    fn test_property_directive_global_scope() {
        let ctx = mock_xacro_context();
        let initial_depth = ctx.properties.scope_depth();

        // Push multiple scopes
        ctx.properties.push_scope(std::collections::HashMap::new());
        ctx.properties.push_scope(std::collections::HashMap::new());
        let final_depth = ctx.properties.scope_depth();
        assert_eq!(
            final_depth,
            initial_depth + 2,
            "Should have pushed two scopes"
        );

        let elem = element_from_xml(r#"<property name="x" value="42" scope="global"/>"#);

        let result = handle_property_directive(elem, &ctx);

        assert!(result.is_ok(), "Global scope property should succeed");
        assert_eq!(
            result.unwrap().len(),
            0,
            "Property directive produces no output"
        );

        // Verify property was stored in global scope
        assert_eq!(
            ctx.properties.lookup_raw_value("x"),
            Some("42".to_string()),
            "Property should be in global scope with correct value"
        );

        // Verify it's accessible from nested scope
        assert!(
            ctx.properties.has_property("x"),
            "Property should be accessible from nested scope"
        );
    }

    // =============================================================================
    // Tests for handle_arg_directive
    // =============================================================================

    #[test]
    fn test_arg_directive_simple() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<arg name="my_arg" default="value"/>"#);

        let result = handle_arg_directive(elem, &ctx);

        assert!(result.is_ok(), "Simple arg definition should succeed");
        assert_eq!(
            result.unwrap().len(),
            0,
            "Arg definitions produce no output"
        );
        assert_eq!(
            ctx.args.borrow().get("my_arg"),
            Some(&"value".to_string()),
            "Arg should be stored"
        );
    }

    #[test]
    fn test_arg_directive_missing_name() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<arg default="value"/>"#);

        let result = handle_arg_directive(elem, &ctx);

        assert!(result.is_err(), "Arg without name should fail");
        assert!(
            matches!(
                result,
                Err(XacroError::MissingAttribute { element, attribute })
                if element == "xacro:arg" && attribute == "name"
            ),
            "Should report missing name attribute"
        );
    }

    #[test]
    fn test_arg_directive_cli_overrides_default() {
        let ctx = mock_xacro_context();
        // Simulate CLI providing arg value
        ctx.args
            .borrow_mut()
            .insert("my_arg".to_string(), "cli_value".to_string());

        let elem = element_from_xml(r#"<arg name="my_arg" default="default_value"/>"#);

        let result = handle_arg_directive(elem, &ctx);

        assert!(result.is_ok(), "Arg with CLI override should succeed");
        assert_eq!(
            ctx.args.borrow().get("my_arg"),
            Some(&"cli_value".to_string()),
            "CLI value should take precedence over default"
        );
    }

    #[test]
    fn test_arg_directive_no_default() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<arg name="required_arg"/>"#);

        let result = handle_arg_directive(elem, &ctx);

        assert!(
            result.is_ok(),
            "Arg without default should succeed (will error on use)"
        );
        assert_eq!(
            ctx.args.borrow().get("required_arg"),
            None,
            "Arg without default and no CLI value should not be set"
        );
    }

    #[test]
    fn test_arg_directive_transitive_defaults() {
        let ctx = mock_xacro_context();
        // Set up first arg
        ctx.args
            .borrow_mut()
            .insert("x".to_string(), "10".to_string());

        // Define second arg with default referencing first arg
        let elem = element_from_xml(r#"<arg name="y" default="$(arg x)"/>"#);

        let result = handle_arg_directive(elem, &ctx);

        assert!(result.is_ok(), "Transitive arg default should succeed");
        assert_eq!(
            ctx.args.borrow().get("y"),
            Some(&"10".to_string()),
            "Arg should resolve transitive default"
        );
    }

    #[test]
    fn test_arg_directive_evaluated_name() {
        let ctx = mock_xacro_context();
        // Set up property for name evaluation
        ctx.properties
            .add_raw_property("prefix".to_string(), "my".to_string());

        let elem = element_from_xml(r#"<arg name="${prefix}_arg" default="value"/>"#);

        let result = handle_arg_directive(elem, &ctx);

        assert!(result.is_ok(), "Arg with evaluated name should succeed");
        assert_eq!(
            ctx.args.borrow().get("my_arg"),
            Some(&"value".to_string()),
            "Arg name should be evaluated"
        );
    }

    // =============================================================================
    // Tests for handle_macro_directive
    // =============================================================================

    #[test]
    fn test_macro_directive_simple() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(
            r#"<macro name="my_macro">
                <link name="test"/>
            </macro>"#,
        );

        let result = handle_macro_directive(elem, &ctx);

        assert!(result.is_ok(), "Simple macro definition should succeed");
        assert_eq!(
            result.unwrap().len(),
            0,
            "Macro definitions produce no output"
        );
        assert!(
            ctx.macros.borrow().contains_key("my_macro"),
            "Macro should be stored"
        );
    }

    #[test]
    fn test_macro_directive_missing_name() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(
            r#"<macro>
                <link name="test"/>
            </macro>"#,
        );

        let result = handle_macro_directive(elem, &ctx);

        assert!(result.is_err(), "Macro without name should fail");
        assert!(
            matches!(
                result,
                Err(XacroError::MissingAttribute { element, attribute })
                if element == "xacro:macro" && attribute == "name"
            ),
            "Should report missing name attribute"
        );
    }

    #[test]
    fn test_macro_directive_with_params() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(
            r#"<macro name="my_macro" params="x y:=default">
                <link name="${x}_${y}"/>
            </macro>"#,
        );

        let result = handle_macro_directive(elem, &ctx);

        assert!(result.is_ok(), "Macro with params should succeed");
        let macro_def = ctx.macros.borrow().get("my_macro").cloned();
        assert!(macro_def.is_some(), "Macro should be stored");

        let macro_def = macro_def.unwrap();
        assert_eq!(macro_def.params.len(), 2, "Should have 2 parameters");
        assert!(
            macro_def.params.contains_key("x"),
            "Should have parameter x"
        );
        assert!(
            macro_def.params.contains_key("y"),
            "Should have parameter y"
        );
    }

    #[test]
    fn test_macro_directive_with_block_params() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(
            r#"<macro name="my_macro" params="*content **lazy_content" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <container>
                    <xacro:insert_block name="content"/>
                    <xacro:insert_block name="lazy_content"/>
                </container>
            </macro>"#,
        );

        let result = handle_macro_directive(elem, &ctx);

        assert!(result.is_ok(), "Macro with block params should succeed");
        let macro_def = ctx.macros.borrow().get("my_macro").cloned();
        assert!(macro_def.is_some(), "Macro should be stored");

        let macro_def = macro_def.unwrap();
        assert!(
            macro_def.block_params.contains("content"),
            "Should have block parameter content"
        );
        assert!(
            macro_def.lazy_block_params.contains("lazy_content"),
            "Should have lazy block parameter lazy_content"
        );
    }

    #[test]
    fn test_macro_directive_name_not_evaluated() {
        let ctx = mock_xacro_context();
        // This is a key behavior: macro names are NOT evaluated during definition
        let elem = element_from_xml(
            r#"<macro name="${ns}/my_macro">
                <link name="test"/>
            </macro>"#,
        );

        let result = handle_macro_directive(elem, &ctx);

        assert!(
            result.is_ok(),
            "Macro with unevaluated expression in name should succeed"
        );
        assert!(
            ctx.macros.borrow().contains_key("${ns}/my_macro"),
            "Macro name should be stored literally (not evaluated)"
        );
    }

    #[test]
    fn test_macro_directive_empty_params() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(
            r#"<macro name="my_macro" params="">
                <link name="test"/>
            </macro>"#,
        );

        let result = handle_macro_directive(elem, &ctx);

        assert!(result.is_ok(), "Macro with empty params should succeed");
        let macro_def = ctx.macros.borrow().get("my_macro").cloned();
        assert!(macro_def.is_some(), "Macro should be stored");
        assert_eq!(
            macro_def.unwrap().params.len(),
            0,
            "Should have no parameters"
        );
    }

    // =============================================================================
    // Tests for handle_conditional_directive
    // =============================================================================

    #[test]
    fn test_conditional_if_true() {
        let ctx = mock_xacro_context();
        ctx.properties
            .add_raw_property("flag".to_string(), "1".to_string());

        let elem = element_from_xml(
            r#"<if value="${flag}">
                <link name="included"/>
            </if>"#,
        );

        let result = handle_conditional_directive(elem, &ctx, true);

        assert!(result.is_ok(), "If with true condition should succeed");
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            1,
            "Should expand children when condition is true"
        );
    }

    #[test]
    fn test_conditional_if_false() {
        let ctx = mock_xacro_context();
        ctx.properties
            .add_raw_property("flag".to_string(), "0".to_string());

        let elem = element_from_xml(
            r#"<if value="${flag}">
                <link name="excluded"/>
            </if>"#,
        );

        let result = handle_conditional_directive(elem, &ctx, true);

        assert!(result.is_ok(), "If with false condition should succeed");
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            0,
            "Should not expand children when condition is false"
        );
    }

    #[test]
    fn test_conditional_unless_true() {
        let ctx = mock_xacro_context();
        ctx.properties
            .add_raw_property("flag".to_string(), "1".to_string());

        let elem = element_from_xml(
            r#"<unless value="${flag}">
                <link name="excluded"/>
            </unless>"#,
        );

        let result = handle_conditional_directive(elem, &ctx, false);

        assert!(result.is_ok(), "Unless with true condition should succeed");
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            0,
            "Unless should not expand when condition is true"
        );
    }

    #[test]
    fn test_conditional_unless_false() {
        let ctx = mock_xacro_context();
        ctx.properties
            .add_raw_property("flag".to_string(), "0".to_string());

        let elem = element_from_xml(
            r#"<unless value="${flag}">
                <link name="included"/>
            </unless>"#,
        );

        let result = handle_conditional_directive(elem, &ctx, false);

        assert!(result.is_ok(), "Unless with false condition should succeed");
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            1,
            "Unless should expand when condition is false"
        );
    }

    #[test]
    fn test_conditional_missing_value() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(
            r#"<if>
                <link name="test"/>
            </if>"#,
        );

        let result = handle_conditional_directive(elem, &ctx, true);

        assert!(result.is_err(), "Conditional without value should fail");
        assert!(
            matches!(
                result,
                Err(XacroError::MissingAttribute { element, attribute })
                if element == "xacro:if" && attribute == "value"
            ),
            "Should report missing value attribute"
        );
    }

    #[test]
    fn test_conditional_complex_expression() {
        let ctx = mock_xacro_context();
        ctx.properties
            .add_raw_property("x".to_string(), "5".to_string());
        ctx.properties
            .add_raw_property("y".to_string(), "3".to_string());

        let elem = element_from_xml(
            r#"<if value="${x > y}">
                <link name="included"/>
            </if>"#,
        );

        let result = handle_conditional_directive(elem, &ctx, true);

        assert!(result.is_ok(), "If with complex expression should succeed");
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            1,
            "Should expand when expression evaluates to true"
        );
    }

    #[test]
    fn test_conditional_string_comparison() {
        let ctx = mock_xacro_context();
        ctx.properties
            .add_raw_property("mode".to_string(), "test".to_string());

        let elem = element_from_xml(
            r#"<if value="${mode == 'test'}">
                <link name="included"/>
            </if>"#,
        );

        let result = handle_conditional_directive(elem, &ctx, true);

        assert!(result.is_ok(), "If with string comparison should succeed");
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            1,
            "Should expand when string comparison is true"
        );
    }

    // =============================================================================
    // Tests for handle_insert_block_directive
    // =============================================================================

    #[test]
    fn test_insert_block_missing_name() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<insert_block/>"#);

        let result = handle_insert_block_directive(elem, &ctx);

        assert!(result.is_err(), "Insert block without name should fail");
        assert!(
            matches!(
                result,
                Err(XacroError::MissingAttribute { element, attribute })
                if element == "xacro:insert_block" && attribute == "name"
            ),
            "Should report missing name attribute"
        );
    }

    #[test]
    fn test_insert_block_undefined() {
        let ctx = mock_xacro_context();
        let elem = element_from_xml(r#"<insert_block name="nonexistent"/>"#);

        let result = handle_insert_block_directive(elem, &ctx);

        assert!(
            result.is_err(),
            "Insert block with undefined name should fail"
        );
        assert!(
            matches!(result, Err(XacroError::UndefinedBlock { .. })),
            "Should report undefined block"
        );
    }

    #[test]
    fn test_insert_block_happy_path() {
        let ctx = mock_xacro_context();

        // Set up a block in the block stack
        let mut blocks = std::collections::HashMap::new();
        let block_content = vec![
            xmltree::XMLNode::Element(element_from_xml(r#"<link name="test_link"/>"#)),
            xmltree::XMLNode::Element(element_from_xml(r#"<joint name="test_joint"/>"#)),
        ];
        blocks.insert("my_block".to_string(), block_content.clone());
        ctx.block_stack.borrow_mut().push(blocks);

        let elem = element_from_xml(r#"<insert_block name="my_block"/>"#);

        let result = handle_insert_block_directive(elem, &ctx);

        assert!(
            result.is_ok(),
            "Insert block with defined name should succeed"
        );
        let nodes = result.unwrap();
        assert_eq!(nodes.len(), 2, "Should return both elements from block");

        // Verify content matches what we inserted
        if let xmltree::XMLNode::Element(ref e) = nodes[0] {
            assert_eq!(e.name, "link", "First element should be link");
        } else {
            panic!("Expected first node to be an Element");
        }

        if let xmltree::XMLNode::Element(ref e) = nodes[1] {
            assert_eq!(e.name, "joint", "Second element should be joint");
        } else {
            panic!("Expected second node to be an Element");
        }
    }

    // =============================================================================
    // Tests for check_unimplemented_directive
    // =============================================================================

    #[test]
    fn test_check_unimplemented_directive_not_unimplemented() {
        let elem = element_from_xml(r#"<property name="x" value="42"/>"#);
        let xacro_ns = "http://www.ros.org/wiki/xacro";

        let result = check_unimplemented_directive(&elem, xacro_ns);

        assert!(result.is_ok(), "Implemented directive should not error");
    }

    #[test]
    fn test_check_unimplemented_directive_regular_element() {
        let elem = element_from_xml(r#"<link name="test"/>"#);
        let xacro_ns = "http://www.ros.org/wiki/xacro";

        let result = check_unimplemented_directive(&elem, xacro_ns);

        assert!(result.is_ok(), "Regular element should not error");
    }

    // Note: Testing actual unimplemented directives requires adding them to
    // UNIMPLEMENTED_FEATURES list, which would be artificial for testing.

    // =============================================================================
    // Tests for directive registry dispatch (via expand_element)
    // =============================================================================

    /// Helper to create an element with xacro namespace
    fn xacro_element_from_xml(xml: &str) -> Element {
        // Wrap in robot element with xacro namespace declaration
        let wrapped = format!(
            r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">{}</robot>"#,
            xml
        );
        let robot = Element::parse(wrapped.as_bytes()).expect("Test XML should be valid");
        // Return the first child (the actual element we want to test)
        if let xmltree::XMLNode::Element(elem) = &robot.children[0] {
            elem.clone()
        } else {
            panic!("Expected element child");
        }
    }

    #[test]
    fn test_registry_dispatch_property() {
        let ctx = mock_xacro_context();
        let elem = xacro_element_from_xml(r#"<xacro:property name="x" value="42"/>"#);

        // Use expand_element to test registry dispatch
        let result = expand_element(elem, &ctx);

        assert!(
            result.is_ok(),
            "Property directive via registry should succeed"
        );
        assert_eq!(result.unwrap().len(), 0, "Property produces no output");
        assert_eq!(
            ctx.properties.lookup_raw_value("x"),
            Some("42".to_string()),
            "Property should be stored via registry dispatch"
        );
    }

    #[test]
    fn test_registry_dispatch_arg() {
        let ctx = mock_xacro_context();
        let elem = xacro_element_from_xml(r#"<xacro:arg name="my_arg" default="value"/>"#);

        let result = expand_element(elem, &ctx);

        assert!(result.is_ok(), "Arg directive via registry should succeed");
        assert_eq!(result.unwrap().len(), 0, "Arg produces no output");
    }

    #[test]
    fn test_registry_dispatch_conditional_true() {
        let ctx = mock_xacro_context();
        let elem = xacro_element_from_xml(
            r#"<xacro:if value="true"><link name="conditional"/></xacro:if>"#,
        );

        let result = expand_element(elem, &ctx);

        assert!(
            result.is_ok(),
            "Conditional directive via registry should succeed"
        );
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            1,
            "Should include content when condition is true"
        );
    }

    #[test]
    fn test_registry_dispatch_conditional_false() {
        let ctx = mock_xacro_context();
        let elem = xacro_element_from_xml(
            r#"<xacro:unless value="true"><link name="conditional"/></xacro:unless>"#,
        );

        let result = expand_element(elem, &ctx);

        assert!(
            result.is_ok(),
            "Conditional directive via registry should succeed"
        );
        let nodes = result.unwrap();
        assert_eq!(
            nodes.len(),
            0,
            "Should skip content when unless condition is true"
        );
    }

    #[test]
    fn test_registry_dispatch_macro_definition() {
        let ctx = mock_xacro_context();
        let elem = xacro_element_from_xml(
            r#"<xacro:macro name="test_macro" params="x"><link name="${x}"/></xacro:macro>"#,
        );

        let result = expand_element(elem, &ctx);

        assert!(
            result.is_ok(),
            "Macro definition via registry should succeed"
        );
        assert_eq!(
            result.unwrap().len(),
            0,
            "Macro definition produces no output"
        );

        // Verify macro was registered
        assert!(
            ctx.macros.borrow().contains_key("test_macro"),
            "Macro should be registered via registry dispatch"
        );
    }

    #[test]
    fn test_registry_dispatch_insert_block() {
        let ctx = mock_xacro_context();

        // Set up a block
        let mut blocks = std::collections::HashMap::new();
        blocks.insert(
            "my_block".to_string(),
            vec![xmltree::XMLNode::Element(element_from_xml(
                r#"<link name="from_block"/>"#,
            ))],
        );
        ctx.block_stack.borrow_mut().push(blocks);

        let elem = xacro_element_from_xml(r#"<xacro:insert_block name="my_block"/>"#);

        let result = expand_element(elem, &ctx);

        assert!(result.is_ok(), "Insert_block via registry should succeed");
        let nodes = result.unwrap();
        assert_eq!(nodes.len(), 1, "Should return block content");
    }

    #[test]
    fn test_registry_dispatch_regular_element() {
        let ctx = mock_xacro_context();
        let elem = xacro_element_from_xml(r#"<link name="test"/>"#);

        let result = expand_element(elem, &ctx);

        assert!(result.is_ok(), "Regular element should be processed");
        let nodes = result.unwrap();
        assert_eq!(nodes.len(), 1, "Regular element should return itself");

        if let xmltree::XMLNode::Element(e) = &nodes[0] {
            assert_eq!(e.name, "link", "Element name should be preserved");
        } else {
            panic!("Expected element node");
        }
    }
}

#[cfg(test)]
mod macro_tests {

    use crate::features::macros::Element;
    use crate::features::macros::MacroDefinition;

    use crate::{features::macros::MacroProcessor, XacroError};

    #[test]
    fn test_empty_param_name_with_default() {
        let result = MacroProcessor::parse_params(":=foo");

        let err = result.expect_err("Should error for empty parameter name");
        assert!(
            matches!(
                err,
                XacroError::InvalidParameterName { ref param } if param == ":=foo"
            ),
            "Expected InvalidParameterName error, got: {:?}",
            err
        );
    }

    #[test]
    fn test_duplicate_param_block_and_regular() {
        let result = MacroProcessor::parse_params("*foo foo");

        let err = result.expect_err("Should error for duplicate param declaration");
        assert!(
            matches!(
                err,
                XacroError::DuplicateParamDeclaration { ref param } if param == "foo"
            ),
            "Expected DuplicateParamDeclaration error, got: {:?}",
            err
        );
    }

    #[test]
    fn test_duplicate_param_with_defaults() {
        let result = MacroProcessor::parse_params("x:=1 x:=2");

        let err = result.expect_err("Should error for duplicate param with defaults");
        assert!(
            matches!(
                err,
                XacroError::DuplicateParamDeclaration { ref param } if param == "x"
            ),
            "Expected DuplicateParamDeclaration error, got: {:?}",
            err
        );
    }

    #[test]
    fn test_block_param_attribute_collision() {
        // Create a macro definition with block param "*content"
        let (params, param_order, block_params) =
            MacroProcessor::parse_params("*content").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            content: Element::new("dummy"),
        };

        // Create a call element with attribute content="bar"
        let mut call_elem = Element::new("test");
        call_elem
            .attributes
            .insert("content".to_string(), "bar".to_string());

        // Should error - block param specified as attribute
        let result = MacroProcessor::collect_macro_args(&call_elem, &macro_def);
        let err = result.expect_err("Should error for block param as attribute");
        assert!(
            matches!(
                err,
                XacroError::BlockParameterAttributeCollision { ref param } if param == "content"
            ),
            "Expected BlockParameterAttributeCollision error, got: {:?}",
            err
        );
    }

    #[test]
    fn test_parameterless_macro() {
        // Empty params string should parse successfully
        let result = MacroProcessor::parse_params("");
        assert!(result.is_ok(), "Empty params should be valid");

        let (params, param_order, block_params) = result.unwrap();
        assert!(params.is_empty(), "Should have no params");
        assert!(param_order.is_empty(), "Should have no param order");
        assert!(block_params.is_empty(), "Should have no block params");
    }

    #[test]
    fn test_insert_block_empty_param_name() {
        // Block param with empty name (just "*") should error
        let result = MacroProcessor::parse_params("*");

        let err = result.expect_err("Empty block param name should error");
        assert!(
            matches!(
                err,
                XacroError::InvalidParameterName { ref param } if param == "*"
            ),
            "Expected InvalidParameterName error, got: {:?}",
            err
        );
    }

    #[test]
    fn test_insert_block_with_default_param() {
        // Block params cannot have defaults
        let result = MacroProcessor::parse_params("*content:=default");

        let err = result.expect_err("Block param with default should error");
        assert!(
            matches!(
                err,
                XacroError::BlockParameterWithDefault { ref param } if param.contains("content")
            ),
            "Expected BlockParameterWithDefault error, got: {:?}",
            err
        );
    }

    // ========== Additional Unit Tests for parse_params ==========

    #[test]
    fn test_parse_params_regular_with_defaults() {
        let result = MacroProcessor::parse_params("x:=1 y:=2.5 z:=foo");
        assert!(result.is_ok(), "Regular params with defaults should parse");

        let (params, param_order, block_params) = result.unwrap();

        // Check params map
        assert_eq!(params.len(), 3);
        assert_eq!(params.get("x"), Some(&Some("1".to_string())));
        assert_eq!(params.get("y"), Some(&Some("2.5".to_string())));
        assert_eq!(params.get("z"), Some(&Some("foo".to_string())));

        // Check param order preservation
        assert_eq!(param_order, vec!["x", "y", "z"]);

        // Check no block params
        assert!(block_params.is_empty());
    }

    #[test]
    fn test_parse_params_regular_without_defaults() {
        let result = MacroProcessor::parse_params("a b c");
        assert!(
            result.is_ok(),
            "Regular params without defaults should parse"
        );

        let (params, param_order, block_params) = result.unwrap();

        // Check params map (None = no default)
        assert_eq!(params.len(), 3);
        assert_eq!(params.get("a"), Some(&None));
        assert_eq!(params.get("b"), Some(&None));
        assert_eq!(params.get("c"), Some(&None));

        // Check param order
        assert_eq!(param_order, vec!["a", "b", "c"]);

        // Check no block params
        assert!(block_params.is_empty());
    }

    #[test]
    fn test_parse_params_block_params() {
        let result = MacroProcessor::parse_params("*origin *geometry");
        assert!(result.is_ok(), "Block params should parse");

        let (params, param_order, block_params) = result.unwrap();

        // Check params map (block params have None value)
        assert_eq!(params.len(), 2);
        assert_eq!(params.get("origin"), Some(&None));
        assert_eq!(params.get("geometry"), Some(&None));

        // Check param order
        assert_eq!(param_order, vec!["origin", "geometry"]);

        // Check block params set
        assert_eq!(block_params.len(), 2);
        assert!(block_params.contains("origin"));
        assert!(block_params.contains("geometry"));
    }

    #[test]
    fn test_parse_params_mixed_regular_and_block() {
        let result = MacroProcessor::parse_params("prefix *origin suffix:=default");
        assert!(result.is_ok(), "Mixed params should parse");

        let (params, param_order, block_params) = result.unwrap();

        // Check all params present
        assert_eq!(params.len(), 3);
        assert_eq!(params.get("prefix"), Some(&None));
        assert_eq!(params.get("origin"), Some(&None));
        assert_eq!(params.get("suffix"), Some(&Some("default".to_string())));

        // Check param order preservation
        assert_eq!(param_order, vec!["prefix", "origin", "suffix"]);

        // Check only "origin" is a block param
        assert_eq!(block_params.len(), 1);
        assert!(block_params.contains("origin"));
    }

    #[test]
    fn test_parse_params_default_with_expression() {
        let result = MacroProcessor::parse_params("angle:=${pi/2} scale:=${2*base}");
        assert!(result.is_ok(), "Defaults with expressions should parse");

        let (params, _param_order, _block_params) = result.unwrap();

        // Parser doesn't evaluate, just stores the raw string
        assert_eq!(params.get("angle"), Some(&Some("${pi/2}".to_string())));
        assert_eq!(params.get("scale"), Some(&Some("${2*base}".to_string())));
    }

    // ========== Additional Unit Tests for collect_macro_args ==========

    #[test]
    fn test_collect_macro_args_attributes_only() {
        use crate::features::macros::MacroProcessor;
        use xmltree::Element;

        // Create macro definition with regular params
        let (params, param_order, block_params) =
            MacroProcessor::parse_params("x y z:=default").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            content: Element::new("dummy"),
        };

        // Create call element with attributes
        let mut call_elem = Element::new("test");
        call_elem
            .attributes
            .insert("x".to_string(), "1".to_string());
        call_elem
            .attributes
            .insert("y".to_string(), "2".to_string());

        // Should successfully collect attributes
        let result = MacroProcessor::collect_macro_args(&call_elem, &macro_def);
        assert!(result.is_ok(), "Should collect attributes successfully");

        let (args, blocks) = result.unwrap();
        assert_eq!(args.len(), 2);
        assert_eq!(args.get("x"), Some(&"1".to_string()));
        assert_eq!(args.get("y"), Some(&"2".to_string()));
        assert!(blocks.is_empty(), "Should have no blocks");
    }

    #[test]
    fn test_collect_macro_args_blocks_only() {
        use xmltree::{Element, XMLNode};

        // Create macro definition with block params
        let (params, param_order, block_params) =
            MacroProcessor::parse_params("*origin *geometry").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            content: Element::new("dummy"),
        };

        // Create call element with child elements
        let mut call_elem = Element::new("test");

        let mut origin_elem = Element::new("origin");
        origin_elem
            .attributes
            .insert("xyz".to_string(), "0 0 0".to_string());

        let mut geometry_elem = Element::new("cylinder");
        geometry_elem
            .attributes
            .insert("radius".to_string(), "0.1".to_string());

        call_elem
            .children
            .push(XMLNode::Element(origin_elem.clone()));
        call_elem
            .children
            .push(XMLNode::Element(geometry_elem.clone()));

        // Should successfully collect blocks
        let result = MacroProcessor::collect_macro_args(&call_elem, &macro_def);
        assert!(result.is_ok(), "Should collect blocks successfully");

        let (args, blocks) = result.unwrap();
        assert!(args.is_empty(), "Should have no args");
        assert_eq!(blocks.len(), 2);

        // Verify blocks captured in order
        let origin_block = blocks.get("origin").expect("origin block");
        assert_eq!(origin_block.name, "origin");
        assert_eq!(
            origin_block.attributes.get("xyz"),
            Some(&"0 0 0".to_string())
        );

        let geometry_block = blocks.get("geometry").expect("geometry block");
        assert_eq!(geometry_block.name, "cylinder");
    }

    #[test]
    fn test_collect_macro_args_missing_block_parameter() {
        use xmltree::Element;

        // Create macro definition expecting a block param
        let (params, param_order, block_params) =
            MacroProcessor::parse_params("*content").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            content: Element::new("dummy"),
        };

        // Create call element with NO children
        let call_elem = Element::new("test");

        // Should error - missing required block parameter
        let result = MacroProcessor::collect_macro_args(&call_elem, &macro_def);
        let err = result.expect_err("Should error for missing block parameter");

        assert!(
            matches!(
                err,
                XacroError::MissingBlockParameter { ref macro_name, ref param }
                if macro_name == "test" && param == "content"
            ),
            "Expected MissingBlockParameter error, got: {:?}",
            err
        );
    }

    #[test]
    fn test_collect_macro_args_extra_children() {
        use xmltree::{Element, XMLNode};

        // Create macro definition expecting 1 block param
        let (params, param_order, block_params) =
            MacroProcessor::parse_params("*content").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            content: Element::new("dummy"),
        };

        // Create call element with MORE children than expected
        let mut call_elem = Element::new("test");
        call_elem
            .children
            .push(XMLNode::Element(Element::new("child1")));
        call_elem
            .children
            .push(XMLNode::Element(Element::new("child2")));
        call_elem
            .children
            .push(XMLNode::Element(Element::new("child3")));

        // Should error - too many children provided
        let result = MacroProcessor::collect_macro_args(&call_elem, &macro_def);
        let err = result.expect_err("Should error for extra children");

        assert!(
            matches!(
                err,
                XacroError::UnusedBlock { ref macro_name, extra_count }
                if macro_name == "test" && extra_count == 2
            ),
            "Expected UnusedBlock error with extra_count=2, got: {:?}",
            err
        );
    }

    #[test]
    fn test_collect_macro_args_mixed_params_and_blocks() {
        use xmltree::{Element, XMLNode};

        // Create macro definition with both types
        let (params, param_order, block_params) =
            MacroProcessor::parse_params("prefix *content suffix").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            content: Element::new("dummy"),
        };

        // Create call element with attributes and child
        let mut call_elem = Element::new("test");
        call_elem
            .attributes
            .insert("prefix".to_string(), "pre_".to_string());
        call_elem
            .attributes
            .insert("suffix".to_string(), "_post".to_string());

        let mut content_elem = Element::new("link");
        content_elem
            .attributes
            .insert("name".to_string(), "base".to_string());
        call_elem.children.push(XMLNode::Element(content_elem));

        // Should successfully collect both
        let result = MacroProcessor::collect_macro_args(&call_elem, &macro_def);
        assert!(result.is_ok(), "Should collect mixed args successfully");

        let (args, blocks) = result.unwrap();

        // Verify attributes
        assert_eq!(args.len(), 2);
        assert_eq!(args.get("prefix"), Some(&"pre_".to_string()));
        assert_eq!(args.get("suffix"), Some(&"_post".to_string()));

        // Verify block
        assert_eq!(blocks.len(), 1);
        let content_block = blocks.get("content").expect("content block");
        assert_eq!(content_block.name, "link");
        assert_eq!(
            content_block.attributes.get("name"),
            Some(&"base".to_string())
        );
    }
}

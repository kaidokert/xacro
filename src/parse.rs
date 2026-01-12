pub mod document;
pub mod macro_def;
pub mod xml;

pub use document::*;
pub use macro_def::*;
pub use xml::*;
#[cfg(test)]
mod macro_tests {
    use std::collections::{HashMap, HashSet};

    use super::*;

    use crate::XacroError;
    use xmltree::XMLNode;

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
        let (params, param_order, block_params, _lazy_block_params) =
            MacroProcessor::parse_params("*content").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            lazy_block_params: HashSet::new(),
            content: Element::new("dummy"),
        };

        // Create a call element with attribute content="bar"
        let mut call_elem = Element::new("test");
        call_elem
            .attributes
            .insert(xmltree::AttributeName::local("content"), "bar".to_string());

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
    fn test_namespaced_macro_parameters_rejected() {
        // Test that namespaced attributes are REJECTED on macro calls
        // (Python xacro behavior: "Invalid parameter 'foo:x'")
        let mut params = HashMap::new();
        params.insert("x".to_string(), None);

        let param_order = vec!["x".to_string()];
        let block_params = HashSet::new();

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            lazy_block_params: HashSet::new(),
            content: Element::new("dummy"),
        };

        // Create call with namespaced foo:x attribute
        let mut call_elem = Element::new("test");
        call_elem.attributes.insert(
            xmltree::AttributeName {
                local_name: "x".to_string(),
                namespace: Some("http://example.com/foo".to_string()),
                prefix: Some("foo".to_string()),
            },
            "1".to_string(),
        );

        // Should error - macro parameters cannot have namespace prefixes
        let result = MacroProcessor::collect_macro_args(&call_elem, &macro_def);
        let err = result.expect_err("Should reject namespaced macro parameter");
        assert!(
            matches!(
                err,
                XacroError::InvalidMacroParameter { ref param, .. } if param == "foo:x"
            ),
            "Expected InvalidMacroParameter for 'foo:x', got: {:?}",
            err
        );
    }

    #[test]
    fn test_parameterless_macro() {
        // Empty params string should parse successfully
        let result = MacroProcessor::parse_params("");
        assert!(result.is_ok(), "Empty params should be valid");

        let (params, param_order, block_params, lazy_block_params) = result.unwrap();
        assert!(params.is_empty(), "Should have no params");
        assert!(param_order.is_empty(), "Should have no param order");
        assert!(block_params.is_empty(), "Should have no block params");
        assert!(
            lazy_block_params.is_empty(),
            "Should have no lazy block params"
        );
    }

    #[test]
    fn test_lazy_block_param_single() {
        // Single lazy block param using ** syntax
        let result = MacroProcessor::parse_params("**blk");
        assert!(
            result.is_ok(),
            "Single lazy block param using ** syntax should parse"
        );

        let (params, param_order, block_params, lazy_block_params) = result.unwrap();
        assert_eq!(params.len(), 1, "Exactly one param should be parsed");
        assert!(params.contains_key("blk"), "Param map should contain `blk`");
        assert_eq!(
            param_order,
            vec!["blk"],
            "Param order should contain only `blk`"
        );
        assert!(
            block_params.contains("blk"),
            "`blk` should be treated as a block param"
        );
        assert_eq!(
            block_params.len(),
            1,
            "Only `blk` should be in block_params for **blk"
        );
        assert!(
            lazy_block_params.contains("blk"),
            "`blk` should be tracked as a lazy block param"
        );
        assert_eq!(
            lazy_block_params.len(),
            1,
            "Only `blk` should be in lazy_block_params for **blk"
        );
    }

    #[test]
    fn test_regular_block_param_single() {
        // Single regular block param using * syntax
        let result = MacroProcessor::parse_params("*blk");
        assert!(
            result.is_ok(),
            "Single regular block param using * syntax should parse"
        );

        let (params, param_order, block_params, lazy_block_params) = result.unwrap();
        assert_eq!(params.len(), 1);
        assert!(params.contains_key("blk"));
        assert_eq!(param_order, vec!["blk"]);
        assert!(
            block_params.contains("blk"),
            "`blk` should be a block param"
        );
        assert_eq!(block_params.len(), 1);
        assert!(
            !lazy_block_params.contains("blk"),
            "`blk` should NOT be lazy (only one star)"
        );
        assert!(
            lazy_block_params.is_empty(),
            "No lazy block params for *blk"
        );
    }

    #[test]
    fn test_mixed_block_params() {
        // Mixed block / lazy-block / normal params
        let result = MacroProcessor::parse_params("*a **b c");
        assert!(
            result.is_ok(),
            "Mixed '*a **b c' params should parse correctly"
        );

        let (params, param_order, block_params, lazy_block_params) = result.unwrap();
        assert_eq!(
            params.len(),
            3,
            "Three params should be parsed from '*a **b c'"
        );
        assert!(params.contains_key("a"));
        assert!(params.contains_key("b"));
        assert!(params.contains_key("c"));

        assert_eq!(
            param_order,
            vec!["a", "b", "c"],
            "Param order should follow the declaration order for '*a **b c'"
        );

        // Block params: both *a and **b are block parameters
        assert!(
            block_params.contains("a"),
            "`a` should be treated as a block param"
        );
        assert!(
            block_params.contains("b"),
            "`b` should be treated as a block param"
        );
        assert!(
            !block_params.contains("c"),
            "`c` should not be treated as a block param"
        );
        assert_eq!(
            block_params.len(),
            2,
            "Only `a` and `b` should be in block_params for '*a **b c'"
        );

        // Lazy-block params: only **b is lazy
        assert!(
            !lazy_block_params.contains("a"),
            "`a` should not be treated as a lazy block param"
        );
        assert!(
            lazy_block_params.contains("b"),
            "`b` should be treated as a lazy block param"
        );
        assert!(
            !lazy_block_params.contains("c"),
            "`c` should not be treated as a lazy block param"
        );
        assert_eq!(
            lazy_block_params.len(),
            1,
            "Only `b` should be in lazy_block_params for '*a **b c'"
        );
    }

    #[test]
    fn test_regular_params_with_defaults() {
        // Regular params with defaults should not be block params
        let result = MacroProcessor::parse_params("x:=1 y:=2.5 z:=foo");
        assert!(result.is_ok(), "Regular params with defaults should parse");

        let (params, param_order, block_params, lazy_block_params) = result.unwrap();

        assert_eq!(params.len(), 3);
        assert_eq!(param_order, vec!["x", "y", "z"]);

        // None of these should be block params
        assert!(
            block_params.is_empty(),
            "Defaulted params are not block params"
        );
        assert!(
            lazy_block_params.is_empty(),
            "Defaulted params are not lazy block params"
        );
    }

    #[test]
    fn test_triple_asterisk_rejected() {
        // Triple asterisk should be rejected (invalid)
        let result = MacroProcessor::parse_params("***foo");
        assert!(
            result.is_err(),
            "Triple asterisk should be rejected as invalid"
        );
        assert!(
            matches!(result.unwrap_err(), XacroError::InvalidParameterName { .. }),
            "Should return InvalidParameterName error"
        );
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

        let (params, param_order, block_params, _lazy_block_params) = result.unwrap();

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

        let (params, param_order, block_params, _lazy_block_params) = result.unwrap();

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

        let (params, param_order, block_params, _lazy_block_params) = result.unwrap();

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

        let (params, param_order, block_params, _lazy_block_params) = result.unwrap();

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

        let (params, _param_order, _block_params, _lazy_block_params) = result.unwrap();

        // Parser doesn't evaluate, just stores the raw string
        assert_eq!(params.get("angle"), Some(&Some("${pi/2}".to_string())));
        assert_eq!(params.get("scale"), Some(&Some("${2*base}".to_string())));
    }

    // ========== Additional Unit Tests for collect_macro_args ==========

    #[test]
    fn test_collect_macro_args_attributes_only() {
        // Create macro definition with regular params
        let (params, param_order, block_params, _lazy_block_params) =
            MacroProcessor::parse_params("x y z:=default").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            lazy_block_params: HashSet::new(),
            content: Element::new("dummy"),
        };

        // Create call element with attributes
        let mut call_elem = Element::new("test");
        call_elem
            .attributes
            .insert(xmltree::AttributeName::local("x"), "1".to_string());
        call_elem
            .attributes
            .insert(xmltree::AttributeName::local("y"), "2".to_string());

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
        // Create macro definition with block params
        let (params, param_order, block_params, _lazy_block_params) =
            MacroProcessor::parse_params("*origin *geometry").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            lazy_block_params: HashSet::new(),
            content: Element::new("dummy"),
        };

        // Create call element with child elements
        let mut call_elem = Element::new("test");

        let mut origin_elem = Element::new("origin");
        origin_elem
            .attributes
            .insert(xmltree::AttributeName::local("xyz"), "0 0 0".to_string());

        let mut geometry_elem = Element::new("cylinder");
        geometry_elem
            .attributes
            .insert(xmltree::AttributeName::local("radius"), "0.1".to_string());

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
            origin_block.get_attribute("xyz").map(String::as_str),
            Some("0 0 0")
        );

        let geometry_block = blocks.get("geometry").expect("geometry block");
        assert_eq!(geometry_block.name, "cylinder");
    }

    #[test]
    fn test_collect_macro_args_missing_block_parameter() {
        // Create macro definition expecting a block param
        let (params, param_order, block_params, _lazy_block_params) =
            MacroProcessor::parse_params("*content").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            lazy_block_params: HashSet::new(),
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
        // Create macro definition expecting 1 block param
        let (params, param_order, block_params, _lazy_block_params) =
            MacroProcessor::parse_params("*content").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            lazy_block_params: HashSet::new(),
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
        // Create macro definition with both types
        let (params, param_order, block_params, _lazy_block_params) =
            MacroProcessor::parse_params("prefix *content suffix").expect("Valid params");

        let macro_def = MacroDefinition {
            name: "test".to_string(),
            params,
            param_order,
            block_params,
            lazy_block_params: HashSet::new(),
            content: Element::new("dummy"),
        };

        // Create call element with attributes and child
        let mut call_elem = Element::new("test");
        call_elem
            .attributes
            .insert(xmltree::AttributeName::local("prefix"), "pre_".to_string());
        call_elem
            .attributes
            .insert(xmltree::AttributeName::local("suffix"), "_post".to_string());

        let mut content_elem = Element::new("link");
        content_elem
            .attributes
            .insert(xmltree::AttributeName::local("name"), "base".to_string());
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
            content_block.get_attribute("name").map(String::as_str),
            Some("base")
        );
    }

    /// Test parsing params with single-quoted default value containing spaces
    #[test]
    fn test_parse_params_quoted_single_word() {
        let result = MacroProcessor::parse_params("name:='value'");
        assert!(result.is_ok(), "Should parse single-quoted value");

        let (params, order, blocks, _lazy_blocks) = result.unwrap();
        assert_eq!(params.len(), 1);
        assert_eq!(order.len(), 1);
        assert_eq!(blocks.len(), 0);
        assert_eq!(params.get("name"), Some(&Some("value".to_string())));
    }

    /// Test parsing params with single-quoted default value containing spaces
    #[test]
    fn test_parse_params_quoted_multi_word_single_quotes() {
        let result = MacroProcessor::parse_params("rpy:='0 0 0'");
        assert!(result.is_ok(), "Should parse multi-word quoted value");

        let (params, order, blocks, _lazy_blocks) = result.unwrap();
        assert_eq!(params.len(), 1);
        assert_eq!(order.len(), 1);
        assert_eq!(blocks.len(), 0);
        assert_eq!(params.get("rpy"), Some(&Some("0 0 0".to_string())));
    }

    /// Test parsing params with double-quoted default value containing spaces
    #[test]
    fn test_parse_params_quoted_multi_word_double_quotes() {
        let result = MacroProcessor::parse_params("xyz:=\"1 2 3\"");
        assert!(
            result.is_ok(),
            "Should parse multi-word double-quoted value"
        );

        let (params, _order, _blocks, _lazy_blocks) = result.unwrap();
        assert_eq!(params.len(), 1);
        assert_eq!(params.get("xyz"), Some(&Some("1 2 3".to_string())));
    }

    /// Test parsing multiple params with quoted defaults (real Franka example)
    #[test]
    fn test_parse_params_franka_hand_style() {
        let result = MacroProcessor::parse_params(
            "connected_to:='' ns:='' rpy:='0 0 0' xyz:='0 0 0' safety_distance:=0",
        );
        assert!(
            result.is_ok(),
            "Should parse Franka hand-style params: {:?}",
            result.err()
        );

        let (params, order, blocks, _lazy_blocks) = result.unwrap();
        assert_eq!(params.len(), 5);
        assert_eq!(order.len(), 5);
        assert_eq!(blocks.len(), 0);

        assert_eq!(params.get("connected_to"), Some(&Some("".to_string())));
        assert_eq!(params.get("ns"), Some(&Some("".to_string())));
        assert_eq!(params.get("rpy"), Some(&Some("0 0 0".to_string())));
        assert_eq!(params.get("xyz"), Some(&Some("0 0 0".to_string())));
        assert_eq!(params.get("safety_distance"), Some(&Some("0".to_string())));
    }

    /// Test parsing mixed params: quoted defaults, unquoted defaults, and block params
    #[test]
    fn test_parse_params_mixed_quoted_unquoted_blocks() {
        let result = MacroProcessor::parse_params("pos:='1 2 3' scale:=0.5 *content name:=test");
        assert!(
            result.is_ok(),
            "Should parse mixed params: {:?}",
            result.err()
        );

        let (params, order, blocks, _lazy_blocks) = result.unwrap();

        // Verify we parsed all 4 parameters
        assert_eq!(params.len(), 4, "Expected 4 parameters total");

        // Verify the exact ordering of parameters (critical for block param matching)
        let expected_order: Vec<String> = vec!["pos", "scale", "content", "name"]
            .into_iter()
            .map(String::from)
            .collect();
        assert_eq!(
            order, expected_order,
            "Parameter order should match declaration sequence"
        );

        // Verify the exact set of block parameters
        assert_eq!(blocks.len(), 1, "Expected exactly one block parameter");
        assert!(
            blocks.contains("content"),
            "Blocks set should contain 'content'"
        );

        // Verify parameter values
        assert_eq!(params.get("pos"), Some(&Some("1 2 3".to_string())));
        assert_eq!(params.get("scale"), Some(&Some("0.5".to_string())));
        assert_eq!(params.get("content"), Some(&None)); // Block param has no default
        assert_eq!(params.get("name"), Some(&Some("test".to_string())));
    }

    /// Test that quoted defaults with ':=' inside quotes are handled correctly
    #[test]
    fn test_parse_params_complex_quoted_content() {
        let result = MacroProcessor::parse_params("expr:='x:=5 y:=10' name:=test");
        assert!(
            result.is_ok(),
            "Should handle ':=' inside quoted string: {:?}",
            result.err()
        );

        let (params, order, _blocks, _lazy_blocks) = result.unwrap();
        assert_eq!(params.len(), 2);
        assert_eq!(order, vec!["expr", "name"]);
        assert_eq!(params.get("expr"), Some(&Some("x:=5 y:=10".to_string())));
        assert_eq!(params.get("name"), Some(&Some("test".to_string())));
    }

    /// Test that block parameters with quoted defaults are rejected
    #[test]
    fn test_parse_params_block_param_with_quoted_default_is_error() {
        use crate::error::XacroError;

        // "*content" is a block parameter; it must not have a default, quoted or otherwise
        let result = MacroProcessor::parse_params("*content:='foo bar'");

        // Ensure we still reject block parameters with defaults when the default is quoted
        assert!(
            matches!(result, Err(XacroError::BlockParameterWithDefault { .. })),
            "Expected BlockParameterWithDefault error for quoted default on block param, got: {:?}",
            result
        );
    }

    /// Test that duplicate parameters are rejected when one has a quoted default
    #[test]
    fn test_parse_params_duplicate_with_quoted_default_is_error() {
        use crate::error::XacroError;

        // Same parameter name "rpy" appears twice, once with a quoted default
        let result = MacroProcessor::parse_params("rpy:='0 0 0' rpy:=1");

        // Ensure duplicate detection still works with quoted defaults and String-based token handling
        assert!(
            matches!(result, Err(XacroError::DuplicateParamDeclaration { .. })),
            "Expected DuplicateParamDeclaration error for duplicate param with quoted default, got: {:?}",
            result
        );
    }

    /// Test edge case: single-character value that's not a quote
    #[test]
    fn test_parse_params_single_char_value() {
        // This was a potential panic case before using strip_prefix/strip_suffix
        // A value like "p:=x" (single char) should work fine
        let result = MacroProcessor::parse_params("p:=x");

        assert!(
            result.is_ok(),
            "Should parse single-char non-quoted value: {:?}",
            result.err()
        );

        let (params, _, _, _) = result.unwrap();
        assert_eq!(params.get("p"), Some(&Some("x".to_string())));
    }

    /// Test edge case: properly quoted single-character string
    #[test]
    fn test_parse_params_single_char_quoted_properly() {
        // A properly quoted single character: "p:='x'"
        let result = MacroProcessor::parse_params("p:='x'");

        assert!(
            result.is_ok(),
            "Should parse single-char quoted value: {:?}",
            result.err()
        );

        let (params, _, _, _) = result.unwrap();
        // Quote-stripping should extract the 'x'
        assert_eq!(params.get("p"), Some(&Some("x".to_string())));
    }

    /// Test escape sequences in quoted defaults (NOT YET SUPPORTED)
    ///
    /// This test documents a limitation: the tokenizer does not support escape
    /// sequences like \' or \" within quoted strings.
    ///
    /// Decision: Deferred pending verification that Python xacro supports this.
    #[test]
    #[ignore = "Escape sequences not yet supported"]
    fn test_parse_params_escape_sequences_not_supported() {
        // Example: Literal single quote inside single-quoted string
        // Expected: name="it's here"
        // Actual: Parse error - tokenizer exits quote mode at \'
        let result = MacroProcessor::parse_params(r"name:='it\'s here'");

        // When escape sequences ARE implemented, this should parse correctly:
        // assert!(result.is_ok());
        // let (params, _, _) = result.unwrap();
        // assert_eq!(params.get("name"), Some(&Some("it's here".to_string())));

        // For now, document that it's expected to fail or produce wrong tokens
        // (This test is ignored, so it won't run in CI)
        let _ = result; // Avoid unused variable warning
    }

    /// Test edge case: unbalanced quotes
    ///
    /// The tokenizer validates that all quotes are properly closed and returns
    /// an error if an unclosed quote is detected.
    #[test]
    fn test_parse_params_unbalanced_quotes() {
        use crate::error::XacroError;

        // Missing closing quote - should return UnbalancedQuote error
        let result = MacroProcessor::parse_params("rpy:='0 0 0");

        assert!(result.is_err(), "Unbalanced quotes should return error");

        // Verify it's the correct error type
        assert!(
            matches!(result, Err(XacroError::UnbalancedQuote { .. })),
            "Expected UnbalancedQuote error, got: {:?}",
            result
        );
    }

    /// Test edge case: adjacent quoted strings without space
    ///
    /// Documents behavior when quoted params have no whitespace between them.
    /// The tokenizer requires whitespace to separate tokens, so adjacent quotes
    /// are treated as a single token.
    #[test]
    fn test_parse_params_adjacent_quoted_strings() {
        // Two quoted params with no space between closing ' and next param
        let result = MacroProcessor::parse_params("a:='val1'b:='val2'");

        // Currently, the tokenizer treats this as ONE token because there's
        // no whitespace between 'val1' and b
        assert!(
            result.is_ok(),
            "Adjacent quoted strings parse as single token: {:?}",
            result.err()
        );

        let (params, order, _, _) = result.unwrap();
        // Only one parameter is parsed - the entire string is one token
        assert_eq!(params.len(), 1, "Adjacent quotes treated as single token");
        assert_eq!(order, vec!["a"]);
        // The value includes everything after := including the adjacent param
        assert_eq!(params.get("a"), Some(&Some("val1'b:='val2".to_string())));
    }

    /// Test edge case: quote character in parameter name causes unbalanced quote
    #[test]
    fn test_parse_params_quote_in_param_name() {
        use crate::error::XacroError;

        // A quote in the parameter name starts quote mode: "param':=value"
        // The tokenizer sees param' and enters quote mode, then reads :=value
        // Since there's no closing ', this is an unbalanced quote
        let result = MacroProcessor::parse_params("param':=value");

        assert!(
            result.is_err(),
            "Quote in param name causes unbalanced quote error"
        );

        // Verify it's the correct error type
        assert!(
            matches!(result, Err(XacroError::UnbalancedQuote { .. })),
            "Expected UnbalancedQuote error, got: {:?}",
            result
        );
    }

    #[test]
    fn test_parse_params_compat_mode_duplicate() {
        // Test that compat mode accepts duplicates with last-declaration-wins
        let result = MacroProcessor::parse_params_compat("x:=1 y:=2 x:=3");
        assert!(result.is_ok(), "Compat mode should accept duplicates");

        let (params, param_order, block_params, _lazy_block_params) = result.unwrap();

        // Last declaration wins for value
        assert_eq!(params.get("x"), Some(&Some("3".to_string())));
        assert_eq!(params.get("y"), Some(&Some("2".to_string())));

        // Order should only contain unique params, in first-seen order
        assert_eq!(param_order, vec!["x", "y"]);

        assert!(block_params.is_empty());
    }

    #[test]
    fn test_parse_params_compat_mode_block_duplicate() {
        // Test that compat mode accepts duplicate block params
        let result = MacroProcessor::parse_params_compat("*body *body");
        assert!(
            result.is_ok(),
            "Compat mode should accept duplicate block params"
        );

        let (params, param_order, block_params, _lazy_block_params) = result.unwrap();

        // Should have one entry
        assert_eq!(params.len(), 1);
        assert_eq!(params.get("body"), Some(&None));

        // Order should only contain unique params
        assert_eq!(param_order, vec!["body"]);

        // Should be marked as block param
        assert!(block_params.contains("body"));
    }
}

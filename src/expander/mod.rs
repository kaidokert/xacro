//! Single-pass recursive expander
//!
//! This module implements the "single-pass recursive walker" architecture. This
//! expander processes nodes in document order, enabling lazy includes and
//! proper in-order semantics.
//!
//! Key features:
//! - Lazy evaluation: Conditionals skip entire subtrees
//! - In-order semantics: Definitions available immediately
//! - Borrow-safe: Free functions with Rc for shared ownership
//! - Scope support: Macro parameters shadow global properties

use crate::{
    error::XacroError,
    parse::xml::{is_known_xacro_uri, is_xacro_element},
};
use xmltree::{Element, XMLNode};

mod children;
mod directives;
#[cfg(test)]
mod directives_tests;
mod guards;
mod include;
mod macro_call;
mod utils;

// Internal use of submodules
use children::expand_children_list;
use directives::{check_unimplemented_directive, get_directive_registry};
use guards::DepthGuard;
use include::handle_include_directive;
use macro_call::{expand_macro_call, is_macro_call};
use utils::normalize_attribute_whitespace;

// Re-export XacroContext from expand module
pub use crate::expand::XacroContext;

/// Main recursive expansion function
///
/// This is the core of the single-pass expander. It processes nodes in document
/// order, handling definitions, conditionals, includes, and macro calls.
///
/// Returns Vec<XMLNode> because a single xacro element can expand to:
/// - Zero nodes (property definitions)
/// - One node (regular elements)
/// - Many nodes (macro expansion)
pub(crate) fn expand_node(
    node: XMLNode,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Check recursion depth at entry to catch infinite loops early
    if *ctx.recursion_depth.borrow() >= ctx.max_recursion_depth {
        return Err(XacroError::MacroRecursionLimit {
            depth: *ctx.recursion_depth.borrow(),
            limit: ctx.max_recursion_depth,
        });
    }

    // Create RAII guard (increments depth on construction, decrements on drop)
    let _depth_guard = DepthGuard::new(&ctx.recursion_depth);

    // Process node (guard will decrement depth on scope exit, even if panic occurs)
    match node {
        XMLNode::Element(elem) => expand_element(elem, ctx),
        XMLNode::Text(text) => {
            // Resolve both ${...} expressions and $(...) extensions in text
            let loc = ctx.get_location_context();
            let resolved = ctx.properties.substitute_all(&text, Some(&loc))?;
            Ok(vec![XMLNode::Text(resolved)])
        }
        other => Ok(vec![other]), // Comments, CDATA, etc. pass through
    }
}

/// Extract directive name from element if it's a xacro directive
///
/// Returns Some(name) if element is a xacro directive, None otherwise.
/// Accepts both exact namespace match and any known xacro URI variant.
fn extract_directive_name<'a>(
    elem: &'a Element,
    xacro_ns: &str,
) -> Option<&'a str> {
    let elem_ns = elem.namespace.as_deref();

    // Require that document declared a xacro namespace
    if xacro_ns.is_empty() {
        return None;
    }

    // Accept exact match with current namespace OR any known xacro URI
    // This ensures directives from included files with different namespace variants
    // (e.g., http://ros.org/wiki/xacro vs http://www.ros.org/wiki/xacro) are recognized
    if elem_ns == Some(xacro_ns) || elem_ns.is_some_and(is_known_xacro_uri) {
        Some(&elem.name)
    } else {
        None
    }
}

/// Expand an element node
fn expand_element(
    mut elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    let xacro_ns = ctx.current_xacro_ns();

    // Try directive registry first (O(1) HashMap lookup)
    if let Some(directive_name) = extract_directive_name(&elem, &xacro_ns) {
        let registry = get_directive_registry();
        if let Some(handler) = registry.get(directive_name) {
            return handler.handle(elem, ctx);
        }
    }

    // Include directive (not in registry yet - kept separate for now)
    if is_xacro_element(&elem, "include", &xacro_ns) {
        return handle_include_directive(elem, ctx);
    }

    // Check for known but unimplemented xacro directives
    check_unimplemented_directive(&elem, &xacro_ns)?;

    // 7. Macro calls (with re-scan for nested macros)
    if is_macro_call(&elem, &ctx.macros.borrow(), &xacro_ns) {
        // Substitute both ${...} and $(...) in macro call attributes BEFORE expansion
        // This allows nested macros to use parameters from outer macro scope:
        // <xacro:macro name="outer" params="prefix">
        //   <xacro:inner prefix="${prefix}"/>  <!-- ${prefix} evaluated here -->
        // </xacro:macro>
        let loc = ctx.get_location_context();
        for attr_value in elem.attributes.values_mut() {
            let substituted = ctx.properties.substitute_all(attr_value, Some(&loc))?;
            *attr_value = normalize_attribute_whitespace(&substituted);
        }

        // Capture parent scope depth BEFORE calling expand_macro_call (RefCell safety)
        let parent_scope_depth = ctx.properties.scope_depth();
        let expanded_nodes = expand_macro_call(&elem, ctx, parent_scope_depth)?;
        // Re-scan the expanded result to handle macros that generate macros
        return expand_children_list(expanded_nodes, ctx);
    }

    // 8. Regular elements: Substitute attributes and recurse to children
    // Substitute both ${...} expressions and $(...) extensions in all attributes
    // and normalize whitespace per XML spec (newlines/tabs -> spaces)
    let loc = ctx.get_location_context();
    for attr_value in elem.attributes.values_mut() {
        let substituted = ctx.properties.substitute_all(attr_value, Some(&loc))?;
        *attr_value = normalize_attribute_whitespace(&substituted);
    }

    // Recursively expand children
    let expanded_children = expand_children_list(elem.children, ctx)?;
    elem.children = expanded_children;

    Ok(vec![XMLNode::Element(elem)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn test_context_creation() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        assert_eq!(ctx.current_xacro_ns(), "http://www.ros.org/wiki/xacro");
        assert_eq!(ctx.include_stack.borrow().len(), 0);
        assert_eq!(ctx.block_stack.borrow().len(), 0);
    }

    #[test]
    fn test_expand_text_node() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        // Set up a test property
        ctx.properties
            .add_raw_property("x".to_string(), "42".to_string());

        let text_node = XMLNode::Text("value: ${x}".to_string());
        let result = expand_node(text_node, &ctx).unwrap();

        assert_eq!(result.len(), 1);
        let text = result[0].as_text().expect("Expected text node");
        assert_eq!(text, "value: 42");
    }

    #[test]
    fn test_expand_empty_property() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        // Add property with empty value
        ctx.properties
            .add_raw_property("empty".to_string(), "".to_string());

        let text_node = XMLNode::Text("prefix_${empty}_suffix".to_string());
        let result = expand_node(text_node, &ctx).unwrap();

        assert_eq!(result.len(), 1);
        let text = result[0].as_text().expect("Expected text node");
        assert_eq!(text, "prefix__suffix");
    }

    #[test]
    fn test_expand_nested_expressions() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        // Set up properties for nested substitution
        ctx.properties
            .add_raw_property("x".to_string(), "10".to_string());
        ctx.properties
            .add_raw_property("y".to_string(), "20".to_string());

        // Multiple expressions in one text node
        let text_node = XMLNode::Text("sum: ${x + y}, product: ${x * y}".to_string());
        let result = expand_node(text_node, &ctx).unwrap();

        assert_eq!(result.len(), 1);
        let text = result[0].as_text().expect("Expected text node");
        assert_eq!(text, "sum: 30, product: 200");
    }

    #[test]
    fn test_expand_nested_property_reference() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        // Property containing reference to another property
        ctx.properties
            .add_raw_property("base".to_string(), "value".to_string());
        ctx.properties
            .add_raw_property("derived".to_string(), "${base}_extended".to_string());

        let text_node = XMLNode::Text("result: ${derived}".to_string());
        let result = expand_node(text_node, &ctx).unwrap();

        assert_eq!(result.len(), 1);
        let text = result[0].as_text().expect("Expected text node");
        assert_eq!(text, "result: value_extended");
    }

    #[test]
    fn test_expand_error_undefined_property() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        // Try to expand text with undefined property
        let text_node = XMLNode::Text("value: ${undefined_property}".to_string());
        let result = expand_node(text_node, &ctx);

        let err = result.expect_err("Should error on undefined property reference");
        assert!(
            matches!(
                err,
                XacroError::EvalError { ref expr, .. } if expr.contains("undefined_property")
            ),
            "Expected EvalError mentioning 'undefined_property', got: {:?}",
            err
        );
    }

    #[test]
    fn test_expand_error_malformed_expression() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        // Try to expand text with malformed expression (invalid syntax)
        let text_node = XMLNode::Text("value: ${1 +* 2}".to_string());
        let result = expand_node(text_node, &ctx);

        assert!(
            result.is_err(),
            "Should error on malformed expression syntax"
        );
    }

    #[test]
    fn test_expand_error_invalid_operation() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        ctx.properties
            .add_raw_property("text".to_string(), "hello".to_string());

        // Try to perform numeric operation on string
        let text_node = XMLNode::Text("value: ${text * 2}".to_string());
        let result = expand_node(text_node, &ctx);

        assert!(
            result.is_err(),
            "Should error on invalid operation (multiply string)"
        );
    }
}

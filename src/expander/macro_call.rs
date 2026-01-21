//! Macro call detection and expansion
//!
//! This module handles:
//! - Detecting if an element is a macro call vs a directive
//! - Expanding macro calls with parameters and blocks
//! - Parameter forwarding with parent scope reference (`^`)
//! - Block parameter semantics (`*param` vs `**param`)

use crate::{
    directives::{IMPLEMENTED_DIRECTIVES, UNIMPLEMENTED_DIRECTIVES},
    error::XacroError,
    parse::{macro_def::MacroProcessor, xml::is_known_xacro_uri},
};
use std::{collections::HashMap, rc::Rc};
use xmltree::{Element, XMLNode};

use super::*;

/// Check if an element is a macro call
///
/// Returns true if:
/// 1. Element is in xacro namespace
/// 2. Element name is NOT a known directive (implemented or unimplemented)
/// 3. Element name matches a defined macro
///
/// # Arguments
/// * `elem` - The element to check
/// * `macros` - HashMap of defined macros
/// * `xacro_ns` - The xacro namespace URI for this document
pub(crate) fn is_macro_call(
    elem: &Element,
    macros: &HashMap<String, Rc<crate::parse::macro_def::MacroDefinition>>,
    xacro_ns: &str,
) -> bool {
    // Must be in xacro namespace (check resolved URI, not prefix).
    // Accept exact match with the document's declared namespace, OR any known xacro URI variant
    // to support cross-namespace macro expansion when includes use different xacro URI variants.
    let in_xacro_ns = !xacro_ns.is_empty()
        && elem
            .namespace
            .as_deref()
            .is_some_and(|elem_ns| elem_ns == xacro_ns || is_known_xacro_uri(elem_ns));

    if !in_xacro_ns {
        return false;
    }

    // Known directives (implemented and unimplemented) are NOT macro calls
    // Element name is the local name (without prefix)
    let elem_name = elem.name.as_str();
    if IMPLEMENTED_DIRECTIVES.contains(&elem_name) || UNIMPLEMENTED_DIRECTIVES.contains(&elem_name)
    {
        return false;
    }

    // Check if it matches a defined macro
    macros.contains_key(&elem.name)
}

/// Expand a macro call
///
/// Handles:
/// - Parameter collection and default evaluation
/// - Block parameter pre-expansion (in caller's scope)
/// - Parent scope forwarding (`^` operator)
/// - Lazy vs regular block semantics (`**param` vs `*param`)
/// - Recursive expansion of macro content
///
/// # Arguments
/// * `call_elem` - The macro call element
/// * `ctx` - XacroContext with macros, properties, and stacks
/// * `parent_scope_depth` - Depth of parent scope for forwarding
///
/// # Returns
/// Expanded nodes from macro content
pub(crate) fn expand_macro_call(
    call_elem: &Element,
    ctx: &XacroContext,
    parent_scope_depth: usize,
) -> Result<Vec<XMLNode>, XacroError> {
    // Extract macro name (element name is already the local name without prefix)
    let macro_name = &call_elem.name;

    // Look up macro definition and clone Rc (cheap pointer copy)
    let macro_def = ctx
        .macros
        .borrow()
        .get(macro_name)
        .cloned()
        .ok_or_else(|| XacroError::UndefinedMacro(macro_name.to_string()))?;

    // Pre-process macro call children to expand conditionals (xacro:if, xacro:unless)
    // before collecting block parameters. This matches Python xacro's behavior:
    // conditionals are evaluated before block parameters are collected.
    let mut processed_elem = call_elem.clone();
    processed_elem.children =
        expand_children_list(core::mem::take(&mut processed_elem.children), ctx)?;

    // Collect macro arguments and blocks from processed element
    let (args, blocks) = MacroProcessor::collect_macro_args(&processed_elem, &macro_def)?;

    // Pre-expand blocks in caller's scope before entering macro scope
    // Must happen BEFORE pushing macro's parameter scope to ensure
    // block content is evaluated with caller's properties, not macro's parameters
    let mut expanded_blocks = HashMap::new();
    let mut unexpanded_blocks = blocks;

    log::debug!(
        "expand_macro_call: macro '{}' received {} blocks from collect_macro_args: {:?}",
        macro_name,
        unexpanded_blocks.len(),
        unexpanded_blocks.keys().collect::<Vec<_>>()
    );

    // Pre-expand blocks in declaration order to preserve in-order evaluation semantics
    for param_name in &macro_def.param_order {
        if macro_def.block_params.contains(param_name) {
            // collect_macro_args guarantees the block exists, so unwrap is safe
            let raw_block = unexpanded_blocks.remove(param_name).unwrap();
            log::debug!(
                "expand_macro_call: expanding block param '{}' (element '<{}>')",
                param_name,
                raw_block.name
            );

            // Behavior depends on * vs ** prefix:
            // *param (regular block) → insert the element itself
            // **param (lazy block) → insert only the element's children
            let is_lazy = macro_def.lazy_block_params.contains(param_name);
            let expanded = if is_lazy {
                // Lazy block (**param): expand children only
                // Example: <wrapper><inner/></wrapper> → inserts <inner/> only
                expand_children_list(raw_block.children, ctx)?
            } else {
                // Regular block (*param): expand the element itself
                // Example: <link name="test"/> → inserts <link name="test"/>
                expand_node(XMLNode::Element(raw_block), ctx)?
            };

            log::debug!(
                "expand_macro_call: block param '{}' {} expanded to {} nodes",
                param_name,
                if is_lazy { "(lazy)" } else { "(regular)" },
                expanded.len()
            );
            expanded_blocks.insert(param_name.clone(), expanded);
        }
    }

    log::debug!(
        "expand_macro_call: pushing {} expanded blocks to stack: {:?}",
        expanded_blocks.len(),
        expanded_blocks.keys().collect::<Vec<_>>()
    );

    // Resolve parameters with defaults, supporting chained defaults
    // Process in declaration order so later defaults can reference earlier ones
    // Example: params="a:=1 b:=${a*2} c:=${b*3}"
    //
    // Optimization: Instead of cloning resolved_params on each iteration (O(N²)),
    // push a single scope and incrementally add parameters to it (O(N))
    ctx.properties.push_scope(HashMap::new());

    // Create guard immediately after push to ensure cleanup on any error path
    let _scope_guard = ScopeGuard {
        properties: &ctx.properties,
    };

    for param_name in &macro_def.param_order {
        if macro_def.block_params.contains(param_name) {
            continue; // Skip block parameters
        }

        if let Some(value) = args.get(param_name) {
            // Parameter explicitly provided at call site
            // Value is already fully evaluated by substitute_all() on macro call attributes
            ctx.properties
                .add_to_current_scope(param_name.clone(), value.clone());
        } else {
            // Handle parameter default based on type
            let param_default = macro_def
                .params
                .get(param_name)
                .expect("Internal logic error: parameter in param_order must exist in params map");

            let evaluated = match param_default {
                crate::parse::macro_def::ParamDefault::None => {
                    // No default and not provided -> error
                    return Err(XacroError::MissingParameter {
                        macro_name: macro_def.name.clone(),
                        param: param_name.clone(),
                    });
                }
                crate::parse::macro_def::ParamDefault::Value(default_expr) => {
                    // Evaluate default expression with cumulative context
                    // Earlier parameters are already in the current scope, so they're visible
                    ctx.properties.substitute_text(default_expr)?
                }
                crate::parse::macro_def::ParamDefault::ForwardRequired(forward_name) => {
                    // Forward from parent scope (required)
                    ctx.properties
                        .lookup_at_depth(forward_name, parent_scope_depth)
                        .ok_or_else(|| XacroError::UndefinedPropertyToForward {
                            macro_name: macro_def.name.clone(),
                            param: param_name.clone(),
                            forward_name: forward_name.clone(),
                        })?
                }
                crate::parse::macro_def::ParamDefault::ForwardWithDefault(
                    forward_name,
                    maybe_default,
                ) => {
                    // Try parent scope first, then fall back to default
                    if let Some(parent_value) = ctx
                        .properties
                        .lookup_at_depth(forward_name, parent_scope_depth)
                    {
                        parent_value
                    } else if let Some(default_expr) = maybe_default.as_ref() {
                        ctx.properties.substitute_text(default_expr)?
                    } else {
                        String::new()
                    }
                }
            };

            ctx.properties
                .add_to_current_scope(param_name.clone(), evaluated);
        }
    }

    // Push pre-expanded blocks to block stack (blocks were expanded before entering macro scope)
    ctx.block_stack.borrow_mut().push(expanded_blocks);
    let _block_guard = BlockGuard {
        blocks: &ctx.block_stack,
    };

    // Clone macro content and recursively expand each child
    let content = macro_def.content.clone();

    // Recursively expand all children of the macro
    // Guards will pop scope and block stack on scope exit, even if panic occurs
    expand_children_list(content.children, ctx)
}

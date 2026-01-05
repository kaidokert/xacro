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
    features::{macros::MacroDefinition, properties::PropertyProcessor},
    utils::xml::is_xacro_element,
};
use std::{collections::HashMap, path::PathBuf, rc::Rc};
use xmltree::{Element, XMLNode};

/// Unified context for single-pass expansion
///
/// This struct holds all the state needed during recursive expansion,
/// including properties, macros, and various stacks for tracking scope.
pub struct XacroContext {
    /// Property processor with scope support (from Step 1)
    pub properties: PropertyProcessor,

    /// Macro definitions wrapped in Rc (from Step 2)
    pub macros: HashMap<String, Rc<MacroDefinition>>,

    /// CLI arguments (for future xacro:arg support)
    pub args: HashMap<String, String>,

    /// Include stack for circular include detection
    pub include_stack: Vec<PathBuf>,

    /// Namespace stack: (file_path, xacro_namespace_prefix)
    pub namespace_stack: Vec<(PathBuf, String)>,

    /// Block stack for insert_block arguments
    pub block_stack: Vec<HashMap<String, Element>>,

    /// Current overall recursion depth (for all expand_node calls)
    pub recursion_depth: usize,

    /// Maximum recursion depth before triggering error
    /// Set conservatively to prevent stack overflow before the check triggers
    pub max_recursion_depth: usize,

    /// Base path for resolving relative includes
    pub base_path: PathBuf,
}

impl XacroContext {
    /// Default maximum recursion depth
    /// Set conservatively to prevent stack overflow before the check triggers
    const DEFAULT_MAX_DEPTH: usize = 50;

    /// Create a new context with the given base path
    pub fn new(
        base_path: PathBuf,
        xacro_ns: String,
    ) -> Self {
        Self {
            properties: PropertyProcessor::new(),
            macros: HashMap::new(),
            args: HashMap::new(),
            include_stack: Vec::new(),
            namespace_stack: vec![(base_path.clone(), xacro_ns)],
            block_stack: Vec::new(),
            recursion_depth: 0,
            max_recursion_depth: Self::DEFAULT_MAX_DEPTH,
            base_path,
        }
    }

    /// Set the maximum recursion depth
    pub fn set_max_recursion_depth(
        &mut self,
        depth: usize,
    ) {
        self.max_recursion_depth = depth;
    }

    /// Get current xacro namespace from top of stack
    pub fn current_xacro_ns(&self) -> &str {
        self.namespace_stack
            .last()
            .map(|(_, ns)| ns.as_str())
            .unwrap_or("xacro:")
    }

    /// Look up a named block from the current macro scope
    pub fn lookup_block(
        &self,
        name: &str,
    ) -> Result<&Element, XacroError> {
        self.block_stack
            .last()
            .and_then(|blocks| blocks.get(name))
            .ok_or_else(|| XacroError::UndefinedBlock {
                name: name.to_string(),
            })
    }
}

/// Main recursive expansion function
///
/// This is the core of the single-pass expander. It processes nodes in document
/// order, handling definitions, conditionals, includes, and macro calls.
///
/// Returns Vec<XMLNode> because a single xacro element can expand to:
/// - Zero nodes (property definitions)
/// - One node (regular elements)
/// - Many nodes (macro expansion)
pub fn expand_node(
    node: XMLNode,
    ctx: &mut XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Check recursion depth at entry to catch infinite loops early
    if ctx.recursion_depth > ctx.max_recursion_depth {
        return Err(XacroError::MacroRecursionLimit {
            depth: ctx.recursion_depth,
            limit: ctx.max_recursion_depth,
        });
    }

    // Increment depth for this call
    ctx.recursion_depth += 1;

    let result = match node {
        XMLNode::Element(elem) => expand_element(elem, ctx),
        XMLNode::Text(text) => {
            // Resolve ${...} expressions in text using scope-aware substitution
            let resolved = ctx.properties.substitute_text(&text)?;
            Ok(vec![XMLNode::Text(resolved)])
        }
        other => Ok(vec![other]), // Comments, CDATA, etc. pass through
    };

    // Decrement depth after processing
    ctx.recursion_depth -= 1;

    result
}

/// Expand an element node
fn expand_element(
    mut elem: Element,
    ctx: &mut XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    let xacro_ns = ctx.current_xacro_ns();

    // 1. Property definitions (no output)
    if is_xacro_element(&elem, "property", xacro_ns) {
        // Extract property name (required)
        let name = elem
            .attributes
            .get("name")
            .ok_or_else(|| XacroError::MissingAttribute {
                element: "xacro:property".to_string(),
                attribute: "name".to_string(),
            })?;

        // Get value and default attributes
        let value_attr = elem.attributes.get("value");
        let default_attr = elem.attributes.get("default");

        // Determine what to do based on attributes
        match (value_attr, default_attr) {
            // value always sets the property
            (Some(value), _) => {
                ctx.properties.add_raw_property(name.clone(), value.clone());
            }
            // default only sets if property not already defined
            (None, Some(default_value)) => {
                // Check if property already exists
                if !ctx.properties.has_property(name) {
                    // Property not defined, use default
                    ctx.properties
                        .add_raw_property(name.clone(), default_value.clone());
                }
                // else: property already defined, keep existing value
            }
            // Neither value nor default - error
            (None, None) => {
                return Err(XacroError::MissingAttribute {
                    element: "xacro:property".to_string(),
                    attribute: "value or default".to_string(),
                });
            }
        }

        // Property definitions don't produce output
        return Ok(vec![]);
    }

    // 2. Macro definitions (no output)
    if is_xacro_element(&elem, "macro", xacro_ns) {
        // Extract macro name
        let name = elem
            .attributes
            .get("name")
            .ok_or_else(|| XacroError::MissingAttribute {
                element: "xacro:macro".to_string(),
                attribute: "name".to_string(),
            })?;

        // Parse params attribute (optional - treat missing as empty string)
        let params_str = elem.attributes.get("params").map_or("", |s| s.as_str());
        let (params_map, param_order, block_params_set) =
            crate::features::macros::MacroProcessor::parse_params(params_str)?;

        // Create macro definition
        let macro_def = MacroDefinition {
            name: name.clone(),
            params: params_map,
            param_order,
            block_params: block_params_set,
            content: elem.clone(),
        };

        // Add to context (wrapped in Rc for shared ownership)
        ctx.macros.insert(name.clone(), Rc::new(macro_def));

        // Macro definitions don't produce output
        return Ok(vec![]);
    }

    // 3. Conditionals (LAZY evaluation)
    if is_xacro_element(&elem, "if", xacro_ns) {
        let value = elem
            .attributes
            .get("value")
            .ok_or_else(|| XacroError::MissingAttribute {
                element: "xacro:if".to_string(),
                attribute: "value".to_string(),
            })?;

        // Evaluate condition using scope-aware property resolution
        let condition = ctx.properties.eval_boolean(value)?;

        if condition {
            return expand_children(elem, ctx);
        } else {
            return Ok(vec![]); // Skip false branch - LAZY!
        }
    }

    if is_xacro_element(&elem, "unless", xacro_ns) {
        let value = elem
            .attributes
            .get("value")
            .ok_or_else(|| XacroError::MissingAttribute {
                element: "xacro:unless".to_string(),
                attribute: "value".to_string(),
            })?;

        // Evaluate condition using scope-aware property resolution
        let condition = ctx.properties.eval_boolean(value)?;

        if !condition {
            return expand_children(elem, ctx);
        } else {
            return Ok(vec![]);
        }
    }

    // 4. Includes (LAZY - only evaluated if reached)
    if is_xacro_element(&elem, "include", xacro_ns) {
        let filename =
            elem.attributes
                .get("filename")
                .ok_or_else(|| XacroError::MissingAttribute {
                    element: "xacro:include".to_string(),
                    attribute: "filename".to_string(),
                })?;

        // Resolve path relative to base_path
        let file_path = ctx.base_path.join(filename);

        // Check for circular includes
        if ctx.include_stack.contains(&file_path) {
            return Err(XacroError::Include(format!(
                "Circular include detected: {}",
                file_path.display()
            )));
        }

        // Read and parse included file
        let content = std::fs::read_to_string(&file_path)?;

        let included_root = Element::parse(content.as_bytes())?;

        // Push to include stack
        ctx.include_stack.push(file_path.clone());

        // Update base_path for nested includes in the included file
        let old_base_path = ctx.base_path.clone();
        ctx.base_path = file_path.parent().unwrap_or(&old_base_path).to_path_buf();

        // Recursively expand all children of the included file
        let mut expanded = Vec::new();
        for child in included_root.children {
            expanded.extend(expand_node(child, ctx)?);
        }

        // Restore base_path and pop include stack
        ctx.base_path = old_base_path;
        ctx.include_stack.pop();

        return Ok(expanded);
    }

    // 5. Block insertion
    // Recursion depth is checked at expand_node entry (catches circular block references)
    if is_xacro_element(&elem, "insert_block", xacro_ns) {
        let name = elem
            .attributes
            .get("name")
            .ok_or_else(|| XacroError::MissingAttribute {
                element: "xacro:insert_block".to_string(),
                attribute: "name".to_string(),
            })?;

        let block = ctx.lookup_block(name)?.clone();
        return expand_node(XMLNode::Element(block), ctx);
    }

    // 6. Macro calls (with re-scan for nested macros)
    if is_macro_call(&elem, &ctx.macros, xacro_ns) {
        // CRITICAL: Substitute ${...} expressions in macro call attributes BEFORE expansion
        // This allows nested macros to use parameters from outer macro scope:
        // <xacro:macro name="outer" params="prefix">
        //   <xacro:inner prefix="${prefix}"/>  <!-- ${prefix} evaluated here -->
        // </xacro:macro>
        for attr_value in elem.attributes.values_mut() {
            *attr_value = ctx.properties.substitute_text(attr_value)?;
        }

        let expanded_nodes = expand_macro_call(&elem, ctx)?;
        // Re-scan the expanded result to handle macros that generate macros
        let mut final_nodes = Vec::new();
        for node in expanded_nodes {
            final_nodes.extend(expand_node(node, ctx)?);
        }
        return Ok(final_nodes);
    }

    // 7. Regular elements: Substitute attributes and recurse to children
    // Substitute ${...} expressions in all attributes
    for attr_value in elem.attributes.values_mut() {
        *attr_value = ctx.properties.substitute_text(attr_value)?;
    }

    // Recursively expand children
    let expanded_children = expand_children_list(elem.children, ctx)?;
    elem.children = expanded_children;

    Ok(vec![XMLNode::Element(elem)])
}

/// Expand all children of an element
fn expand_children(
    elem: Element,
    ctx: &mut XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    expand_children_list(elem.children, ctx)
}

/// Expand a list of child nodes
fn expand_children_list(
    children: Vec<XMLNode>,
    ctx: &mut XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    let mut result = Vec::new();
    for child in children {
        result.extend(expand_node(child, ctx)?);
    }
    Ok(result)
}

/// Check if an element is a macro call
fn is_macro_call(
    elem: &Element,
    macros: &HashMap<String, Rc<MacroDefinition>>,
    xacro_ns: &str,
) -> bool {
    // Must be in xacro namespace (check resolved URI, not prefix)
    if elem.namespace.as_deref() != Some(xacro_ns) {
        return false;
    }

    // Known directives are NOT macro calls
    const KNOWN_DIRECTIVES: &[&str] = &[
        "property",
        "macro",
        "arg",
        "include",
        "if",
        "unless",
        "insert_block",
    ];

    // Element name is the local name (without prefix)
    if KNOWN_DIRECTIVES.contains(&elem.name.as_str()) {
        return false;
    }

    // Check if it matches a defined macro
    macros.contains_key(&elem.name)
}

/// Expand a macro call (free function for borrow-safety)
fn expand_macro_call(
    call_elem: &Element,
    ctx: &mut XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Extract macro name (element name is already the local name without prefix)
    let macro_name = &call_elem.name;

    // Look up macro definition
    let macro_def = ctx
        .macros
        .get(macro_name)
        .ok_or_else(|| XacroError::UndefinedMacro(macro_name.to_string()))?;

    // Clone the Rc to get owned reference (cheap pointer copy)
    let macro_def = Rc::clone(macro_def);

    // Collect macro arguments and blocks from call element
    let (args, blocks) =
        crate::features::macros::MacroProcessor::collect_macro_args(call_elem, &macro_def)?;

    // Resolve parameters with defaults, supporting chained defaults
    // Process in declaration order so later defaults can reference earlier ones
    // Example: params="a:=1 b:=${a*2} c:=${b*3}"
    let mut resolved_params = HashMap::new();

    for param_name in &macro_def.param_order {
        if macro_def.block_params.contains(param_name) {
            continue; // Skip block parameters
        }

        if let Some(value) = args.get(param_name) {
            // Parameter explicitly provided at call site
            resolved_params.insert(param_name.clone(), value.clone());
        } else if let Some(default_expr) = macro_def
            .params
            .get(param_name)
            .and_then(|opt| opt.as_ref())
        {
            // Evaluate default expression with cumulative context
            // Push temp scope with previously resolved params so they're visible to this default
            ctx.properties.push_scope(resolved_params.clone());
            let evaluated = ctx.properties.substitute_text(default_expr)?;
            ctx.properties.pop_scope();

            resolved_params.insert(param_name.clone(), evaluated);
        } else {
            // No default and not provided -> error
            return Err(XacroError::MissingParameter {
                macro_name: macro_def.name.clone(),
                param: param_name.clone(),
            });
        }
    }

    // Push macro scope and blocks (RAII guards will pop on drop)
    ctx.properties.push_scope(resolved_params);
    ctx.block_stack.push(blocks);

    // Clone macro content and recursively expand each child
    let content = macro_def.content.clone();

    // Recursively expand all children of the macro
    // The scope and block stack are active, so properties and blocks are available
    let mut expanded = Vec::new();
    let expansion_result = (|| -> Result<Vec<XMLNode>, XacroError> {
        for child in content.children {
            expanded.extend(expand_node(child, ctx)?);
        }
        Ok(expanded)
    })();

    // Clean up: pop scope and blocks (happens even on error)
    ctx.properties.pop_scope();
    ctx.block_stack.pop();

    // Return the result
    expansion_result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_context_creation() {
        let ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        assert_eq!(ctx.current_xacro_ns(), "http://www.ros.org/wiki/xacro");
        assert_eq!(ctx.include_stack.len(), 0);
        assert_eq!(ctx.block_stack.len(), 0);
    }

    #[test]
    fn test_expand_text_node() {
        let mut ctx = XacroContext::new(
            PathBuf::from("/test"),
            "http://www.ros.org/wiki/xacro".to_string(),
        );

        // Set up a test property
        ctx.properties
            .add_raw_property("x".to_string(), "42".to_string());

        let text_node = XMLNode::Text("value: ${x}".to_string());
        let result = expand_node(text_node, &mut ctx).unwrap();

        assert_eq!(result.len(), 1);
        match &result[0] {
            XMLNode::Text(t) => assert_eq!(t, "value: 42"),
            _ => panic!("Expected text node"),
        }
    }
}

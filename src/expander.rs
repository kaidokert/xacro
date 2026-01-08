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
    utils::xml::extract_xacro_namespace,
    utils::xml::is_xacro_element,
};
use core::cell::RefCell;
use std::{collections::HashMap, path::PathBuf, rc::Rc};
use xmltree::{Element, XMLNode};

// ============================================================================
// RAII Guards for Panic-Safe Stack Management
// ============================================================================
// These guards ensure that pushed state (scopes, stacks, depth counters) is
// always popped/restored when the guard goes out of scope, even if a panic
// occurs during expansion. This prevents the XacroContext from being left in
// a corrupted state.

/// RAII guard for recursion depth tracking
///
/// Automatically decrements recursion_depth when dropped, ensuring correct
/// depth tracking even if expansion panics.
struct DepthGuard<'a> {
    depth: &'a RefCell<usize>,
}

impl Drop for DepthGuard<'_> {
    fn drop(&mut self) {
        *self.depth.borrow_mut() -= 1;
    }
}

/// RAII guard for include stack, base path, and namespace stack
///
/// Automatically restores base_path and pops include_stack and namespace_stack when dropped,
/// ensuring correct file context and per-file namespace isolation even if include expansion panics.
struct IncludeGuard<'a> {
    base_path: &'a RefCell<PathBuf>,
    include_stack: &'a RefCell<Vec<PathBuf>>,
    namespace_stack: &'a RefCell<Vec<(PathBuf, String)>>,
    old_base_path: PathBuf,
}

impl Drop for IncludeGuard<'_> {
    fn drop(&mut self) {
        *self.base_path.borrow_mut() = self.old_base_path.clone();
        self.include_stack.borrow_mut().pop();
        self.namespace_stack.borrow_mut().pop();
    }
}

/// RAII guard for property scopes
///
/// Automatically pops property scope when dropped, ensuring correct variable
/// shadowing even if macro expansion panics.
struct ScopeGuard<'a> {
    properties: &'a PropertyProcessor,
}

impl Drop for ScopeGuard<'_> {
    fn drop(&mut self) {
        self.properties.pop_scope();
    }
}

/// RAII guard for block stacks
///
/// Automatically pops block stack when dropped, ensuring correct block
/// resolution even if macro expansion panics.
struct BlockGuard<'a> {
    blocks: &'a RefCell<Vec<HashMap<String, Vec<XMLNode>>>>,
}

impl Drop for BlockGuard<'_> {
    fn drop(&mut self) {
        self.blocks.borrow_mut().pop();
    }
}

// ============================================================================

/// Unified context for single-pass expansion
///
/// This struct holds all the state needed during recursive expansion,
/// including properties, macros, and various stacks for tracking scope.
pub struct XacroContext {
    /// Property processor with scope support (from Step 1)
    pub properties: PropertyProcessor,

    /// Macro definitions wrapped in Rc (from Step 2) - uses RefCell for interior mutability
    pub macros: RefCell<HashMap<String, Rc<MacroDefinition>>>,

    /// CLI arguments (shared with PropertyProcessor for $(arg) resolution)
    /// Wrapped in Rc<RefCell<...>> for shared mutable access
    pub args: Rc<RefCell<HashMap<String, String>>>,

    /// Include stack for circular include detection (uses RefCell for interior mutability)
    pub include_stack: RefCell<Vec<PathBuf>>,

    /// Namespace stack: (file_path, xacro_namespace_prefix) (uses RefCell for interior mutability)
    pub namespace_stack: RefCell<Vec<(PathBuf, String)>>,

    /// Block stack for insert_block arguments (uses RefCell for interior mutability)
    /// Stores pre-expanded XMLNode content for each block parameter
    pub block_stack: RefCell<Vec<HashMap<String, Vec<XMLNode>>>>,

    /// Current base path for resolving relative includes (uses RefCell for interior mutability)
    pub base_path: RefCell<PathBuf>,

    /// Current overall recursion depth (uses RefCell for interior mutability to enable RAII guards)
    pub recursion_depth: RefCell<usize>,

    /// Maximum recursion depth before triggering error
    /// Set conservatively to prevent stack overflow before the check triggers
    pub max_recursion_depth: usize,
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
        Self::new_with_args(base_path, xacro_ns, HashMap::new())
    }

    /// Create a new context with CLI arguments
    ///
    /// # Arguments
    /// * `base_path` - Base directory for resolving relative includes
    /// * `xacro_ns` - Xacro namespace prefix (e.g., "xacro")
    /// * `cli_args` - Arguments from CLI (take precedence over XML defaults)
    pub fn new_with_args(
        base_path: PathBuf,
        xacro_ns: String,
        cli_args: HashMap<String, String>,
    ) -> Self {
        // Create shared args map for both PropertyProcessor and expander
        // Initialize with CLI args (these take precedence over XML defaults)
        let args = Rc::new(RefCell::new(cli_args));

        Self {
            properties: PropertyProcessor::new_with_args(args.clone()),
            macros: RefCell::new(HashMap::new()),
            args,
            include_stack: RefCell::new(Vec::new()),
            namespace_stack: RefCell::new(vec![(base_path.clone(), xacro_ns)]),
            block_stack: RefCell::new(Vec::new()),
            base_path: RefCell::new(base_path),
            recursion_depth: RefCell::new(0),
            max_recursion_depth: Self::DEFAULT_MAX_DEPTH,
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
    pub fn current_xacro_ns(&self) -> String {
        self.namespace_stack
            .borrow()
            .last()
            .map(|(_, ns)| ns.clone())
            .expect("namespace_stack should never be empty - initialized in XacroContext::new()")
    }

    /// Look up a named block from the current macro scope
    /// Returns pre-expanded XMLNodes
    pub fn lookup_block(
        &self,
        name: &str,
    ) -> Result<Vec<XMLNode>, XacroError> {
        self.block_stack
            .borrow()
            .last()
            .and_then(|blocks| blocks.get(name).cloned())
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
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Check recursion depth at entry to catch infinite loops early
    if *ctx.recursion_depth.borrow() > ctx.max_recursion_depth {
        return Err(XacroError::MacroRecursionLimit {
            depth: *ctx.recursion_depth.borrow(),
            limit: ctx.max_recursion_depth,
        });
    }

    // Increment depth for this call and create RAII guard for automatic decrement
    *ctx.recursion_depth.borrow_mut() += 1;
    let _depth_guard = DepthGuard {
        depth: &ctx.recursion_depth,
    };

    // Process node (guard will decrement depth on scope exit, even if panic occurs)
    match node {
        XMLNode::Element(elem) => expand_element(elem, ctx),
        XMLNode::Text(text) => {
            // Resolve both ${...} expressions and $(...) extensions in text
            let resolved = ctx.properties.substitute_all(&text)?;
            Ok(vec![XMLNode::Text(resolved)])
        }
        other => Ok(vec![other]), // Comments, CDATA, etc. pass through
    }
}

/// Expand an element node
fn expand_element(
    mut elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    let xacro_ns = ctx.current_xacro_ns();

    // 1. Property definitions (no output)
    if is_xacro_element(&elem, "property", &xacro_ns) {
        // Extract property name (required) and substitute expressions
        let name = ctx
            .properties
            .substitute_text(elem.attributes.get("name").ok_or_else(|| {
                XacroError::MissingAttribute {
                    element: "xacro:property".to_string(),
                    attribute: "name".to_string(),
                }
            })?)?;

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
                if !ctx.properties.has_property(&name) {
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

    // 1b. Argument definitions (no output)
    if is_xacro_element(&elem, "arg", &xacro_ns) {
        // Extract raw name attribute (required)
        let raw_name = elem
            .attributes
            .get("name")
            .ok_or_else(|| XacroError::MissingAttribute {
                element: "xacro:arg".to_string(),
                attribute: "name".to_string(),
            })?;

        // CRITICAL: Evaluate name with properties only (no extensions in arg names)
        // This prevents circular dependencies: $(arg ${x}) where x="..."
        let name = ctx.properties.substitute_text(raw_name)?;

        // CLI arguments take precedence over defaults (The "Precedence Rake")
        if !ctx.args.borrow().contains_key(&name) {
            // Only set default if CLI didn't provide a value
            if let Some(default_value) = elem.attributes.get("default") {
                // CRITICAL: Evaluate default with FULL substitution (may contain $(arg ...))
                // This enables transitive defaults: <xacro:arg name="y" default="$(arg x)"/>
                let default = ctx.properties.substitute_all(default_value)?;
                ctx.args.borrow_mut().insert(name.clone(), default);
            }
            // else: No default and no CLI value provided
            // Don't insert anything - let it error with UndefinedArgument on first use
            // This allows args to be declared without defaults, but requires CLI value
        }

        // The directive consumes itself (doesn't appear in output)
        return Ok(vec![]);
    }

    // 2. Macro definitions (no output)
    if is_xacro_element(&elem, "macro", &xacro_ns) {
        // Extract macro name and substitute expressions
        let name = ctx
            .properties
            .substitute_text(elem.attributes.get("name").ok_or_else(|| {
                XacroError::MissingAttribute {
                    element: "xacro:macro".to_string(),
                    attribute: "name".to_string(),
                }
            })?)?;

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
        ctx.macros
            .borrow_mut()
            .insert(name.clone(), Rc::new(macro_def));

        // Macro definitions don't produce output
        return Ok(vec![]);
    }

    // 3. Conditionals (LAZY evaluation) - consolidated if/unless
    let is_if = is_xacro_element(&elem, "if", &xacro_ns);
    let is_unless = is_xacro_element(&elem, "unless", &xacro_ns);

    if is_if || is_unless {
        let tag_name = if is_if { "xacro:if" } else { "xacro:unless" };

        let value = elem
            .attributes
            .get("value")
            .ok_or_else(|| XacroError::MissingAttribute {
                element: tag_name.to_string(),
                attribute: "value".to_string(),
            })?;

        // Evaluate condition using scope-aware property resolution
        let condition = ctx.properties.eval_boolean(value)?;

        // For 'if': expand if true; for 'unless': expand if false
        let should_expand = if is_if { condition } else { !condition };

        if should_expand {
            return expand_children(elem, ctx);
        } else {
            return Ok(vec![]); // Skip branch - LAZY!
        }
    }

    // 4. Includes (LAZY - only evaluated if reached)
    if is_xacro_element(&elem, "include", &xacro_ns) {
        // Extract filename and substitute expressions
        let filename =
            ctx.properties
                .substitute_text(elem.attributes.get("filename").ok_or_else(|| {
                    XacroError::MissingAttribute {
                        element: "xacro:include".to_string(),
                        attribute: "filename".to_string(),
                    }
                })?)?;

        // Resolve path relative to base_path
        let file_path = ctx.base_path.borrow().join(&filename);

        // Check for circular includes
        if ctx.include_stack.borrow().contains(&file_path) {
            return Err(XacroError::Include(format!(
                "Circular include detected: {}",
                file_path.display()
            )));
        }

        // Read and parse included file
        let content = std::fs::read_to_string(&file_path).map_err(|e| {
            XacroError::Include(format!(
                "Failed to read file '{}': {}",
                file_path.display(),
                e
            ))
        })?;

        let included_root = Element::parse(content.as_bytes()).map_err(|e| {
            XacroError::Include(format!(
                "Failed to parse XML in file '{}': {}",
                file_path.display(),
                e
            ))
        })?;

        // Extract xacro namespace from included file (may differ from root file)
        let included_ns = extract_xacro_namespace(&included_root)?;

        // Push to include stack, namespace stack, and update base_path with RAII guard for automatic cleanup
        let old_base_path = ctx.base_path.borrow().clone();
        // Use PathBuf::pop() to get parent directory - handles root directories correctly
        // (unlike parent().unwrap_or() which would incorrectly fall back to old_base_path at root)
        let mut new_base_path = file_path.clone();
        new_base_path.pop(); // Removes filename, leaving parent directory (or root if already at root)
        *ctx.base_path.borrow_mut() = new_base_path;
        ctx.include_stack.borrow_mut().push(file_path.clone());
        ctx.namespace_stack
            .borrow_mut()
            .push((file_path.clone(), included_ns));

        let _include_guard = IncludeGuard {
            base_path: &ctx.base_path,
            include_stack: &ctx.include_stack,
            namespace_stack: &ctx.namespace_stack,
            old_base_path,
        };

        // Recursively expand all children of the included file
        // Guard will restore base_path and pop include_stack and namespace_stack on scope exit, even if panic occurs
        return expand_children_list(included_root.children, ctx);
    }

    // 5. Block insertion
    // Recursion depth is checked at expand_node entry (catches circular block references)
    if is_xacro_element(&elem, "insert_block", &xacro_ns) {
        // Extract block name and substitute expressions
        let name = ctx
            .properties
            .substitute_text(elem.attributes.get("name").ok_or_else(|| {
                XacroError::MissingAttribute {
                    element: "xacro:insert_block".to_string(),
                    attribute: "name".to_string(),
                }
            })?)?;

        // Retrieve and return the pre-expanded block content
        let nodes = ctx.lookup_block(&name)?;
        return Ok(nodes);
    }

    // 6. Check for known but unimplemented xacro directives
    // These should error explicitly rather than silently pass through as literal XML
    use crate::error::{IMPLEMENTED_FEATURES, UNIMPLEMENTED_FEATURES};
    for feature in UNIMPLEMENTED_FEATURES {
        // Strip "xacro:" prefix to get element name
        let directive = feature.strip_prefix("xacro:").unwrap_or(feature);
        if is_xacro_element(&elem, directive, &xacro_ns) {
            return Err(XacroError::UnimplementedFeature(format!(
                "<xacro:{}>\n\
                 This element was not processed. Either:\n\
                 1. The feature is not implemented yet (known unimplemented: {})\n\
                 2. There's a bug in the processor\n\
                 \n\
                 Currently implemented: {}",
                elem.name,
                UNIMPLEMENTED_FEATURES.join(", "),
                IMPLEMENTED_FEATURES.join(", ")
            )));
        }
    }

    // 7. Macro calls (with re-scan for nested macros)
    if is_macro_call(&elem, &ctx.macros.borrow(), &xacro_ns) {
        // CRITICAL: Substitute both ${...} and $(...) in macro call attributes BEFORE expansion
        // This allows nested macros to use parameters from outer macro scope:
        // <xacro:macro name="outer" params="prefix">
        //   <xacro:inner prefix="${prefix}"/>  <!-- ${prefix} evaluated here -->
        // </xacro:macro>
        for attr_value in elem.attributes.values_mut() {
            *attr_value = ctx.properties.substitute_all(attr_value)?;
        }

        let expanded_nodes = expand_macro_call(&elem, ctx)?;
        // Re-scan the expanded result to handle macros that generate macros
        return expand_children_list(expanded_nodes, ctx);
    }

    // 8. Regular elements: Substitute attributes and recurse to children
    // Substitute both ${...} expressions and $(...) extensions in all attributes
    for attr_value in elem.attributes.values_mut() {
        *attr_value = ctx.properties.substitute_all(attr_value)?;
    }

    // Recursively expand children
    let expanded_children = expand_children_list(elem.children, ctx)?;
    elem.children = expanded_children;

    Ok(vec![XMLNode::Element(elem)])
}

/// Expand all children of an element
fn expand_children(
    elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    expand_children_list(elem.children, ctx)
}

/// Expand a list of child nodes
fn expand_children_list(
    children: Vec<XMLNode>,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    children.into_iter().try_fold(Vec::new(), |mut acc, child| {
        acc.extend(expand_node(child, ctx)?);
        Ok::<Vec<XMLNode>, XacroError>(acc)
    })
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

    // Known directives (implemented and unimplemented) are NOT macro calls
    // Use single source of truth from features/mod.rs to prevent maintenance hazards
    use crate::features::{IMPLEMENTED_DIRECTIVES, UNIMPLEMENTED_DIRECTIVES};

    // Element name is the local name (without prefix)
    let elem_name = elem.name.as_str();
    if IMPLEMENTED_DIRECTIVES.contains(&elem_name) || UNIMPLEMENTED_DIRECTIVES.contains(&elem_name)
    {
        return false;
    }

    // Check if it matches a defined macro
    macros.contains_key(&elem.name)
}

/// Expand a macro call (free function for borrow-safety)
fn expand_macro_call(
    call_elem: &Element,
    ctx: &XacroContext,
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

    // Collect macro arguments and blocks from call element
    let (args, blocks) =
        crate::features::macros::MacroProcessor::collect_macro_args(call_elem, &macro_def)?;

    // Pre-expand blocks in caller's scope before entering macro scope
    // CRITICAL: Must happen BEFORE pushing macro's parameter scope to ensure
    // block content is evaluated with caller's properties, not macro's parameters
    let mut expanded_blocks = HashMap::new();
    for (name, raw_block) in blocks {
        // Wrap element in XMLNode and expand it in the current scope
        let expanded = expand_node(XMLNode::Element(raw_block), ctx)?;
        expanded_blocks.insert(name, expanded);
    }

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
            ctx.properties
                .add_to_current_scope(param_name.clone(), value.clone());
        } else if let Some(default_expr) = macro_def
            .params
            .get(param_name)
            .and_then(|opt| opt.as_ref())
        {
            // Evaluate default expression with cumulative context
            // Earlier parameters are already in the current scope, so they're visible
            let evaluated = ctx.properties.substitute_text(default_expr)?;
            ctx.properties
                .add_to_current_scope(param_name.clone(), evaluated);
        } else {
            // No default and not provided -> error
            // Guard will clean up the scope
            return Err(XacroError::MissingParameter {
                macro_name: macro_def.name.clone(),
                param: param_name.clone(),
            });
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
        assert_eq!(text, "value: 42.0");
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
        assert_eq!(text, "sum: 30.0, product: 200.0");
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

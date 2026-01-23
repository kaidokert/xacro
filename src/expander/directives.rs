//! Directive handlers for xacro elements
//!
//! This module handles all xacro directive processing:
//! - xacro:property - Property definitions with scope support
//! - xacro:arg - Command-line argument definitions
//! - xacro:macro - Macro definitions
//! - xacro:if / xacro:unless - Conditional expansion
//! - xacro:insert_block - Block parameter insertion

use crate::{
    error::{EnrichError, XacroError, IMPLEMENTED_FEATURES, UNIMPLEMENTED_FEATURES},
    eval::PropertyScope,
    parse::{macro_def::MacroDefinition, macro_def::MacroProcessor},
};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::OnceLock;
use xmltree::{Element, XMLNode};

use super::{expand_children_list, XacroContext};

// =============================================================================
// DirectiveHandler Trait and Registry
// =============================================================================

/// Trait for directive handlers that process xacro elements
///
/// This enables extensible directive dispatch via a registry pattern.
/// Each directive (property, arg, macro, if, etc.) implements this trait.
pub(super) trait DirectiveHandler: Send + Sync {
    /// Handle the directive element
    ///
    /// # Arguments
    /// * `elem` - The xacro directive element to process
    /// * `ctx` - XacroContext with properties, macros, and stacks
    ///
    /// # Returns
    /// Expanded nodes (may be empty for definitions like property/arg/macro)
    fn handle(
        &self,
        elem: Element,
        ctx: &XacroContext,
    ) -> Result<Vec<XMLNode>, XacroError>;
}

/// Global directive handler registry (initialized once on first access)
static DIRECTIVE_REGISTRY: OnceLock<HashMap<&'static str, Box<dyn DirectiveHandler>>> =
    OnceLock::new();

/// Get the directive handler registry
///
/// Lazily initializes the registry on first call, then returns cached reference.
/// Uses OnceLock for thread-safe lazy initialization without runtime overhead.
pub(super) fn get_directive_registry() -> &'static HashMap<&'static str, Box<dyn DirectiveHandler>>
{
    DIRECTIVE_REGISTRY.get_or_init(|| {
        let mut registry: HashMap<&'static str, Box<dyn DirectiveHandler>> = HashMap::new();

        // Register all built-in directive handlers
        registry.insert("property", Box::new(PropertyDirective));
        registry.insert("arg", Box::new(ArgDirective));
        registry.insert("macro", Box::new(MacroDirective));
        registry.insert("if", Box::new(IfDirective));
        registry.insert("unless", Box::new(UnlessDirective));
        registry.insert("insert_block", Box::new(InsertBlockDirective));

        registry
    })
}

// =============================================================================
// Directive Handler Implementations
// =============================================================================

/// Property directive handler
struct PropertyDirective;

impl DirectiveHandler for PropertyDirective {
    fn handle(
        &self,
        elem: Element,
        ctx: &XacroContext,
    ) -> Result<Vec<XMLNode>, XacroError> {
        handle_property_directive(elem, ctx)
    }
}

/// Arg directive handler
struct ArgDirective;

impl DirectiveHandler for ArgDirective {
    fn handle(
        &self,
        elem: Element,
        ctx: &XacroContext,
    ) -> Result<Vec<XMLNode>, XacroError> {
        handle_arg_directive(elem, ctx)
    }
}

/// Macro directive handler
struct MacroDirective;

impl DirectiveHandler for MacroDirective {
    fn handle(
        &self,
        elem: Element,
        ctx: &XacroContext,
    ) -> Result<Vec<XMLNode>, XacroError> {
        handle_macro_directive(elem, ctx)
    }
}

/// If directive handler
struct IfDirective;

impl DirectiveHandler for IfDirective {
    fn handle(
        &self,
        elem: Element,
        ctx: &XacroContext,
    ) -> Result<Vec<XMLNode>, XacroError> {
        handle_conditional_directive(elem, ctx, true)
    }
}

/// Unless directive handler
struct UnlessDirective;

impl DirectiveHandler for UnlessDirective {
    fn handle(
        &self,
        elem: Element,
        ctx: &XacroContext,
    ) -> Result<Vec<XMLNode>, XacroError> {
        handle_conditional_directive(elem, ctx, false)
    }
}

/// Insert block directive handler
struct InsertBlockDirective;

impl DirectiveHandler for InsertBlockDirective {
    fn handle(
        &self,
        elem: Element,
        ctx: &XacroContext,
    ) -> Result<Vec<XMLNode>, XacroError> {
        handle_insert_block_directive(elem, ctx)
    }
}

// =============================================================================
// Original Function-Based Handlers (kept for backward compatibility)
// =============================================================================

/// Handle xacro:property directive
///
/// Supports:
/// - value attribute: Always sets property
/// - default attribute: Sets only if not already defined
/// - Lazy properties: XML body content (stored with '**' prefix)
/// - Scope attribute: local (default), parent, global
///
/// # Returns
/// Empty vec (property definitions produce no output)
pub(super) fn handle_property_directive(
    elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Extract property name (required) and substitute expressions
    let loc = ctx.get_location_context();
    let name = ctx
        .properties
        .substitute_text(
            elem.get_attribute("name")
                .ok_or_else(|| XacroError::MissingAttribute {
                    element: "xacro:property".to_string(),
                    attribute: "name".to_string(),
                })?,
            Some(&loc),
        )
        .with_loc(&loc)?;

    // Parse scope attribute (default: local)
    let scope = match elem.get_attribute("scope").map(|s| s.as_str()) {
        None => PropertyScope::Local, // Default
        Some("parent") => PropertyScope::Parent,
        Some("global") => PropertyScope::Global,
        Some(other) => {
            return Err(XacroError::InvalidScopeAttribute {
                property: name.clone(),
                scope: other.to_string(),
            })
        }
    };

    // Get value and default attributes
    let value_attr = elem.get_attribute("value");
    let default_attr = elem.get_attribute("default");

    // Helper closure to define property with scope-aware evaluation
    let define_property = |raw_value: String| -> Result<(), XacroError> {
        // ALWAYS use define_property - it handles all scopes correctly
        // Local properties use lazy evaluation (raw_value as-is)
        // Parent/Global properties use eager evaluation (substitute first)
        if scope == PropertyScope::Local {
            ctx.properties
                .define_property(name.clone(), raw_value, scope);
        } else {
            // Eagerly evaluate for parent/global scope (both ${...} and $(...))
            let evaluated = ctx
                .properties
                .substitute_all(&raw_value, Some(&loc))
                .with_loc(&loc)?;
            ctx.properties
                .define_property(name.clone(), evaluated, scope);
        }
        Ok(())
    };

    // Determine what to do based on attributes
    match (value_attr, default_attr) {
        // value always sets the property
        (Some(value), _) => {
            define_property(value.clone())?;
        }
        // default only sets if property not already defined
        (None, Some(default_value)) => {
            // Check if property already exists in target scope
            if !ctx.properties.has_property_in_scope(&name, scope) {
                define_property(default_value.clone())?;
            }
            // else: property already defined in target scope, keep existing value
        }
        // Neither value nor default - check for lazy property (body-based)
        (None, None) => {
            // Rule: Text-only properties are NOT created (matches Python xacro)
            // Valid if: (empty) OR (has at least one Element, Comment, CDATA, or PI child)
            let has_content = crate::parse::xml::has_structural_content(&elem.children);
            let is_text_only = !elem.children.is_empty() && !has_content;

            if is_text_only {
                // Text-only or whitespace-only - skip definition (matches Python xacro)
                return Ok(vec![]);
            }

            // Create lazy property (empty or has structural content)
            // Prefix with '**' to distinguish from regular value properties (matches Python xacro)
            let lazy_name = format!("**{}", name);

            // For non-local scopes, eagerly expand children before storing
            if scope == PropertyScope::Local {
                // Serialize children to raw XML string (NO substitution yet - lazy evaluation)
                let content = crate::parse::xml::serialize_nodes(&elem.children)?;
                ctx.properties.define_property(lazy_name, content, scope);
            } else {
                // Eagerly expand children for parent/global scope
                let expanded_nodes = expand_children_list(elem.children, ctx)?;
                let content = crate::parse::xml::serialize_nodes(&expanded_nodes)?;
                ctx.properties.define_property(lazy_name, content, scope);
            }
        }
    }

    // Property definitions don't produce output
    Ok(vec![])
}

/// Handle xacro:arg directive
///
/// Defines command-line arguments with optional defaults.
/// CLI arguments always take precedence over defaults.
///
/// # Returns
/// Empty vec (arg definitions produce no output)
pub(super) fn handle_arg_directive(
    elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Extract raw name attribute (required)
    let raw_name = elem
        .get_attribute("name")
        .ok_or_else(|| XacroError::MissingAttribute {
            element: "xacro:arg".to_string(),
            attribute: "name".to_string(),
        })?;

    // Evaluate name with properties only (no extensions in arg names)
    // This prevents circular dependencies: $(arg ${x}) where x="..."
    let loc = ctx.get_location_context();
    let name = ctx
        .properties
        .substitute_text(raw_name, Some(&loc))
        .with_loc(&loc)?;

    // CLI arguments take precedence over defaults (The "Precedence Rake")
    // Check if CLI provided a value BEFORE evaluating default expression
    // (avoids errors in unused default expressions with undefined references)
    if !ctx.args.borrow().contains_key(&name) {
        // Only set default if CLI didn't provide a value
        if let Some(default_value) = elem.get_attribute("default") {
            // Evaluate default with FULL substitution (may contain $(arg ...))
            // This enables transitive defaults: <xacro:arg name="y" default="$(arg x)"/>
            let default = ctx
                .properties
                .substitute_all(default_value, Some(&loc))
                .with_loc(&loc)?;
            ctx.args.borrow_mut().insert(name.clone(), default);
        }
        // else: No default and no CLI value provided
        // Don't insert anything - let it error with UndefinedArgument on first use
        // This allows args to be declared without defaults, but requires CLI value
    }

    // The directive consumes itself (doesn't appear in output)
    Ok(vec![])
}

/// Handle xacro:macro directive
///
/// Defines a macro with parameters and block parameters.
/// Macro names are NOT evaluated during definition.
///
/// # Returns
/// Empty vec (macro definitions produce no output)
pub(super) fn handle_macro_directive(
    elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Extract macro name (raw, no substitution)
    // Python xacro does NOT evaluate expressions in macro names during definition
    // This allows names like "${ns}/box_inertia" where ns is undefined at definition time
    let name = elem
        .get_attribute("name")
        .ok_or_else(|| XacroError::MissingAttribute {
            element: "xacro:macro".to_string(),
            attribute: "name".to_string(),
        })?
        .to_string();

    // Parse params attribute (optional - treat missing as empty string)
    let params_str = elem.get_attribute("params").map_or("", |s| s.as_str());
    let (params_map, param_order, block_params_set, lazy_block_params_set) =
        if ctx.compat_mode.duplicate_params {
            MacroProcessor::parse_params_compat(params_str)?
        } else {
            MacroProcessor::parse_params(params_str)?
        };

    // Create macro definition
    let macro_def = MacroDefinition {
        name: name.clone(),
        params: params_map,
        param_order,
        block_params: block_params_set,
        lazy_block_params: lazy_block_params_set,
        content: elem,
    };

    // Add to context (wrapped in Rc for shared ownership)
    ctx.macros
        .borrow_mut()
        .insert(name.clone(), Rc::new(macro_def));

    // Macro definitions don't produce output
    Ok(vec![])
}

/// Handle xacro:if and xacro:unless directives
///
/// Conditionally expands children based on boolean value attribute.
/// - xacro:if: expands if value is true
/// - xacro:unless: expands if value is false
///
/// # Arguments
/// * `elem` - The conditional element
/// * `ctx` - XacroContext
/// * `is_if` - true for xacro:if, false for xacro:unless
///
/// # Returns
/// Expanded children if condition met, empty vec otherwise
pub(super) fn handle_conditional_directive(
    elem: Element,
    ctx: &XacroContext,
    is_if: bool,
) -> Result<Vec<XMLNode>, XacroError> {
    let tag_name = if is_if { "xacro:if" } else { "xacro:unless" };

    let value = elem
        .get_attribute("value")
        .ok_or_else(|| XacroError::MissingAttribute {
            element: tag_name.to_string(),
            attribute: "value".to_string(),
        })?;

    // Evaluate condition using scope-aware property resolution with location context
    let loc = ctx.get_location_context();
    let condition = ctx
        .properties
        .eval_boolean(value, Some(&loc))
        .with_loc(&loc)?;

    // For 'if': expand if true; for 'unless': expand if false
    let should_expand = if is_if { condition } else { !condition };

    if should_expand {
        expand_children_list(elem.children, ctx)
    } else {
        Ok(vec![]) // Skip branch - LAZY!
    }
}

/// Handle xacro:insert_block directive
///
/// Inserts a block parameter or lazy property.
/// Precedence: lazy properties (XML body) FIRST, then block parameters.
///
/// # Returns
/// Expanded nodes from the block/property
pub(super) fn handle_insert_block_directive(
    elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Extract block name and substitute expressions
    let loc = ctx.get_location_context();
    let name = ctx
        .properties
        .substitute_text(
            elem.get_attribute("name")
                .ok_or_else(|| XacroError::MissingAttribute {
                    element: "xacro:insert_block".to_string(),
                    attribute: "name".to_string(),
                })?,
            Some(&loc),
        )
        .with_loc(&loc)?;

    // PRECEDENCE ORDER (matches Python xacro behavior):
    // 1. LAZY properties FIRST (properties with XML body content)
    //    Lazy properties are stored with '**' prefix (matches Python xacro)
    //    Search all scopes (local to global) for lazy properties
    let lazy_name = format!("**{}", name);
    if let Some(raw_value) = ctx.properties.lookup_raw_value(&lazy_name) {
        // Found lazy property - parse and expand
        match crate::parse::xml::parse_xml_fragment(&raw_value) {
            Ok(nodes) => {
                let expanded = expand_children_list(nodes, ctx)?;
                return Ok(expanded);
            }
            Err(e) => {
                // Lazy properties are written by this code path and should always be valid XML.
                // A parse failure here likely indicates a bug or data corruption.
                log::error!(
                    "Failed to parse stored lazy property '{}' as XML fragment: {}. Raw value: '{}'",
                    name, e, raw_value
                );
                // Return error instead of silently falling through
                return Err(XacroError::InvalidXml(format!(
                    "Corrupted lazy property '{}': failed to parse stored XML fragment: {}",
                    name, e
                )));
            }
        }
    }

    // 2. Block stack SECOND (macro block parameters)
    // This searches all parent scopes
    // If not found, propagate the UndefinedBlock error
    ctx.lookup_block(&name)
}

/// Check if element is an unimplemented directive and error
///
/// Explicitly errors for known but unimplemented features.
///
/// # Returns
/// Ok(()) if not an unimplemented directive
/// Err if it matches an unimplemented feature
pub(super) fn check_unimplemented_directive(
    elem: &Element,
    xacro_ns: &str,
) -> Result<(), XacroError> {
    for feature in UNIMPLEMENTED_FEATURES {
        // Strip "xacro:" prefix to get element name
        let directive = feature.strip_prefix("xacro:").unwrap_or(feature);
        if crate::parse::xml::is_xacro_element(elem, directive, xacro_ns) {
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
    Ok(())
}

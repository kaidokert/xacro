//! Property evaluation and scope management for xacro expressions
//!
//! This module implements Python xacro's lazy property evaluation semantics,
//! where properties are stored as raw strings and only evaluated when accessed.
//! This allows forward references and avoids errors for unused properties.
//!
//! # Architecture
//!
//! The module is split into focused submodules:
//! - `metadata`: Property metadata tracking for Python-compatible formatting
//! - `resolution`: Lazy property resolution with circular dependency detection
//! - `scope_stack`: Scope push/pop/lookup for macro parameter shadowing
//! - `substitution`: Iterative substitution engine for `${...}` expressions and `$(...)` extensions
//! - `util`: Helper functions for string manipulation and identifier extraction
//!
//! # Core Type
//!
//! `EvalContext` is the central type that manages:
//! - Property storage (raw and cached)
//! - Scope stack for macro parameters
//! - Extension handlers for `$(command args)`
//! - Circular dependency detection
//!
//! # Usage
//!
//! ```ignore
//! use xacro_rs::eval::scope::EvalContext;
//!
//! let ctx = EvalContext::new();
//! ctx.add_raw_property("width".to_string(), "100".to_string());
//! ctx.add_raw_property("double_width".to_string(), "${width * 2}".to_string());
//!
//! let result = ctx.substitute_text("Width is ${width}, double is ${double_width}")?;
//! // Result: "Width is 100, double is 200"
//! ```

mod metadata;
mod resolution;
mod scope_stack;
mod substitution;
mod util;

#[cfg(feature = "compat")]
use self::metadata::PropertyMetadata;
use self::util::*;
use super::interpreter::constants::BUILTIN_CONSTANTS;
#[cfg(test)]
use crate::extensions::core::default_extensions;
use crate::extensions::ExtensionHandler;
use core::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// Where to define a property
///
/// Used by `<xacro:property scope="...">` to control which scope receives the definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PropertyScope {
    /// Default: Define in current scope (top of stack or global if no stack)
    Local,
    /// scope="parent": Define in parent scope (one level up from current)
    Parent,
    /// scope="global": Define in global scope (raw_properties)
    Global,
}

pub(crate) struct EvalContext<const MAX_SUBSTITUTION_DEPTH: usize = 100> {
    // Lazy evaluation infrastructure for Python xacro compatibility
    // Store raw, unevaluated property values: "x" -> "${y * 2}"
    raw_properties: RefCell<HashMap<String, String>>,
    // Cache fully evaluated values: "x" -> "20"
    // NOTE: Cache is only used when scope_stack is empty (global scope)
    evaluated_cache: RefCell<HashMap<String, String>>,
    // Stack for circular dependency detection: ["x", "y"]
    resolution_stack: RefCell<Vec<String>>,
    // Stack of scopes for macro parameter shadowing
    // Each scope is a HashMap of parameter bindings for a macro call
    // Innermost scope is at the end of the vector
    scope_stack: RefCell<Vec<HashMap<String, String>>>,
    // Shared reference to arguments map (injected from XacroContext)
    // Args are separate from properties and follow CLI-precedence semantics
    args: Rc<RefCell<HashMap<String, String>>>,
    // Extension handlers for $(command args...) resolution
    // Handlers are called in order until one returns Some(result)
    // Wrapped in Rc for efficient sharing without cloning
    extensions: Rc<Vec<Box<dyn ExtensionHandler>>>,
    // YAML tag handler registry for custom YAML tags (e.g., !degrees, !millimeters)
    // Handlers are injected at runtime, not compile-time feature-gated
    // Wrapped in Rc for efficient sharing without cloning (trait objects can't be cloned)
    #[cfg(feature = "yaml")]
    yaml_tag_handler_registry: Rc<crate::eval::yaml_tag_handler::YamlTagHandlerRegistry>,
    // Current location context for error reporting and print_location()
    // Set during substitute_text/substitute_all, accessed by print_location() builtin
    current_location: RefCell<Option<crate::eval::LocationContext>>,
    // Python-compatible number formatting (compat feature)
    #[cfg(feature = "compat")]
    use_python_compat: bool,
    #[cfg(feature = "compat")]
    property_metadata: RefCell<HashMap<String, PropertyMetadata>>,
}

impl<const MAX_SUBSTITUTION_DEPTH: usize> EvalContext<MAX_SUBSTITUTION_DEPTH> {
    /// Create a new EvalContext with custom extensions
    ///
    /// This constructor allows providing custom extension handlers and YAML tag handlers,
    /// which is used when XacroProcessor is configured with custom extensions.
    ///
    /// # Arguments
    /// * `args` - Shared reference to the arguments map (CLI + XML args)
    /// * `extensions` - Custom extension handlers (wrapped in Rc for sharing)
    /// * `yaml_tag_handlers` - YAML tag handler registry (wrapped in Rc for sharing)
    pub fn new_with_extensions(
        args: Rc<RefCell<HashMap<String, String>>>,
        extensions: Rc<Vec<Box<dyn ExtensionHandler>>>,
        #[cfg(feature = "yaml")] yaml_tag_handlers: Rc<
            crate::eval::yaml_tag_handler::YamlTagHandlerRegistry,
        >,
    ) -> Self {
        Self {
            raw_properties: RefCell::new(HashMap::new()),
            evaluated_cache: RefCell::new(HashMap::new()),
            resolution_stack: RefCell::new(Vec::new()),
            scope_stack: RefCell::new(Vec::new()),
            args,
            extensions,
            #[cfg(feature = "yaml")]
            yaml_tag_handler_registry: yaml_tag_handlers,
            current_location: RefCell::new(None),
            #[cfg(feature = "compat")]
            use_python_compat: true, // Enabled by default when compat feature is active
            #[cfg(feature = "compat")]
            property_metadata: RefCell::new(HashMap::new()),
        }
    }

    /// Add a raw property definition
    ///
    /// Used by the single-pass expander to add properties as they are encountered.
    /// The value is stored unevaluated (lazy evaluation).
    /// Add a raw property with pre-computed metadata (private helper)
    ///
    /// This is the core implementation used by both `add_raw_property` and `define_property`.
    /// Accepts pre-computed metadata to avoid redundant computation.
    ///
    /// # Arguments
    /// * `name` - Property name
    /// * `value` - Property value (raw, unevaluated)
    /// * `metadata` - Pre-computed metadata (only used if compat feature enabled)
    #[cfg(feature = "compat")]
    fn add_raw_property_with_metadata(
        &self,
        name: String,
        value: String,
        metadata: PropertyMetadata,
    ) {
        // Warn if user is defining a property with the same name as a built-in math constant
        if BUILTIN_CONSTANTS.iter().any(|(k, _)| *k == name.as_str()) {
            log::warn!(
                "Property '{}' overrides built-in math constant. \
                 This may cause unexpected behavior. \
                 Consider using a different name.",
                name
            );
        }

        self.property_metadata
            .borrow_mut()
            .insert(name.clone(), metadata);
        self.raw_properties.borrow_mut().insert(name.clone(), value);
        self.evaluated_cache.borrow_mut().remove(&name);
    }

    /// Add a raw property without pre-computed metadata (private helper, non-compat)
    #[cfg(not(feature = "compat"))]
    fn add_raw_property_with_metadata(
        &self,
        name: String,
        value: String,
        _metadata: (),
    ) {
        // Warn if user is defining a property with the same name as a built-in math constant
        if BUILTIN_CONSTANTS.iter().any(|(k, _)| *k == name.as_str()) {
            log::warn!(
                "Property '{}' overrides built-in math constant. \
                 This may cause unexpected behavior. \
                 Consider using a different name.",
                name
            );
        }

        self.raw_properties.borrow_mut().insert(name.clone(), value);
        self.evaluated_cache.borrow_mut().remove(&name);
    }

    /// Define a property at a specific scope level
    ///
    /// Used for implementing `scope="parent"` and `scope="global"` attributes.
    /// Single unified method for all property definitions.
    ///
    /// # Arguments
    /// * `name` - Property name
    /// * `value` - Property value (raw, will be stored as-is for lazy eval)
    /// * `scope` - Where to define the property
    ///
    /// # Behavior
    /// - `Local`: Defines in current scope (top of stack) or global if no stack
    /// - `Parent`: Defines in parent scope (stack[len-2]) or global if at top level
    /// - `Global`: Always defines in global scope
    ///
    /// # Edge Cases
    /// - `Parent` at top level (stack.len() == 0): Warns and falls back to global scope
    /// - `Parent` at first macro (stack.len() == 1): Defines in global (parent of first macro)
    pub fn define_property(
        &self,
        name: String,
        value: String,
        scope: PropertyScope,
    ) {
        // Compute metadata before acquiring any borrows to avoid RefCell panic
        #[cfg(feature = "compat")]
        let metadata = self.compute_property_metadata(&value);

        match scope {
            PropertyScope::Local => {
                // Add to current top scope (or global if no stack)
                let mut stack = self.scope_stack.borrow_mut();
                let scope_depth = stack.len();
                if let Some(top) = stack.last_mut() {
                    // Update metadata for scoped property
                    #[cfg(feature = "compat")]
                    {
                        let scoped_key = format!("{}:{}", scope_depth, name);
                        self.property_metadata
                            .borrow_mut()
                            .insert(scoped_key, metadata);
                    }
                    top.insert(name, value);
                } else {
                    drop(stack);
                    #[cfg(feature = "compat")]
                    {
                        self.add_raw_property_with_metadata(name, value, metadata);
                    }
                    #[cfg(not(feature = "compat"))]
                    {
                        self.add_raw_property_with_metadata(name, value, ());
                    }
                }
            }

            PropertyScope::Parent => {
                let mut stack = self.scope_stack.borrow_mut();
                if stack.len() >= 2 {
                    // Parent exists (stack[len-2])
                    let parent_idx = stack.len() - 2;
                    let scope_depth = parent_idx + 1; // Convert index to depth
                                                      // Update metadata for scoped property
                    #[cfg(feature = "compat")]
                    {
                        let scoped_key = format!("{}:{}", scope_depth, name);
                        self.property_metadata
                            .borrow_mut()
                            .insert(scoped_key, metadata);
                    }
                    stack[parent_idx].insert(name, value);
                } else if stack.len() == 1 {
                    // At first macro level - parent is global (valid case)
                    drop(stack);
                    #[cfg(feature = "compat")]
                    {
                        self.add_raw_property_with_metadata(name, value, metadata);
                    }
                    #[cfg(not(feature = "compat"))]
                    {
                        self.add_raw_property_with_metadata(name, value, ());
                    }
                } else {
                    // stack.len() == 0: At global scope, no parent exists
                    drop(stack);
                    log::warn!(
                        "Property '{}': cannot use scope='parent' at global scope (no parent exists), \
                         falling back to global scope",
                        name
                    );
                    #[cfg(feature = "compat")]
                    {
                        self.add_raw_property_with_metadata(name, value, metadata);
                    }
                    #[cfg(not(feature = "compat"))]
                    {
                        self.add_raw_property_with_metadata(name, value, ());
                    }
                }
            }

            PropertyScope::Global => {
                // Always add to raw_properties (depth 0)
                #[cfg(feature = "compat")]
                {
                    self.add_raw_property_with_metadata(name, value, metadata);
                }
                #[cfg(not(feature = "compat"))]
                {
                    self.add_raw_property_with_metadata(name, value, ());
                }
            }
        }
    }
}

// Test-only constructors and methods
#[cfg(test)]
impl<const MAX_SUBSTITUTION_DEPTH: usize> EvalContext<MAX_SUBSTITUTION_DEPTH> {
    /// Create a new EvalContext for testing (with default extensions)
    #[allow(clippy::new_without_default)]
    pub(crate) fn new() -> Self {
        // Create with empty args map
        Self::new_with_args(Rc::new(RefCell::new(HashMap::new())))
    }

    /// Create a new EvalContext with a shared args reference for testing
    ///
    /// This constructor is used when args need to be shared with the expander
    /// (for xacro:arg directive processing). The args map is shared via Rc<RefCell<...>>
    /// to allow both the expander (to define args) and EvalContext (to resolve $(arg))
    /// to access it.
    ///
    /// # Arguments
    /// * `args` - Shared reference to the arguments map (CLI + XML args)
    pub(crate) fn new_with_args(args: Rc<RefCell<HashMap<String, String>>>) -> Self {
        let extensions = Rc::new(default_extensions());
        Self::new_with_extensions(
            args,
            extensions,
            #[cfg(feature = "yaml")]
            Rc::new(crate::eval::yaml_tag_handler::YamlTagHandlerRegistry::new()),
        )
    }

    /// Add a raw property definition (test helper)
    ///
    /// Used by tests to add properties without going through the define_property API.
    /// In production code, use `define_property` instead.
    pub(crate) fn add_raw_property(
        &self,
        name: String,
        value: String,
    ) {
        #[cfg(feature = "compat")]
        {
            let metadata = self.compute_property_metadata(&value);
            self.add_raw_property_with_metadata(name, value, metadata);
        }

        #[cfg(not(feature = "compat"))]
        {
            self.add_raw_property_with_metadata(name, value, ());
        }
    }

    /// Check if a property is defined (test helper)
    ///
    /// Used by tests to check property existence.
    /// In production code, property resolution happens automatically during substitution.
    ///
    /// # Arguments
    /// * `name` - The property name to check
    ///
    /// # Returns
    /// true if the property is defined (either in a macro scope or globally), false otherwise
    pub(crate) fn has_property(
        &self,
        name: &str,
    ) -> bool {
        self.lookup_raw_value(name).is_some()
    }
}

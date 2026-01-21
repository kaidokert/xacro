//! Scope stack management for property shadowing and lookup
//!
//! Provides scope manipulation methods for the `EvalContext` struct.
//! Scopes are used during macro expansion to shadow global properties
//! with macro parameters.

use super::{EvalContext, PropertyScope};
use std::collections::HashMap;

impl<const MAX_SUBSTITUTION_DEPTH: usize> EvalContext<MAX_SUBSTITUTION_DEPTH> {
    /// Push a new scope for macro parameter bindings
    ///
    /// Used during macro expansion to create a temporary scope where macro parameters
    /// shadow global properties. The scope must be popped when the macro expansion completes.
    ///
    /// # Example
    /// ```ignore
    /// // Illustrative example (requires internal access for setup)
    /// use std::collections::HashMap;
    ///
    /// // Global: x=10
    /// let mut scope = HashMap::new();
    /// scope.insert("x".to_string(), "5".to_string());
    /// processor.push_scope(scope);
    ///
    /// // Now ${x} resolves to "5" (shadowing global x=10)
    /// processor.pop_scope();
    /// // Now ${x} resolves to "10" again
    /// ```
    pub fn push_scope(
        &self,
        bindings: HashMap<String, String>,
    ) {
        self.scope_stack.borrow_mut().push(bindings);
    }

    /// Pop the innermost scope
    ///
    /// Should be called when a macro expansion completes to restore the previous scope.
    /// Panics if the scope stack is empty (indicates a programming error).
    pub fn pop_scope(&self) {
        // Get properties from the scope being popped before we pop it
        let properties_to_clear: Vec<String> = self
            .scope_stack
            .borrow()
            .last()
            .map(|scope| scope.keys().cloned().collect())
            .unwrap_or_default();

        // Clear cache entries for properties that were in the popped scope
        // This prevents scoped properties from leaking via the cache
        let mut cache = self.evaluated_cache.borrow_mut();
        for prop_name in &properties_to_clear {
            cache.remove(prop_name);
        }
        drop(cache);

        #[cfg(feature = "compat")]
        {
            // Clean up metadata for the scope being popped
            let scope_depth = self.scope_stack.borrow().len();
            let prefix = format!("{}:", scope_depth);
            self.property_metadata
                .borrow_mut()
                .retain(|key, _| !key.starts_with(&prefix));
        }

        self.scope_stack
            .borrow_mut()
            .pop()
            .expect("Scope stack underflow - mismatched push/pop");
    }

    /// Add a property to the current (innermost) macro scope
    ///
    /// Used to bind macro parameter values when entering a macro.
    /// Different from `add_raw_property` which adds to the global scope.
    ///
    /// # Arguments
    /// * `name` - Property name (macro parameter)
    /// * `value` - Property value (from macro call site)
    ///
    /// # Panics
    /// Panics if the scope stack is empty (no scope has been pushed)
    pub fn add_to_current_scope(
        &self,
        name: String,
        value: String,
    ) {
        // Compat feature: Track float and boolean metadata for scoped properties
        #[cfg(feature = "compat")]
        {
            let metadata = self.compute_property_metadata(&value);
            let scope_depth = self.scope_stack.borrow().len();
            let scoped_key = format!("{}:{}", scope_depth, name);
            self.property_metadata
                .borrow_mut()
                .insert(scoped_key, metadata);
        }

        self.scope_stack
            .borrow_mut()
            .last_mut()
            .expect("No scope to add to - push a scope first")
            .insert(name, value);
    }

    /// Check if a property exists in a specific scope
    ///
    /// Used by macro parameter binding to determine where to look for properties.
    /// The `^` operator uses this to find properties in parent/global scopes.
    ///
    /// # Arguments
    /// * `name` - Property name to look up
    /// * `scope` - Which scope to search (Local, Parent, or Global)
    ///
    /// # Returns
    /// * `true` if property exists in the target scope
    /// * `false` if property does not exist in the target scope
    pub fn has_property_in_scope(
        &self,
        name: &str,
        scope: PropertyScope,
    ) -> bool {
        match scope {
            PropertyScope::Local => {
                // Check current scope (top of stack or global if no stack)
                let stack = self.scope_stack.borrow();
                if let Some(top) = stack.last() {
                    top.contains_key(name)
                } else {
                    self.raw_properties.borrow().contains_key(name)
                }
            }
            PropertyScope::Parent => {
                // Check parent scope (stack[len-2] or global if at first macro)
                let stack = self.scope_stack.borrow();
                if stack.len() >= 2 {
                    let parent_idx = stack.len() - 2;
                    stack[parent_idx].contains_key(name)
                } else {
                    // Parent is global scope
                    self.raw_properties.borrow().contains_key(name)
                }
            }
            PropertyScope::Global => {
                // Check global scope only
                self.raw_properties.borrow().contains_key(name)
            }
        }
    }

    /// Get current scope depth
    ///
    /// Returns the number of active macro scopes:
    /// - 0 = global scope only (no macros running)
    /// - 1 = inside first macro
    /// - 2 = inside nested macro, etc.
    ///
    /// Used for scope manipulation (^ operator and scope="parent/global")
    pub fn scope_depth(&self) -> usize {
        self.scope_stack.borrow().len()
    }

    /// Look up property starting from a specific depth downwards
    ///
    /// Used by `^` operator to search parent scope and below.
    /// Searches from `depth` down through the scope stack to global.
    ///
    /// # Arguments
    /// * `key` - Property name to look up
    /// * `depth` - Scope depth to start search from (1-based, where depth 0 = global only)
    ///
    /// # Returns
    /// * `Some(value)` if found in any scope from depth downward
    /// * `None` if not found anywhere
    ///
    /// # Example
    /// ```ignore
    /// // scope_stack = [Scope1, Scope2]  (depth = 2)
    /// // To look up in Scope1 and below:
    /// ctx.lookup_at_depth("x", 1)  // Searches: Scope1 -> Global
    /// ```
    pub fn lookup_at_depth(
        &self,
        key: &str,
        depth: usize,
    ) -> Option<String> {
        let stack = self.scope_stack.borrow();

        // Search stack from depth down to 1
        // depth=1 means scope_stack[0], depth=2 means scope_stack[1], etc.
        let start_index = depth.min(stack.len()).saturating_sub(1);
        if depth > 0 {
            for i in (0..=start_index).rev() {
                if let Some(value) = stack[i].get(key) {
                    return Some(value.clone());
                }
            }
        }

        // Fallback to global (depth 0)
        self.raw_properties.borrow().get(key).cloned()
    }
}

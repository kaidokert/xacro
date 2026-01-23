//! Property resolution with lazy evaluation and circular dependency detection
//!
//! Implements Python xacro's lazy evaluation strategy where properties are stored
//! as raw strings and only evaluated when accessed. This allows forward references
//! and avoids errors for unused properties with undefined variables.

use super::{extract_identifiers_outside_strings, is_lambda_parameter, EvalContext};
use crate::error::XacroError;
use crate::eval::lexer::{Lexer, TokenType};
use core::cell::RefCell;
use std::collections::{HashMap, HashSet};

impl<const MAX_SUBSTITUTION_DEPTH: usize> EvalContext<MAX_SUBSTITUTION_DEPTH> {
    /// Look up raw (unevaluated) property value with scope support
    ///
    /// Searches scope stack (innermost first), then falls back to global properties.
    /// Returns the raw, unevaluated value - use `resolve_property` for full evaluation.
    ///
    /// # Arguments
    /// * `name` - Property name to look up
    ///
    /// # Returns
    /// * `Some(value)` if property exists in any scope
    /// * `None` if property not found anywhere
    pub fn lookup_raw_value(
        &self,
        name: &str,
    ) -> Option<String> {
        // Search scope stack (innermost first)
        let scope_stack = self.scope_stack.borrow();
        for scope in scope_stack.iter().rev() {
            if let Some(value) = scope.get(name) {
                return Some(value.clone());
            }
        }
        drop(scope_stack);

        // Fall back to global properties
        self.raw_properties.borrow().get(name).cloned()
    }

    /// Lazy property resolution with circular dependency detection and scope support
    ///
    /// Python xacro stores property values as raw strings and evaluates them only when accessed.
    /// This allows forward references (property A can reference B even if B is defined later)
    /// and avoids errors for unused properties with undefined variables.
    ///
    /// This method implements the same lazy evaluation strategy with scope support:
    /// 1. If in macro scope (scope_stack non-empty), SKIP cache to avoid stale values
    /// 2. Check cache (only in global scope)
    /// 3. Check circular dependency
    /// 4. Get raw value (searches scopes, then global)
    /// 5. Mark active (push to resolution stack)
    /// 6. Dependency Discovery & Resolution
    /// 7. Evaluate using context with resolved dependencies
    /// 8. Unmark & Cache result (only in global scope)
    pub(super) fn resolve_property(
        &self,
        name: &str,
    ) -> Result<String, XacroError> {
        // Skip cache when in macro scope to avoid stale cached values
        // Example bug this prevents:
        //   Global: x=10 (cached)
        //   Push macro scope: x=5
        //   Lookup ${x} → BUG: would return cached 10, should be 5
        let in_macro_scope = !self.scope_stack.borrow().is_empty();

        // 1. Check cache (only in global scope)
        if !in_macro_scope {
            if let Some(cached) = self.evaluated_cache.borrow().get(name) {
                return Ok(cached.clone());
            }
        }

        // 2. Check circular dependency
        if self.resolution_stack.borrow().contains(&name.to_string()) {
            let chain = self.resolution_stack.borrow().join(" -> ");
            return Err(XacroError::CircularPropertyDependency {
                chain: format!("{} -> {}", chain, name),
            });
        }

        // 3. Get raw value (searches scopes, then global)
        let raw_value = self
            .lookup_raw_value(name)
            .ok_or_else(|| XacroError::UndefinedProperty(name.to_string()))?;

        // 4. Mark active (push to resolution stack)
        self.resolution_stack.borrow_mut().push(name.to_string());

        // 5-6. Dependency Discovery, Resolution & Evaluation
        // RAII guard ensures stack cleanup even on panic
        struct StackGuard<'a> {
            stack: &'a RefCell<Vec<String>>,
        }
        impl Drop for StackGuard<'_> {
            fn drop(&mut self) {
                self.stack.borrow_mut().pop();
            }
        }
        let _guard = StackGuard {
            stack: &self.resolution_stack,
        };

        // First resolve any $(...) extensions in the raw value
        // This allows property values to contain $(arg ...) references
        // Example: <xacro:property name="size" value="${$(arg scale) * 2}"/>
        // Clone the location to avoid holding a borrow during nested calls
        let loc_cloned = self.current_location.borrow().clone();
        let value_with_extensions_resolved =
            self.substitute_extensions_only(&raw_value, loc_cloned.as_ref())?;

        // Recompute metadata after extension resolution (fixes boolean/float arg tracking)
        // Example: value="$(arg namespace)" with namespace:=true
        // After resolution, value_with_extensions_resolved = "true"
        // We need to recompute metadata to reflect the resolved value
        #[cfg(feature = "compat")]
        if self.use_python_compat && value_with_extensions_resolved != raw_value {
            let new_is_boolean = self.compute_boolean_metadata(&value_with_extensions_resolved);
            let new_is_float = self.compute_float_metadata(&value_with_extensions_resolved);

            // Find the correct metadata key (scoped or global)
            let mut metadata_map = self.property_metadata.borrow_mut();
            let mut key_to_update = None;

            // Search scopes for the property to find its scoped metadata key
            let scopes = self.scope_stack.borrow();
            for (i, scope) in scopes.iter().enumerate().rev() {
                if scope.contains_key(name) {
                    let depth = i + 1;
                    key_to_update = Some(format!("{}:{}", depth, name));
                    break;
                }
            }

            // If not found in scopes, it must be a global property
            if key_to_update.is_none() && self.raw_properties.borrow().contains_key(name) {
                key_to_update = Some(name.to_string());
            }

            // Update metadata using the correct key
            if let Some(key) = key_to_update {
                if let Some(meta) = metadata_map.get_mut(&key) {
                    meta.is_pseudo_boolean = new_is_boolean;
                    meta.is_float = new_is_float;
                }
            }
        }

        // Parse expression to find all variable references, then recursively resolve them
        let eval_context = self.build_eval_context(&value_with_extensions_resolved)?;
        // Evaluate using the context (all dependencies are now resolved)
        let evaluated = self.substitute_in_text(&value_with_extensions_resolved, &eval_context)?;

        // 7. Cache result (only in global scope)
        // Don't cache in macro scope - parameters are short-lived and caching would waste memory
        if !in_macro_scope {
            self.evaluated_cache
                .borrow_mut()
                .insert(name.to_string(), evaluated.clone());
        }

        Ok(evaluated)
    }

    /// Build evaluation context for a property value
    ///
    /// Extracts all property references from the value, then recursively resolves them.
    /// Returns a HashMap containing only the resolved properties needed to evaluate the value.
    ///
    /// Since resolve_property is cached, we don't need to preload all cached properties.
    /// We only add the properties actually referenced in this value.
    pub(super) fn build_eval_context(
        &self,
        value: &str,
    ) -> Result<HashMap<String, String>, XacroError> {
        let mut context = HashMap::new();
        let mut to_process: Vec<String> = self
            .extract_property_references(value)
            .into_iter()
            .collect();
        let mut processed = HashSet::new();

        // Recursively extract properties, including those referenced in lambda bodies
        while let Some(prop_name) = to_process.pop() {
            if processed.contains(&prop_name) {
                continue;
            }
            processed.insert(prop_name.clone());

            // Try to resolve, but skip if not found (extract_property_references over-captures)
            // Properties in string literals (e.g., 'test' in "${var == 'test'}") will be
            // extracted but can't be resolved - that's OK, they're not actually property references
            match self.resolve_property(&prop_name) {
                Ok(resolved) => {
                    // If this property is a lambda, extract properties it references
                    if resolved.trim().starts_with("lambda ") {
                        let nested_refs = self.extract_property_references(&resolved);
                        for nested_ref in nested_refs {
                            if !processed.contains(&nested_ref) {
                                to_process.push(nested_ref);
                            }
                        }
                    }
                    context.insert(prop_name, resolved);
                }
                Err(XacroError::UndefinedProperty(_)) => {
                    // Skip undefined - likely over-captured from string literal
                }
                Err(e) => {
                    // Propagate critical errors (CircularPropertyDependency, evaluation errors, etc.)
                    return Err(e);
                }
            }
        }

        Ok(context)
    }

    /// Extract property names referenced in a value string using lexical scanning
    ///
    /// Strategy:
    /// 1. Use our Lexer to find ${...} blocks in the raw string
    /// 2. Use extract_identifiers_outside_strings to find identifiers not in string literals
    /// 3. Return all found variable names
    ///
    /// This method properly handles dictionary key access like `data['key']` where 'key'
    /// should NOT be treated as a property reference.
    pub(super) fn extract_property_references(
        &self,
        value: &str,
    ) -> HashSet<String> {
        let mut refs = HashSet::new();

        // Special case: if value is a lambda expression (no ${...} wrapper),
        // extract variables directly from the lambda body
        let trimmed = value.trim();
        if trimmed.starts_with("lambda ") {
            // Extract variables from lambda body, skipping string literals
            let extracted = extract_identifiers_outside_strings(trimmed);
            for name in extracted {
                // Filter out lambda parameter names
                if !is_lambda_parameter(trimmed, &name) {
                    refs.insert(name);
                }
            }
            return refs;
        }

        // Normal case: extract from ${...} blocks
        let lexer = Lexer::new(value);
        for (token_type, token_value) in lexer {
            if token_type == TokenType::Expr {
                // Extract all identifier-like tokens from the expression,
                // but skip those inside string literals
                let extracted = extract_identifiers_outside_strings(&token_value);
                refs.extend(extracted);
            }
        }

        refs
    }
}

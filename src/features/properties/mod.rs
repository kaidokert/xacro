use crate::error::XacroError;
use crate::utils::eval::{eval_boolean, eval_text_with_interpreter};
use crate::utils::lexer::{Lexer, TokenType};
use core::cell::RefCell;
use pyisheval::Interpreter;
use regex::Regex;
use std::collections::{HashMap, HashSet};
use std::sync::OnceLock;

/// Cached regex for extracting variable names from expressions
/// Compiled once and reused across all property reference extractions
static VAR_REGEX: OnceLock<Regex> = OnceLock::new();

pub struct PropertyProcessor<const MAX_SUBSTITUTION_DEPTH: usize = 100> {
    interpreter: Interpreter,
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
}

/// Check if a name is a Python keyword or built-in that shouldn't be treated as a property
fn is_python_keyword(name: &str) -> bool {
    matches!(
        name,
        // Python keywords
        "True" | "False" | "None" |
        "and" | "or" | "not" | "is" | "in" |
        "if" | "else" | "elif" |
        "for" | "while" |
        "lambda" |
        "def" | "class" | "return" | "yield" |
        "try" | "except" | "finally" | "raise" |
        "with" | "as" |
        "import" | "from" |
        "pass" | "break" | "continue" |
        "global" | "nonlocal" |
        "assert" | "del" |
        // Common built-in functions that pyisheval supports
        "abs" | "min" | "max" | "sum" | "len" | "range" |
        "int" | "float" | "str" | "bool" | "list" | "tuple" | "dict" |
        "sin" | "cos" | "tan" | "sqrt" | "radians" | "degrees"
    )
}

impl<const MAX_SUBSTITUTION_DEPTH: usize> PropertyProcessor<MAX_SUBSTITUTION_DEPTH> {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        let processor = Self {
            interpreter: Interpreter::new(),
            raw_properties: RefCell::new(HashMap::new()),
            evaluated_cache: RefCell::new(HashMap::new()),
            resolution_stack: RefCell::new(Vec::new()),
            scope_stack: RefCell::new(Vec::new()),
        };

        // Initialize math constants (pi, e, tau, etc.)
        // These are commonly used in xacro expressions
        let math_constants = [
            ("pi", core::f64::consts::PI.to_string()),
            ("e", core::f64::consts::E.to_string()),
            ("tau", core::f64::consts::TAU.to_string()),
            ("M_PI", core::f64::consts::PI.to_string()),
            ("inf", f64::INFINITY.to_string()),
            ("nan", f64::NAN.to_string()),
        ];
        for (name, value) in math_constants {
            processor.add_raw_property(name.to_string(), value);
        }

        processor
    }

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
        self.scope_stack
            .borrow_mut()
            .pop()
            .expect("Scope stack underflow - mismatched push/pop");
    }

    /// Add a property to the current (top) scope
    ///
    /// This is used for incrementally building macro parameter scopes
    /// without O(N²) cloning overhead.
    ///
    /// # Panics
    /// Panics if the scope stack is empty (no scope has been pushed)
    pub fn add_to_current_scope(
        &self,
        name: String,
        value: String,
    ) {
        self.scope_stack
            .borrow_mut()
            .last_mut()
            .expect("No scope to add to - push a scope first")
            .insert(name, value);
    }

    /// Add a raw property definition
    ///
    /// Used by the single-pass expander to add properties as they are encountered.
    /// The value is stored unevaluated (lazy evaluation).
    pub fn add_raw_property(
        &self,
        name: String,
        value: String,
    ) {
        self.raw_properties.borrow_mut().insert(name.clone(), value);
        // Invalidate cache for this property if it exists
        self.evaluated_cache.borrow_mut().remove(&name);
    }

    /// Check if a property is defined (in scope stack or global properties)
    ///
    /// Used by the single-pass expander to implement the `default` attribute behavior:
    /// `<xacro:property name="x" default="5"/>` only sets x if x is not already defined.
    ///
    /// # Arguments
    /// * `name` - The property name to check
    ///
    /// # Returns
    /// true if the property is defined (either in a macro scope or globally), false otherwise
    pub fn has_property(
        &self,
        name: &str,
    ) -> bool {
        self.lookup_raw_value(name).is_some()
    }

    /// Substitute ${...} expressions in text using scope-aware resolution
    ///
    /// This is the public API for the single-pass expander. It uses the current
    /// scope stack to resolve properties, properly handling macro parameter shadowing.
    ///
    /// # Arguments
    /// * `text` - The text containing ${...} expressions to substitute
    ///
    /// # Returns
    /// The text with all ${...} expressions resolved
    pub fn substitute_text(
        &self,
        text: &str,
    ) -> Result<String, XacroError> {
        // Iterative substitution: keep evaluating as long as ${...} expressions remain
        let mut result = text.to_string();
        let mut iteration = 0;

        while result.contains("${") && iteration < MAX_SUBSTITUTION_DEPTH {
            // Build context with only the properties referenced in this iteration
            // This is more efficient than resolving all properties upfront
            let properties = self.build_eval_context(&result)?;

            let next = eval_text_with_interpreter(&result, &properties, &self.interpreter)
                .map_err(|e| XacroError::EvalError {
                    expr: match &e {
                        crate::utils::eval::EvalError::PyishEval { expr, .. } => expr.clone(),
                        crate::utils::eval::EvalError::InvalidBoolean { condition, .. } => {
                            condition.clone()
                        }
                    },
                    source: e,
                })?;

            // If result didn't change, we're done (avoids infinite loop on unresolvable expressions)
            if next == result {
                break;
            }

            result = next;
            iteration += 1;
        }

        // If we hit the limit with remaining placeholders, this indicates a problem
        if iteration >= MAX_SUBSTITUTION_DEPTH && result.contains("${") {
            let snippet = if result.len() > 100 {
                let mut end_idx = 100;
                while !result.is_char_boundary(end_idx) {
                    end_idx -= 1;
                }
                format!("{}...", &result[..end_idx])
            } else {
                result.clone()
            };
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet,
            });
        }

        Ok(result)
    }

    /// Evaluate a boolean expression using scope-aware resolution
    ///
    /// This is used for conditional evaluation (xacro:if, xacro:unless).
    /// It uses the current scope stack to resolve properties.
    ///
    /// # Arguments
    /// * `expr` - The expression to evaluate (e.g., "${x > 5}" or "true")
    ///
    /// # Returns
    /// * `Ok(true)` if the expression evaluates to true
    /// * `Ok(false)` if the expression evaluates to false
    /// * `Err` if the expression can't be evaluated
    pub fn eval_boolean(
        &self,
        expr: &str,
    ) -> Result<bool, XacroError> {
        // Build context with only the properties referenced in this expression
        // This is more efficient than resolving all properties upfront
        let properties = self.build_eval_context(expr)?;

        // Evaluate the boolean expression
        eval_boolean(expr, &properties).map_err(|e| XacroError::EvalError {
            expr: match &e {
                crate::utils::eval::EvalError::PyishEval { expr, .. } => expr.clone(),
                crate::utils::eval::EvalError::InvalidBoolean { condition, .. } => {
                    condition.clone()
                }
            },
            source: e,
        })
    }

    pub(crate) fn substitute_in_text(
        &self,
        text: &str,
        properties: &HashMap<String, String>,
    ) -> Result<String, XacroError> {
        // Iterative substitution: keep evaluating as long as ${...} expressions remain
        // This handles cases where property values themselves contain expressions
        // Example: full_name = "link_${name}" where name = "base" → "link_base"
        let mut result = text.to_string();
        let mut iteration = 0;

        while result.contains("${") && iteration < MAX_SUBSTITUTION_DEPTH {
            let next = eval_text_with_interpreter(&result, properties, &self.interpreter).map_err(
                |e| XacroError::EvalError {
                    expr: match &e {
                        crate::utils::eval::EvalError::PyishEval { expr, .. } => expr.clone(),
                        crate::utils::eval::EvalError::InvalidBoolean { condition, .. } => {
                            condition.clone()
                        }
                    },
                    source: e,
                },
            )?;

            // If result didn't change, we're done (avoids infinite loop on unresolvable expressions)
            if next == result {
                break;
            }

            result = next;
            iteration += 1;
        }

        // If we hit the limit with remaining placeholders, this indicates a problem
        if iteration >= MAX_SUBSTITUTION_DEPTH && result.contains("${") {
            let snippet = if result.len() > 100 {
                // Find a safe UTF-8 character boundary to avoid panic
                let mut end_idx = 100;
                while !result.is_char_boundary(end_idx) {
                    end_idx -= 1;
                }
                format!("{}...", &result[..end_idx])
            } else {
                result.clone()
            };
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet,
            });
        }

        Ok(result)
    }

    /// Look up raw property value, searching scopes first, then global
    ///
    /// This is the core lookup mechanism that implements scope shadowing for macro parameters.
    /// Search order:
    /// 1. scope_stack (innermost to outermost) - macro parameters
    /// 2. raw_properties - global properties
    ///
    /// Returns None if the property is not found in any scope.
    fn lookup_raw_value(
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
    fn resolve_property(
        &self,
        name: &str,
    ) -> Result<String, XacroError> {
        // CRITICAL: Skip cache when in macro scope to avoid stale cached values
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

        // Parse expression to find all variable references, then recursively resolve them
        let eval_context = self.build_eval_context(&raw_value)?;
        // Evaluate using the context (all dependencies are now resolved)
        let evaluated = self.substitute_in_text(&raw_value, &eval_context)?;

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
    fn build_eval_context(
        &self,
        value: &str,
    ) -> Result<HashMap<String, String>, XacroError> {
        let mut context = HashMap::new();
        let referenced_props = self.extract_property_references(value);

        for prop_name in referenced_props {
            // Try to resolve, but skip if not found (extract_property_references over-captures)
            // Properties in string literals (e.g., 'test' in "${var == 'test'}") will be
            // extracted but can't be resolved - that's OK, they're not actually property references
            if let Ok(resolved) = self.resolve_property(&prop_name) {
                context.insert(prop_name, resolved);
            }
        }

        Ok(context)
    }

    /// Extract property names referenced in a value string using lexical scanning
    ///
    /// Strategy:
    /// 1. Use our Lexer to find ${...} blocks in the raw string
    /// 2. Use regex to find identifiers that look like variables (not in strings/numbers)
    /// 3. Return all found variable names
    ///
    /// Note: Originally planned to use pyisheval AST parsing (PHASE_X_PLAN.md), but
    /// pyisheval 0.9.0 doesn't expose parser/AST modules publicly. This regex approach
    /// handles the common cases and is simpler. It may over-capture in some edge cases,
    /// but that's safe (we'll get proper errors during evaluation if truly undefined).
    fn extract_property_references(
        &self,
        value: &str,
    ) -> HashSet<String> {
        let mut refs = HashSet::new();
        let lexer = Lexer::new(value);

        // Get cached regex (compiled once on first use)
        // Matches: variable names (letters/underscore followed by letters/digits/underscore)
        // But NOT when preceded by a dot (e.g., obj.method) or inside strings
        let var_regex = VAR_REGEX.get_or_init(|| {
            Regex::new(r"\b([a-zA-Z_][a-zA-Z0-9_]*)\b").expect("Valid regex pattern")
        });

        for (token_type, token_value) in lexer {
            if token_type == TokenType::Expr {
                // Extract all identifier-like tokens from the expression
                for cap in var_regex.captures_iter(&token_value) {
                    if let Some(name_match) = cap.get(1) {
                        let name = name_match.as_str();
                        let match_end = name_match.end();

                        // Skip if this is a function call (identifier followed by '(')
                        // Use defensive slicing to be robust against future regex changes
                        let is_function_call = token_value
                            .get(match_end..)
                            .map(|s| s.trim_start().starts_with('('))
                            .unwrap_or(false);

                        // Filter out Python keywords, built-in functions, and function calls
                        // (over-capturing is safe, under-capturing breaks things)
                        if !is_python_keyword(name) && !is_function_call {
                            refs.insert(name.to_string());
                        }
                    }
                }
            }
        }

        refs
    }
}

#[cfg(test)]
mod tests;

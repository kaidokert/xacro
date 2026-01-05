use crate::error::XacroError;
use crate::utils::eval::{eval_boolean, eval_text_with_interpreter};
use crate::utils::lexer::{Lexer, TokenType};
use core::cell::RefCell;
use pyisheval::Interpreter;
use regex::Regex;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::sync::OnceLock;

/// Cached regex for extracting variable names from expressions
/// Compiled once and reused across all property reference extractions
static VAR_REGEX: OnceLock<Regex> = OnceLock::new();

/// Built-in math constants (name, value) that are pre-initialized
/// Users can override these, but will receive a warning
const BUILTIN_CONSTANTS: &[(&str, f64)] = &[
    ("pi", core::f64::consts::PI),
    ("e", core::f64::consts::E),
    ("tau", core::f64::consts::TAU),
    ("M_PI", core::f64::consts::PI), // Legacy alias
    ("inf", f64::INFINITY),
    ("nan", f64::NAN),
];

/// Truncate text to a safe length (100 chars) respecting UTF-8 boundaries
///
/// Used for error messages to prevent overwhelming output while ensuring
/// we don't panic on UTF-8 character boundaries.
fn truncate_snippet(text: &str) -> String {
    if text.len() > 100 {
        let mut end_idx = 100;
        while !text.is_char_boundary(end_idx) {
            end_idx -= 1;
        }
        format!("{}...", &text[..end_idx])
    } else {
        text.to_string()
    }
}

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
    // NEW: Shared reference to arguments map (injected from XacroContext)
    // Args are separate from properties and follow CLI-precedence semantics
    args: Rc<RefCell<HashMap<String, String>>>,
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
        // Create with empty args map
        Self::new_with_args(Rc::new(RefCell::new(HashMap::new())))
    }

    /// Create a new PropertyProcessor with a shared args reference
    ///
    /// This constructor is used when args need to be shared with the expander
    /// (for xacro:arg directive processing). The args map is shared via Rc<RefCell<...>>
    /// to allow both the expander (to define args) and PropertyProcessor (to resolve $(arg))
    /// to access it.
    ///
    /// # Arguments
    /// * `args` - Shared reference to the arguments map (CLI + XML args)
    pub fn new_with_args(args: Rc<RefCell<HashMap<String, String>>>) -> Self {
        let mut interpreter = Interpreter::new();

        // Initialize math constants in the interpreter
        // These are loaded directly into the interpreter's environment for use in expressions
        for (name, value) in BUILTIN_CONSTANTS {
            if let Err(e) = interpreter.eval(&format!("{} = {}", name, value)) {
                // Some constants like 'inf' and 'nan' may not be assignable in pyisheval
                // Log a warning but continue initialization
                log::debug!(
                    "Could not initialize built-in constant '{}': {}. \
                     This constant will not be available in expressions.",
                    name,
                    e
                );
            }
        }

        // Add math conversion functions as lambda expressions directly in the interpreter
        // This makes them available as callable functions in all expressions
        interpreter
            .eval("radians = lambda x: x * pi / 180")
            .expect("Failed to define radians function");
        interpreter
            .eval("degrees = lambda x: x * 180 / pi")
            .expect("Failed to define degrees function");

        Self {
            interpreter,
            raw_properties: RefCell::new(HashMap::new()),
            evaluated_cache: RefCell::new(HashMap::new()),
            resolution_stack: RefCell::new(Vec::new()),
            scope_stack: RefCell::new(Vec::new()),
            args,
        }
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
        // Warn if user is overriding a built-in math constant (only if it's already defined)
        if self.raw_properties.borrow().contains_key(&name)
            && BUILTIN_CONSTANTS.iter().any(|(k, _)| *k == name.as_str())
        {
            log::warn!(
                "Property '{}' overrides built-in math constant. \
                 This may cause unexpected behavior. \
                 Consider using a different name.",
                name
            );
        }

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

            let next = eval_text_with_interpreter(&result, &properties, &self.interpreter)?;

            // If result didn't change, we're done (avoids infinite loop on unresolvable expressions)
            if next == result {
                break;
            }

            result = next;
            iteration += 1;
        }

        // If we hit the limit with remaining placeholders, this indicates a problem
        if iteration >= MAX_SUBSTITUTION_DEPTH && result.contains("${") {
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet: truncate_snippet(&result),
            });
        }

        Ok(result)
    }

    /// Parse and resolve an extension like `$(arg foo)`, `$(find pkg)`, `$(env VAR)`
    ///
    /// Extensions are distinct from expressions - they provide external data sources.
    /// Document-order eager evaluation prevents circular dependencies naturally.
    ///
    /// # Arguments
    /// * `content` - The extension content without `$(` and `)`, e.g., "arg foo"
    ///
    /// # Returns
    /// The resolved value from the appropriate extension handler
    ///
    /// # Errors
    /// * `InvalidExtension` - Malformed syntax (empty, missing argument, multi-word, etc.)
    /// * `UnknownExtension` - Extension type not recognized
    /// * `UndefinedArgument` - Argument not found in args map
    /// * `UnimplementedFeature` - Extension type not yet implemented (find, env)
    fn resolve_extension(
        &self,
        content: &str,
    ) -> Result<String, XacroError> {
        // Simple whitespace-based parser for extension arguments
        // This is sufficient for `arg`, `find`, and `env` which expect single-token arguments
        // Note: Does not support quoted arguments with spaces (unlike shlex.split)
        let mut parts_iter = content.split_whitespace();

        // Extract extension type (required)
        let ext_type = parts_iter
            .next()
            .ok_or_else(|| XacroError::InvalidExtension {
                content: content.to_string(),
                reason: "Extension cannot be empty: $()".to_string(),
            })?;

        // Extract argument name (required)
        let arg_name = parts_iter
            .next()
            .ok_or_else(|| XacroError::InvalidExtension {
                content: content.to_string(),
                reason: format!("Extension '{}' requires an argument.", ext_type),
            })?;

        // Validate no extra parts (rejects multi-word arguments like "foo bar")
        if parts_iter.next().is_some() {
            return Err(XacroError::InvalidExtension {
                content: content.to_string(),
                reason: "Extensions must have format: $(type arg). Extra parts found.".to_string(),
            });
        }

        match ext_type {
            "arg" => {
                // Lookup in args map
                // Note: Circular dependencies are prevented by document-order eager evaluation.
                // If arg A's default references arg B, B must be defined earlier in the document.
                // Self-references like <xacro:arg name="a" default="$(arg a)"/> naturally fail
                // with UndefinedArgument because 'a' doesn't exist yet when its default is evaluated.
                self.args.borrow().get(arg_name).cloned().ok_or_else(|| {
                    XacroError::UndefinedArgument {
                        name: arg_name.to_string(),
                    }
                })
            }
            "find" => Err(XacroError::UnimplementedFeature(
                "$(find ...) extension not yet implemented".to_string(),
            )),
            "env" => Err(XacroError::UnimplementedFeature(
                "$(env ...) extension not yet implemented".to_string(),
            )),
            _ => Err(XacroError::UnknownExtension {
                ext_type: ext_type.to_string(),
            }),
        }
    }

    /// Resolve only extensions `$(...)`, preserving expressions `${...}`
    ///
    /// This is used as a helper to resolve extensions inside expressions before
    /// evaluating the expression itself. For example, `${$(arg count) * 2}` needs
    /// the `$(arg count)` resolved first before pyisheval can evaluate the arithmetic.
    ///
    /// # Arguments
    /// * `text` - Text potentially containing both `$()` and `${}`
    ///
    /// # Returns
    /// Text with `$()` resolved but `${}` preserved
    fn substitute_extensions_only(
        &self,
        text: &str,
    ) -> Result<String, XacroError> {
        self.substitute_extensions_only_inner(text, 0)
    }

    /// Inner recursive implementation of substitute_extensions_only with depth tracking
    ///
    /// # Arguments
    /// * `text` - Text potentially containing both `$()` and `${}`
    /// * `depth` - Current recursion depth (for overflow protection)
    ///
    /// # Returns
    /// Text with `$()` resolved but `${}` preserved
    fn substitute_extensions_only_inner(
        &self,
        text: &str,
        depth: usize,
    ) -> Result<String, XacroError> {
        // Prevent infinite recursion
        if depth >= MAX_SUBSTITUTION_DEPTH {
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet: truncate_snippet(text),
            });
        }

        let lexer = Lexer::new(text);
        let mut result = String::new();

        for (token_type, content) in lexer {
            match token_type {
                TokenType::Text => result.push_str(&content),
                TokenType::Extension => {
                    let resolved = self.resolve_extension(&content)?;
                    result.push_str(&resolved);
                }
                TokenType::Expr => {
                    // Don't evaluate expressions - just preserve them
                    // BUT: recursively resolve any $() inside the expression content
                    let content_with_extensions_resolved =
                        self.substitute_extensions_only_inner(&content, depth + 1)?;
                    result.push_str("${");
                    result.push_str(&content_with_extensions_resolved);
                    result.push('}');
                }
                TokenType::DollarDollarBrace => {
                    // Preserve escape sequence
                    result.push('$');
                    result.push_str(&content);
                }
            }
        }

        Ok(result)
    }

    /// Substitute both extensions `$(...)` and expressions `${...}` iteratively
    ///
    /// This is the full substitution method that handles both args and properties.
    /// It iterates until no more substitutions are possible or max depth is reached.
    ///
    /// **Evaluation order** (critical for nested cases):
    /// 1. Tokenize text into Text/Extension/Expression tokens
    /// 2. For Extension tokens: resolve immediately (lookup in args/find/env)
    /// 3. For Expression tokens:
    ///    a. First resolve any `$(...)` inside the expression
    ///    b. Then evaluate the expression with pyisheval
    /// 4. Repeat until no changes or max depth reached
    ///
    /// **Example**: `${$(arg count) * 2}` where count=5
    /// - Iteration 1: Expression token `$(arg count) * 2`
    ///   - Resolve extension: `5 * 2`
    ///   - Evaluate: `10`
    /// - Iteration 2: Text token `10` (no changes, done)
    ///
    /// # Arguments
    /// * `text` - Text containing any combination of `$()` and `${}`
    ///
    /// # Returns
    /// Fully resolved text with all substitutions applied
    ///
    /// # Errors
    /// * `MaxSubstitutionDepth` - Exceeded iteration limit (likely circular refs)
    /// * `UndefinedArgument` - Referenced arg not found
    /// * `EvalError` - Expression evaluation failed
    pub fn substitute_all(
        &self,
        text: &str,
    ) -> Result<String, XacroError> {
        let mut result = text.to_string();
        let mut iteration = 0;

        // Keep iterating while we have either ${} or $() remaining
        while (result.contains("${") || result.contains("$(")) && iteration < MAX_SUBSTITUTION_DEPTH
        {
            let lexer = Lexer::new(&result);
            let mut changed = false;
            let mut new_result = String::new();

            for (token_type, content) in lexer {
                match token_type {
                    TokenType::Text => {
                        new_result.push_str(&content);
                    }
                    TokenType::Extension => {
                        // Resolve extension immediately
                        let resolved = self.resolve_extension(&content)?;
                        new_result.push_str(&resolved);
                        changed = true;
                    }
                    TokenType::Expr => {
                        // First resolve any extensions INSIDE the expression
                        let expr_with_extensions_resolved =
                            self.substitute_extensions_only(&content)?;

                        // Then evaluate the expression (which may still contain properties)
                        // CRITICAL: wrap in ${...} so build_eval_context can extract variable names
                        let wrapped_expr = format!("${{{}}}", expr_with_extensions_resolved);
                        let properties = self.build_eval_context(&wrapped_expr)?;
                        let eval_result = eval_text_with_interpreter(
                            &wrapped_expr,
                            &properties,
                            &self.interpreter,
                        )?;

                        // Only mark changed if evaluation actually modified the expression
                        if eval_result != wrapped_expr {
                            changed = true;
                        }
                        new_result.push_str(&eval_result);
                    }
                    TokenType::DollarDollarBrace => {
                        // Preserve escape sequence
                        new_result.push('$');
                        new_result.push_str(&content);
                    }
                }
            }

            // If nothing changed, we're done
            if !changed {
                break;
            }

            result = new_result;
            iteration += 1;
        }

        // Check if we hit the depth limit with remaining substitutions
        if iteration >= MAX_SUBSTITUTION_DEPTH && (result.contains("${") || result.contains("$(")) {
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet: truncate_snippet(&result),
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
        // FIRST: Resolve any $(arg ...) extensions in the expression
        // Use substitute_extensions_only to preserve ${...} property refs for eval_boolean
        let resolved_expr = self.substitute_extensions_only(expr)?;

        // THEN: Build context with properties referenced in the resolved expression
        // This is more efficient than resolving all properties upfront
        let properties = self.build_eval_context(&resolved_expr)?;

        // FINALLY: Evaluate the fully resolved boolean expression
        eval_boolean(&resolved_expr, &properties).map_err(Into::into)
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
            let next = eval_text_with_interpreter(&result, properties, &self.interpreter)?;

            // If result didn't change, we're done (avoids infinite loop on unresolvable expressions)
            if next == result {
                break;
            }

            result = next;
            iteration += 1;
        }

        // If we hit the limit with remaining placeholders, this indicates a problem
        if iteration >= MAX_SUBSTITUTION_DEPTH && result.contains("${") {
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet: truncate_snippet(&result),
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

        // CRITICAL: First resolve any $(...) extensions in the raw value
        // This allows property values to contain $(arg ...) references
        // Example: <xacro:property name="size" value="${$(arg scale) * 2}"/>
        let value_with_extensions_resolved = self.substitute_extensions_only(&raw_value)?;

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
            match self.resolve_property(&prop_name) {
                Ok(resolved) => {
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

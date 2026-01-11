use crate::error::XacroError;
use crate::utils::eval::{
    eval_boolean, eval_text_with_interpreter, evaluate_expression, format_value_python_style,
    remove_quotes,
};
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
///
/// Note: `inf` and `nan` are NOT included because pyisheval cannot parse them:
/// - `inf` is not a valid Python literal (Python uses `float('inf')`)
/// - `nan` is not a valid Python literal (Python uses `float('nan')`)
/// - Large exponents like `9e999` fail parsing ("Unexpected trailing input")
/// - pyisheval doesn't expose an API to inject values without parsing
///
/// Instead, `inf` and `nan` are injected directly into the pyisheval context
/// HashMap in `build_pyisheval_context()` to bypass parsing limitations.
///
/// LIMITATION: Lambda expressions that reference properties with `nan` values will
/// fail with "undefined variable" errors because pyisheval cannot create NaN
/// (0.0/0.0 triggers DivisionByZero). Properties with `inf` values work correctly
/// (created using 10**400 arithmetic).
pub const BUILTIN_CONSTANTS: &[(&str, f64)] = &[
    ("pi", core::f64::consts::PI),
    ("e", core::f64::consts::E),
    ("tau", core::f64::consts::TAU),
    ("M_PI", core::f64::consts::PI), // Legacy alias
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

/// Metadata tracked for each property to support Python-like formatting
#[cfg(feature = "compat")]
#[derive(Debug, Clone)]
struct PropertyMetadata {
    /// Whether this property should be formatted as float (keep .0 for whole numbers)
    /// True if:
    /// - Property value contains decimal point (e.g., "1.5", "100.0")
    /// - Property comes from division expression (e.g., "${255/255}")
    /// - Property references a float property (e.g., "${float_prop * 2}")
    is_float: bool,
}

pub struct PropertyProcessor<const MAX_SUBSTITUTION_DEPTH: usize = 100> {
    interpreter: RefCell<Interpreter>,
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
    // Python-compatible number formatting (compat feature)
    #[cfg(feature = "compat")]
    use_python_compat: bool,
    #[cfg(feature = "compat")]
    property_metadata: RefCell<HashMap<String, PropertyMetadata>>,
}

/// Check if a name is a lambda parameter in the given lambda expression
///
/// Example: in "lambda x: x + 1", "x" is a parameter
/// Example: in "lambda x, y: x + y", both "x" and "y" are parameters
fn is_lambda_parameter(
    lambda_expr: &str,
    name: &str,
) -> bool {
    // Extract the parameter list from "lambda <params>: <body>"
    if let Some(after_lambda) = lambda_expr.strip_prefix("lambda") {
        if let Some(colon_pos) = after_lambda.find(':') {
            let param_part = &after_lambda[..colon_pos].trim();
            // Split by comma and check if name matches any parameter
            param_part.split(',').any(|param| param.trim() == name)
        } else {
            false
        }
    } else {
        false
    }
}

/// Check if a name is a Python keyword or built-in that shouldn't be treated as a property
///
/// NOTE: Newer Python keywords (match/case from 3.10+, async/await from 3.5+)
/// are intentionally omitted because ROS xacro targets older Python versions (2.7/3.x)
/// for broad compatibility. These features are not used in the xacro ecosystem.
fn is_python_keyword(name: &str) -> bool {
    matches!(
        name,
        // Python keywords (but NOT built-in functions that can be shadowed)
        "True"
            | "False"
            | "None"
            | "and"
            | "or"
            | "not"
            | "is"
            | "in"
            | "if"
            | "else"
            | "elif"
            | "for"
            | "while"
            | "lambda"
            | "def"
            | "class"
            | "return"
            | "yield"
            | "try"
            | "except"
            | "finally"
            | "raise"
            | "with"
            | "as"
            | "import"
            | "from"
            | "pass"
            | "break"
            | "continue"
            | "global"
            | "nonlocal"
            | "assert"
            | "del"
            // Math functions that are preprocessed (cannot be shadowed due to preprocessing)
            // See preprocess_math_functions() in src/utils/eval.rs
            | "abs" | "sin" | "cos" | "tan" | "asin" | "acos" | "atan"
            | "sqrt" | "floor" | "ceil" // NOTE: radians() and degrees() are NOT filtered here because they are
                                        // implemented as lambda functions in pyisheval, so they CAN be shadowed.
                                        // NOTE: len, min, max, sum, range, int, float, str, bool, list, tuple, dict
                                        // are also NOT filtered here because they can be shadowed by macro parameters
                                        // or properties. Python allows: def foo(len): return len * 2
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
        use crate::utils::eval::init_interpreter;

        let interpreter = RefCell::new(init_interpreter());

        Self {
            interpreter,
            raw_properties: RefCell::new(HashMap::new()),
            evaluated_cache: RefCell::new(HashMap::new()),
            resolution_stack: RefCell::new(Vec::new()),
            scope_stack: RefCell::new(Vec::new()),
            args,
            #[cfg(feature = "compat")]
            use_python_compat: true, // Enabled by default when compat feature is active
            #[cfg(feature = "compat")]
            property_metadata: RefCell::new(HashMap::new()),
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

    /// Check if a variable has float metadata (compat feature only)
    ///
    /// Looks up a property by name across all scopes (current to global) to determine
    /// if it should be formatted as a float. Checks scoped metadata first (depth:name),
    /// then falls back to global metadata (no scope prefix).
    ///
    /// # Arguments
    /// * `var` - The variable name to look up
    ///
    /// # Returns
    /// `true` if the variable has float metadata, `false` otherwise
    #[cfg(feature = "compat")]
    fn is_var_float(
        &self,
        var: &str,
    ) -> bool {
        let metadata = self.property_metadata.borrow();
        let scope_depth = self.scope_stack.borrow().len();

        // Try scoped metadata first (current scope down to global)
        for depth in (1..=scope_depth).rev() {
            let scoped_key = format!("{}:{}", depth, var);
            if let Some(meta) = metadata.get(&scoped_key) {
                return meta.is_float;
            }
        }

        // Fall back to checking global (no scope prefix)
        metadata.get(var).map(|m| m.is_float).unwrap_or(false)
    }

    /// Compute float metadata for a property value (compat feature only)
    ///
    /// Determines if a property should be formatted as float (with .0 for whole numbers)
    /// based on Python xacro's int/float distinction heuristics.
    ///
    /// Detection rules:
    /// 1. Value contains decimal point → float
    /// 2. Value is inf/nan → float
    /// 3. Value contains division (/) → float
    /// 4. Value references a float property → float (propagation)
    #[cfg(feature = "compat")]
    fn compute_float_metadata(
        &self,
        value: &str,
    ) -> bool {
        if !self.use_python_compat {
            return false;
        }

        let has_decimal = value.contains('.');
        let is_special = value.parse::<f64>().is_ok_and(|n| !n.is_finite());
        let has_division = value.contains('/');

        // Check if expression references any float properties
        let refs_float_prop = if value.contains("${") {
            let refs = self.extract_property_references(value);
            refs.iter().any(|r| self.is_var_float(r))
        } else {
            false
        };

        has_decimal || is_special || has_division || refs_float_prop
    }

    #[cfg(not(feature = "compat"))]
    #[allow(dead_code)]
    fn compute_float_metadata(
        &self,
        _value: &str,
    ) -> bool {
        false
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
        // Compat feature: Track float metadata for scoped properties
        #[cfg(feature = "compat")]
        {
            let is_float = self.compute_float_metadata(&value);
            let scope_depth = self.scope_stack.borrow().len();
            let scoped_key = format!("{}:{}", scope_depth, name);
            self.property_metadata
                .borrow_mut()
                .insert(scoped_key, PropertyMetadata { is_float });
        }

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

        // Compat feature: Track float metadata for properties
        #[cfg(feature = "compat")]
        {
            let is_float = self.compute_float_metadata(&value);
            self.property_metadata
                .borrow_mut()
                .insert(name.clone(), PropertyMetadata { is_float });
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

    /// Check if an expression result should be formatted as float (keep .0 for whole numbers)
    ///
    /// Returns true if:
    /// 1. Expression contains division (/) - division always produces float in Python
    /// 2. Expression references a float property - float-ness propagates
    /// 3. Result has fractional part - always float
    ///
    /// # Arguments
    /// * `expr` - The expression string (e.g., "width * 2")
    /// * `result_value` - The evaluated result
    ///
    /// # Returns
    /// Whether to format the result as float (with .0 for whole numbers)
    #[cfg(feature = "compat")]
    fn should_format_as_float(
        &self,
        expr: &str,
        result_value: &pyisheval::Value,
    ) -> bool {
        // Check if expression contains division (always produces float)
        if expr.contains('/') {
            return true;
        }

        // Check if result has fractional part (always float)
        if let pyisheval::Value::Number(n) = result_value {
            if n.fract() != 0.0 {
                return true;
            }
        }

        // Special case: if expression is just a simple variable name (no operators),
        // check its metadata directly
        let trimmed = expr.trim();
        if !is_python_keyword(trimmed)
            && trimmed.chars().all(|c| c.is_alphanumeric() || c == '_')
            && trimmed
                .chars()
                .next()
                .is_some_and(|c| c.is_alphabetic() || c == '_')
        {
            return self.is_var_float(trimmed);
        }

        // Complex expression: extract variables and check if any are float properties
        // Note: extract_property_references expects ${...} syntax, so wrap the expression
        let vars = self.extract_property_references(&format!("${{{}}}", expr));

        for var in vars {
            if self.is_var_float(&var) {
                return true;
            }
        }

        false
    }

    /// Format an evaluation result with Python-compatible number formatting
    ///
    /// When compat feature is enabled, uses metadata to decide whether to format
    /// numbers as float (with .0) or int. When disabled, uses default formatting.
    /// Always strips quotes from string literals.
    ///
    /// # Arguments
    /// * `value` - The evaluated value to format
    /// * `expr` - The original expression (used for metadata lookup)
    ///
    /// # Returns
    /// Formatted string with quotes stripped if it was a string literal
    #[cfg_attr(not(feature = "compat"), allow(unused_variables))]
    fn format_evaluation_result(
        &self,
        value: &pyisheval::Value,
        expr: &str,
    ) -> String {
        // Format the value
        let formatted = {
            #[cfg(feature = "compat")]
            {
                if self.use_python_compat {
                    let force_float = self.should_format_as_float(expr, value);
                    format_value_python_style(value, force_float)
                } else {
                    // Preserve old behavior: always format with .0 for whole numbers
                    format_value_python_style(value, true)
                }
            }
            #[cfg(not(feature = "compat"))]
            {
                // Preserve old behavior: always format with .0 for whole numbers
                format_value_python_style(value, true)
            }
        };

        // Strip quotes from string literals (always done, not compat-specific)
        remove_quotes(&formatted).to_string()
    }

    /// Perform one pass of substitution with metadata-aware formatting
    ///
    /// This is similar to eval_text_with_interpreter but uses property metadata
    /// to determine whether to format numbers as float (with .0) or int (without .0).
    ///
    /// # Arguments
    /// * `text` - The text containing ${...} to evaluate
    /// * `properties` - The property context (pre-resolved)
    ///
    /// # Returns
    /// The text with one level of ${...} expressions resolved
    fn substitute_one_pass(
        &self,
        text: &str,
        properties: &HashMap<String, String>,
    ) -> Result<String, XacroError> {
        use crate::utils::eval::build_pyisheval_context;

        // Build pyisheval context
        let context = build_pyisheval_context(properties, &mut self.interpreter.borrow_mut())
            .map_err(|e| XacroError::EvalError {
                expr: text.to_string(),
                source: e,
            })?;

        // Tokenize and process
        let lexer = Lexer::new(text);
        let mut result = String::new();

        for (token_type, token_value) in lexer {
            match token_type {
                TokenType::Text => {
                    result.push_str(&token_value);
                }
                TokenType::Expr => {
                    // Evaluate expression using centralized helper
                    let value_opt = evaluate_expression(
                        &mut self.interpreter.borrow_mut(),
                        &token_value,
                        &context,
                    )
                    .map_err(|e| XacroError::EvalError {
                        expr: token_value.clone(),
                        source: crate::utils::eval::EvalError::PyishEval {
                            expr: token_value.clone(),
                            source: e,
                        },
                    })?;

                    // Handle result
                    match value_opt {
                        Some(value) => {
                            // Format result with compat-aware number formatting
                            let formatted = self.format_evaluation_result(&value, &token_value);
                            result.push_str(&formatted);
                        }
                        None => {
                            // Special case returned no output (e.g., xacro.print_location())
                            continue;
                        }
                    }
                }
                TokenType::Extension => {
                    // Preserve extension syntax for later resolution
                    result.push_str("$(");
                    result.push_str(&token_value);
                    result.push(')');
                }
                TokenType::DollarDollarBrace => {
                    // Preserve escape sequence: $$ → $
                    result.push('$');
                    result.push_str(&token_value);
                }
            }
        }

        Ok(result)
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

            // Use metadata-aware substitution
            let next = self.substitute_one_pass(&result, &properties)?;

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
                        // Wrap in ${...} and use substitute_text for evaluation + formatting
                        let wrapped_expr = format!("${{{}}}", expr_with_extensions_resolved);
                        let eval_result = self.substitute_text(&wrapped_expr)?;

                        // Only mark changed if evaluation actually modified the expression
                        if eval_result != expr_with_extensions_resolved {
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
            let next = eval_text_with_interpreter(
                &result,
                properties,
                &mut self.interpreter.borrow_mut(),
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
    /// Get raw property value without expansion or caching
    ///
    /// Used by lazy properties which need fresh expansion every time.
    /// Searches scope stack first (innermost to outermost), then falls back
    /// to global properties.
    ///
    /// # Arguments
    /// * `name` - The property name to lookup
    ///
    /// # Returns
    /// The raw, unexpanded property value if found, None otherwise
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

        // Get cached regex (compiled once on first use)
        // Matches: variable names (letters/underscore followed by letters/digits/underscore)
        // But NOT when preceded by a dot (e.g., obj.method) or inside strings
        let var_regex = VAR_REGEX.get_or_init(|| {
            Regex::new(r"\b([a-zA-Z_][a-zA-Z0-9_]*)\b").expect("Valid regex pattern")
        });

        // Special case: if value is a lambda expression (no ${...} wrapper),
        // extract variables directly from the lambda body
        let trimmed = value.trim();
        if trimmed.starts_with("lambda ") {
            // Extract variables from lambda body
            for cap in var_regex.captures_iter(trimmed) {
                if let Some(name_match) = cap.get(1) {
                    let name = name_match.as_str();
                    // Filter out Python keywords, built-in functions, and lambda parameter names
                    // For lambda parameter detection, we look for "lambda <param>:" pattern
                    if !is_python_keyword(name) && !is_lambda_parameter(trimmed, name) {
                        refs.insert(name.to_string());
                    }
                }
            }
            return refs;
        }

        // Normal case: extract from ${...} blocks
        let lexer = Lexer::new(value);
        for (token_type, token_value) in lexer {
            if token_type == TokenType::Expr {
                // Extract all identifier-like tokens from the expression
                for cap in var_regex.captures_iter(&token_value) {
                    if let Some(name_match) = cap.get(1) {
                        let name = name_match.as_str();

                        // Note: We DO include function calls (identifier followed by '(')
                        // because lambdas stored as properties need to be in the context.
                        // Over-capturing is safe (undefined properties are skipped in build_eval_context)

                        // Filter out Python keywords and built-in functions only
                        if !is_python_keyword(name) {
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

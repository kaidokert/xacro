//! Substitution engine for expressions and extensions
//!
//! Implements iterative substitution of ${...} expressions and $(...) extensions
//! with proper handling of nested cases, recursive evaluation, and depth limits.

use super::{truncate_snippet, EvalContext};
use crate::error::XacroError;
use crate::eval::interpreter::{build_pyisheval_context, eval_boolean, init_interpreter};
use crate::eval::lexer::{Lexer, TokenType};
use crate::extensions::extension_utils;
use std::collections::HashMap;

impl<const MAX_SUBSTITUTION_DEPTH: usize> EvalContext<MAX_SUBSTITUTION_DEPTH> {
    /// Generic iterative substitution helper
    ///
    /// Applies a processing function iteratively until no more substitutions are possible
    /// or the maximum depth is reached. This eliminates code duplication across multiple
    /// substitution methods.
    ///
    /// # Arguments
    /// * `text` - Input text to process
    /// * `should_continue` - Predicate to check if processing should continue
    /// * `process_fn` - Function to apply one processing pass
    ///
    /// # Returns
    /// Fully processed text
    fn substitute_iteratively<F, P>(
        &self,
        text: &str,
        should_continue: F,
        process_fn: P,
    ) -> Result<String, XacroError>
    where
        F: Fn(&str) -> bool,
        P: Fn(&str) -> Result<String, XacroError>,
    {
        let mut result = text.to_string();
        let mut iteration = 0;

        while should_continue(&result) && iteration < MAX_SUBSTITUTION_DEPTH {
            let next = process_fn(&result)?;

            // Short-circuit: if result didn't change, we're done
            if next == result {
                break;
            }

            result = next;
            iteration += 1;
        }

        // Check if we hit the depth limit with remaining substitutions
        if iteration >= MAX_SUBSTITUTION_DEPTH && should_continue(&result) {
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet: truncate_snippet(&result),
            });
        }

        Ok(result)
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
        // Create fresh interpreter for this evaluation to prevent state leakage.
        // Using a shared interpreter caused properties defined in macro scopes to persist
        // after the scope was popped, violating Python xacro's scoping semantics.
        let mut fresh_interp = init_interpreter();

        // Build pyisheval context with the fresh interpreter
        let context = build_pyisheval_context(properties, &mut fresh_interp).map_err(|e| {
            XacroError::EvalError {
                expr: text.to_string(),
                source: e,
            }
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
                    // Evaluate expression using centralized helper with fresh interpreter
                    let value_opt = crate::eval::interpreter::evaluate_expression_impl(
                        &mut fresh_interp,
                        &token_value,
                        &context,
                        #[cfg(feature = "yaml")]
                        Some(&self.yaml_tag_handler_registry),
                    )
                    .map_err(|e| XacroError::EvalError {
                        expr: token_value.clone(),
                        source: crate::eval::EvalError::PyishEval {
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
                    // Resolve extension immediately
                    let resolved = self.resolve_extension(&token_value)?;
                    result.push_str(&resolved);
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
        self.substitute_iteratively(
            text,
            |s| s.contains("${") || s.contains("$("),
            |result| {
                // Build context with only the properties referenced in this iteration
                let properties = self.build_eval_context(result)?;
                // Use metadata-aware substitution
                self.substitute_one_pass(result, &properties)
            },
        )
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
        // Parse extension command and args
        // Format: "command arg1 arg2 ..."
        let mut parts_iter = content.split_whitespace();

        // Extract extension command (required)
        let command = parts_iter
            .next()
            .ok_or_else(|| XacroError::InvalidExtension {
                content: content.to_string(),
                reason: "Extension cannot be empty: $()".to_string(),
            })?;

        // Collect remaining parts as raw args string
        let args_raw = parts_iter.collect::<Vec<_>>().join(" ");

        // Fully resolve args: both ${...} expressions and nested $(...) extensions
        // This allows patterns like: $(find ${package_name}) and $(arg $(arg inner))
        // MAX_SUBSTITUTION_DEPTH in substitute_all prevents infinite recursion
        let args_evaluated = self.substitute_all(&args_raw)?;

        // Handle $(arg ...) specially using self.args directly
        // This ensures arg resolution uses the shared args map that gets modified
        // by xacro:arg directives during expansion.
        //
        // IMPORTANT: This check runs BEFORE the extension handler chain, which means
        // $(arg ...) cannot be overridden by custom extensions. This is intentional
        // to guarantee correctness of argument handling and prevent shadowing of this
        // core feature. See notes/EXTENSION_IMPL_ACTUAL.md for rationale.
        if command == "arg" {
            let arg_parts =
                extension_utils::expect_args(&args_evaluated, "arg", 1).map_err(|e| {
                    XacroError::InvalidExtension {
                        content: content.to_string(),
                        reason: e.to_string(),
                    }
                })?;

            return self
                .args
                .borrow()
                .get(&arg_parts[0])
                .cloned()
                .ok_or_else(|| XacroError::UndefinedArgument {
                    name: arg_parts[0].clone(),
                });
        }

        // Try each extension handler in order
        for handler in self.extensions.iter() {
            match handler.resolve(command, &args_evaluated) {
                Ok(Some(result)) => return Ok(result),
                Ok(None) => continue, // Try next handler
                Err(e) => {
                    // Handler recognized the command but resolution failed
                    return Err(XacroError::InvalidExtension {
                        content: content.to_string(),
                        reason: e.to_string(),
                    });
                }
            }
        }

        // No handler recognized the command
        Err(XacroError::UnknownExtension {
            ext_type: command.to_string(),
        })
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
    pub(super) fn substitute_extensions_only(
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
    /// 1. Resolve all extensions `$(...)` in the text
    /// 2. Evaluate all expressions `${...}` in the text
    /// 3. Repeat until no changes or max depth reached
    ///
    /// **Example**: `${$(arg count) * 2}` where count=5
    /// - Iteration 1:
    ///   - Resolve extensions: `${5 * 2}`
    ///   - Evaluate expressions: `10`
    /// - Iteration 2: No changes (done)
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
        self.substitute_iteratively(
            text,
            |s| s.contains("${") || s.contains("$("),
            |s| {
                // First resolve all extensions
                let with_extensions = self.substitute_extensions_only(s)?;
                // Then evaluate all expressions
                self.substitute_text(&with_extensions)
            },
        )
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

    /// Internal substitution helper used by property resolution
    ///
    /// Similar to substitute_text but takes an explicit property context instead
    /// of building it from the scope stack. Used during lazy property resolution
    /// to avoid recursion issues.
    ///
    /// # Arguments
    /// * `text` - Text containing ${...} expressions
    /// * `properties` - Pre-resolved property context
    ///
    /// # Returns
    /// Text with all ${...} expressions resolved
    pub(crate) fn substitute_in_text(
        &self,
        text: &str,
        properties: &HashMap<String, String>,
    ) -> Result<String, XacroError> {
        self.substitute_iteratively(
            text,
            |s| s.contains("${"),
            |result| self.substitute_one_pass(result, properties),
        )
    }
}

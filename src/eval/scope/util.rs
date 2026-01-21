//! Utility functions for property evaluation and string processing
//!
//! This module provides helper functions for:
//! - UTF-8-safe string truncation
//! - Identifier extraction from expressions
//! - Python keyword and identifier validation

use regex::Regex;
use std::collections::HashSet;
use std::sync::OnceLock;

/// Cached regex for extracting variable names from expressions
/// Compiled once and reused across all property reference extractions
static VAR_REGEX: OnceLock<Regex> = OnceLock::new();

/// Truncate text to a safe length (100 chars) respecting UTF-8 boundaries
///
/// Used for error messages to prevent overwhelming output while ensuring
/// we don't break in the middle of a multi-byte UTF-8 character.
pub(super) fn truncate_snippet(text: &str) -> String {
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

/// Extract identifiers from an expression, skipping those inside string literals
///
/// This function parses an expression and extracts variable names, but ignores
/// any identifiers that appear within single or double-quoted strings. This is
/// essential for dependency tracking where we only care about actual variable
/// references, not string content.
///
/// # Arguments
/// * `expr` - Expression to analyze (e.g., "x + 'hello' + y")
///
/// # Returns
/// Set of identifier names found outside string literals (e.g., {"x", "y"})
///
/// # Examples
/// ```text
/// extract("x + 'hello' + y") → {"x", "y"}
/// extract("'x' + y") → {"y"}
/// extract("name.attr['key']") → {"name"}
/// extract("lambda x: x + y") → {"y"} (x is a lambda param)
/// ```
///
/// # Implementation Notes
/// - Uses lazy-compiled regex for efficiency
/// - Tracks string literal state while iterating
/// - Skips lambda parameters (detected via `is_lambda_parameter`)
/// - Filters Python keywords and built-ins
pub(super) fn extract_identifiers_outside_strings(expr: &str) -> HashSet<String> {
    let mut refs = HashSet::new();
    let var_regex = VAR_REGEX.get_or_init(|| {
        Regex::new(r"\b([a-zA-Z_][a-zA-Z0-9_]*)\b").expect("VAR_REGEX should be valid")
    });

    // Track string literal boundaries to skip identifiers inside strings
    let mut in_single_quote = false;
    let mut in_double_quote = false;
    let mut escaped = false;

    // Build a mask of which characters are inside strings
    let mut inside_string = vec![false; expr.len()];
    for (idx, ch) in expr.char_indices() {
        if escaped {
            escaped = false;
            if in_single_quote || in_double_quote {
                inside_string[idx] = true;
            }
            continue;
        }

        match ch {
            '\\' => {
                escaped = true;
                if in_single_quote || in_double_quote {
                    inside_string[idx] = true;
                }
            }
            '\'' if !in_double_quote => {
                inside_string[idx] = true; // Include the quote itself
                in_single_quote = !in_single_quote;
            }
            '"' if !in_single_quote => {
                inside_string[idx] = true; // Include the quote itself
                in_double_quote = !in_double_quote;
            }
            _ => {
                if in_single_quote || in_double_quote {
                    inside_string[idx] = true;
                }
            }
        }
    }

    // Extract identifiers, but only those outside of strings
    for cap in var_regex.captures_iter(expr) {
        if let Some(name_match) = cap.get(1) {
            let name = name_match.as_str();
            let start_pos = name_match.start();

            // Check if this identifier is inside a string
            if start_pos < inside_string.len()
                && !inside_string[start_pos]
                && !is_python_keyword(name)
            {
                refs.insert(name.to_string());
            }
        }
    }

    refs
}

/// Check if a name is a lambda parameter in the given lambda expression
///
/// Detects lambda parameters to avoid treating them as property references
/// during dependency tracking. For example, in "lambda x: x + y", x is a
/// parameter (not a property) but y is a property reference.
///
/// # Arguments
/// * `lambda_expr` - Full expression that may contain lambda
/// * `name` - Identifier to check
///
/// # Returns
/// `true` if name is a lambda parameter in the expression
///
/// # Examples
/// ```text
/// is_lambda_parameter("lambda x: x + y", "x") → true
/// is_lambda_parameter("lambda x: x + y", "y") → false
/// is_lambda_parameter("lambda a, b: a + b", "a") → true
/// is_lambda_parameter("x + y", "x") → false
/// ```
pub(super) fn is_lambda_parameter(
    lambda_expr: &str,
    name: &str,
) -> bool {
    // Check if the expression contains a lambda that defines this parameter
    if let Some(lambda_pos) = lambda_expr.find("lambda") {
        if let Some(colon_pos) = lambda_expr[lambda_pos..].find(':') {
            let params_section = &lambda_expr[lambda_pos + 6..lambda_pos + colon_pos];
            // Simple check: parameter name appears in params list
            for param in params_section.split(',') {
                if param.trim() == name {
                    return true;
                }
            }
        }
    }
    false
}

/// Check if a name is a Python keyword or built-in that shouldn't be treated as a property
///
/// Used during dependency tracking to filter out identifiers that are Python
/// language constructs or preprocessed functions, not property references.
///
/// Only includes:
/// - Python keywords (cannot be shadowed)
/// - Math functions that are preprocessed and cannot be shadowed
///
/// Does NOT include general built-ins (len, str, int, file, etc.) because they
/// CAN be shadowed by macro parameters or properties.
///
/// # Arguments
/// * `name` - Identifier to check
///
/// # Returns
/// `true` if name is a Python keyword or preprocessed function
pub(super) fn is_python_keyword(name: &str) -> bool {
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
            // See preprocess_math_functions() in src/eval/interpreter/math.rs
            | "abs"
            | "sin"
            | "cos"
            | "tan"
            | "asin"
            | "acos"
            | "atan"
            | "atan2"
            | "sqrt"
            | "floor"
            | "ceil" // NOTE: radians() and degrees() are NOT filtered here because they are
                     // implemented as lambda functions in pyisheval, so they CAN be shadowed.
                     // NOTE: len, min, max, sum, range, int, float, str, bool, list, tuple, dict
                     // are also NOT filtered here because they can be shadowed by macro parameters
                     // or properties. Python allows: def foo(len): return len * 2
    )
}

/// Check if a string is a simple identifier (not a complex expression)
///
/// Used to determine if a property name is a simple variable reference
/// (eligible for direct lookup) vs. a complex expression that needs evaluation.
///
/// # Arguments
/// * `name` - String to check
///
/// # Returns
/// `true` if name is a simple identifier (alphanumeric + underscore, starts with letter/underscore)
///
/// # Examples
/// ```text
/// is_simple_identifier("x") → true
/// is_simple_identifier("my_var") → true
/// is_simple_identifier("var123") → true
/// is_simple_identifier("x + y") → false
/// is_simple_identifier("123abc") → false
/// is_simple_identifier("True") → false (Python keyword)
/// ```
pub(super) fn is_simple_identifier(name: &str) -> bool {
    !is_python_keyword(name)
        && name.chars().all(|c| c.is_alphanumeric() || c == '_')
        && name
            .chars()
            .next()
            .is_some_and(|c| c.is_alphabetic() || c == '_')
}

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
/// extract("lambda x: x + y") → {"lambda", "x", "y"}
/// ```
///
/// # Implementation Notes
/// - Uses lazy-compiled regex for efficiency
/// - Tracks string literal state while iterating
/// - Filters Python keywords and built-ins
/// - Does NOT filter lambda parameters (caller must use `is_lambda_parameter` if needed)
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
///
/// # Limitations
///
/// This implementation uses simple string parsing and does not handle:
/// - Default parameter values: `lambda x=1: x + y`
/// - Type annotations (Python 3): `lambda x: int: x`
/// - Nested parentheses in parameter defaults
///
/// These edge cases are acceptable for xacro's typical lambda usage patterns.
pub(super) fn is_lambda_parameter(
    lambda_expr: &str,
    name: &str,
) -> bool {
    // Check if the expression starts with "lambda " (with proper word boundary)
    if let Some(after_lambda) = lambda_expr.strip_prefix("lambda") {
        // Find the colon that separates parameters from body
        if let Some(colon_pos) = after_lambda.find(':') {
            let params_section = &after_lambda[..colon_pos].trim();
            // Check if name matches any parameter (comma-separated)
            params_section.split(',').any(|param| param.trim() == name)
        } else {
            false
        }
    } else {
        false
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_identifiers_basic() {
        let expr = "x + y * z";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 3);
        assert!(ids.contains("x"));
        assert!(ids.contains("y"));
        assert!(ids.contains("z"));
    }

    #[test]
    fn test_extract_identifiers_single_quote_string() {
        let expr = "data['initial_positions']";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 1);
        assert!(ids.contains("data"));
        assert!(
            !ids.contains("initial_positions"),
            "String literal should be skipped"
        );
    }

    #[test]
    fn test_extract_identifiers_double_quote_string() {
        let expr = r#"data["key_name"]"#;
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 1);
        assert!(ids.contains("data"));
        assert!(
            !ids.contains("key_name"),
            "String literal should be skipped"
        );
    }

    #[test]
    fn test_extract_identifiers_escaped_quote_single() {
        let expr = r"message['can\'t']";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 1);
        assert!(ids.contains("message"));
        assert!(!ids.contains("can"));
        assert!(!ids.contains("t"));
    }

    #[test]
    fn test_extract_identifiers_escaped_quote_double() {
        let expr = r#"data["quote\"inside"]"#;
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 1);
        assert!(ids.contains("data"));
        assert!(!ids.contains("quote"));
        assert!(!ids.contains("inside"));
    }

    #[test]
    fn test_extract_identifiers_mixed_quotes() {
        let expr = r#"func('single') + other("double")"#;
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 2);
        assert!(ids.contains("func"));
        assert!(ids.contains("other"));
        assert!(!ids.contains("single"));
        assert!(!ids.contains("double"));
    }

    #[test]
    fn test_extract_identifiers_quote_inside_different_quote() {
        // Single quote inside double quotes
        let expr = r#"data["it's"]"#;
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 1);
        assert!(ids.contains("data"));
        assert!(!ids.contains("it"));
        assert!(!ids.contains("s"));

        // Double quote inside single quotes
        let expr2 = r#"data['"quoted"']"#;
        let ids2 = extract_identifiers_outside_strings(expr2);
        assert_eq!(ids2.len(), 1);
        assert!(ids2.contains("data"));
        assert!(!ids2.contains("quoted"));
    }

    #[test]
    fn test_extract_identifiers_function_call() {
        let expr = "load_yaml(filename)";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 2);
        assert!(ids.contains("load_yaml"));
        assert!(ids.contains("filename"));
    }

    #[test]
    fn test_extract_identifiers_complex_expression() {
        let expr = "load_yaml(file)['initial_positions']['joint1']";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 2);
        assert!(ids.contains("load_yaml"));
        assert!(ids.contains("file"));
        assert!(
            !ids.contains("initial_positions"),
            "String literal in brackets"
        );
        assert!(!ids.contains("joint1"), "String literal in brackets");
    }

    #[test]
    fn test_extract_identifiers_python_keywords_filtered() {
        let expr = "if x and y or z";
        let ids = extract_identifiers_outside_strings(expr);
        // Keywords (if, and, or) should be filtered out
        assert_eq!(ids.len(), 3);
        assert!(ids.contains("x"));
        assert!(ids.contains("y"));
        assert!(ids.contains("z"));
        assert!(!ids.contains("if"));
        assert!(!ids.contains("and"));
        assert!(!ids.contains("or"));
    }

    #[test]
    fn test_extract_identifiers_empty_string() {
        let expr = "";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 0);
    }

    #[test]
    fn test_extract_identifiers_only_strings() {
        let expr = "'hello' + \"world\"";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 0, "Only string literals, no identifiers");
    }

    #[test]
    fn test_extract_identifiers_multiple_occurrences() {
        let expr = "x + x * x";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 1, "Set should deduplicate");
        assert!(ids.contains("x"));
    }

    #[test]
    fn test_extract_identifiers_underscore_names() {
        let expr = "_private + __dunder__ + normal_name";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 3);
        assert!(ids.contains("_private"));
        assert!(ids.contains("__dunder__"));
        assert!(ids.contains("normal_name"));
    }

    #[test]
    fn test_extract_identifiers_numbers_excluded() {
        let expr = "x + 123 + y";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 2);
        assert!(ids.contains("x"));
        assert!(ids.contains("y"));
        // Numbers should not be captured by the identifier regex
    }

    #[test]
    fn test_extract_identifiers_empty_quotes() {
        let expr = "data[''] + other[\"\"]";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 2);
        assert!(ids.contains("data"));
        assert!(ids.contains("other"));
    }

    #[test]
    fn test_extract_identifiers_adjacent_strings() {
        let expr = "'first''second'";
        let ids = extract_identifiers_outside_strings(expr);
        assert_eq!(ids.len(), 0, "Adjacent string literals");
    }

    #[test]
    fn test_extract_identifiers_backslash_at_end_of_string() {
        let expr = r"data['path\\'] + x";
        let ids = extract_identifiers_outside_strings(expr);
        assert!(ids.contains("data"));
        assert!(ids.contains("x"));
        assert!(!ids.contains("path"));
    }
}

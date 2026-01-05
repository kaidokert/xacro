use crate::utils::lexer::{Lexer, TokenType};
use pyisheval::{Interpreter, Value};
use std::collections::HashMap;

#[derive(Debug, thiserror::Error)]
pub enum EvalError {
    #[error("Failed to evaluate expression '{expr}': {source}")]
    PyishEval {
        expr: String,
        #[source]
        source: pyisheval::EvalError,
    },

    #[error("Xacro conditional \"{condition}\" evaluated to \"{evaluated}\", which is not a boolean expression.")]
    InvalidBoolean {
        condition: String,
        evaluated: String,
    },
}

/// Remove quotes from string values (handles both single and double quotes)
fn remove_quotes(s: &str) -> &str {
    // pyisheval's StringLit to_string() returns strings with single quotes
    if (s.starts_with('\'') && s.ends_with('\'')) || (s.starts_with('"') && s.ends_with('"')) {
        if s.len() >= 2 {
            &s[1..s.len() - 1]
        } else {
            s
        }
    } else {
        s
    }
}

/// Evaluate text containing ${...} expressions
///
/// Examples:
///   "hello ${name}" with {name: "world"} → "hello world"
///   "${2 + 3}" → "5"
///   "${width * 2}" with {width: "0.5"} → "1"
pub fn eval_text(
    text: &str,
    properties: &HashMap<String, String>,
) -> Result<String, EvalError> {
    let interp = Interpreter::new();
    eval_text_with_interpreter(text, properties, &interp)
}

/// Build a pyisheval context HashMap from properties
///
/// Converts string properties to pyisheval Values, parsing numbers when possible.
/// For lambda expressions, evaluates them to callable lambda values.
/// This allows properties to be used in expressions with correct types.
fn build_pyisheval_context(properties: &HashMap<String, String>) -> HashMap<String, Value> {
    let mut interp = Interpreter::new();

    // First pass: Load all constants and non-lambda properties into the interpreter
    // This ensures that lambda expressions can reference them during evaluation
    for (name, value) in properties.iter() {
        let trimmed = value.trim();
        if !trimmed.starts_with("lambda ") {
            // Load constants and regular properties into interpreter
            if let Ok(num) = value.parse::<f64>() {
                let _ = interp.eval(&format!("{} = {}", name, num));
            }
        }
    }

    // Second pass: Build the actual context, evaluating lambdas
    properties
        .iter()
        .map(|(name, value)| {
            // Try to parse as number first
            if let Ok(num) = value.parse::<f64>() {
                return (name.clone(), Value::Number(num));
            }

            // Check if it's a lambda expression
            let trimmed = value.trim();
            if trimmed.starts_with("lambda ") {
                // Evaluate the lambda expression to get a callable lambda value
                // The interpreter now has all constants loaded from first pass
                match interp.eval(trimmed) {
                    Ok(lambda_value) => return (name.clone(), lambda_value),
                    Err(_) => {
                        // If evaluation fails, treat as string
                        // This shouldn't happen for valid lambda expressions
                    }
                }
            }

            // Default: store as string literal
            (name.clone(), Value::StringLit(value.clone()))
        })
        .collect()
}

/// Evaluate text containing ${...} expressions using a provided interpreter
///
/// This version allows reusing an Interpreter instance for better performance
/// when processing multiple text blocks with the same properties context.
pub fn eval_text_with_interpreter(
    text: &str,
    properties: &HashMap<String, String>,
    interp: &Interpreter,
) -> Result<String, EvalError> {
    // Build context for pyisheval
    let context = build_pyisheval_context(properties);

    // Tokenize the input text
    let lexer = Lexer::new(text);
    let mut result = Vec::new();

    // Process each token
    for (token_type, token_value) in lexer {
        match token_type {
            TokenType::Text => {
                // Plain text, keep as-is
                result.push(token_value);
            }
            TokenType::Expr => {
                // ${expr} - evaluate using pyisheval
                match interp.eval_with_context(&token_value, &context) {
                    Ok(value) => {
                        let value_str = value.to_string();
                        result.push(remove_quotes(&value_str).to_string());
                    }
                    Err(e) => {
                        return Err(EvalError::PyishEval {
                            expr: token_value.clone(),
                            source: e,
                        });
                    }
                }
            }
            TokenType::Extension => {
                // $(extension) - handle later (Phase 6)
                // For now, just keep the original text
                result.push(format!("$({})", token_value));
            }
            TokenType::DollarDollarBrace => {
                // $$ escape - output $ followed by the delimiter ({ or ()
                result.push(format!("${}", token_value));
            }
        }
    }

    Ok(result.join(""))
}

/// Apply Python xacro's STRICT string truthiness rules
///
/// Accepts: "true", "True", "false", "False", or parseable integers
/// Rejects: Everything else (including "nonsense", empty string, floats as strings)
fn apply_string_truthiness(
    s: &str,
    original: &str,
) -> Result<bool, EvalError> {
    let trimmed = s.trim();

    // Exact string matches for boolean literals
    if trimmed == "true" || trimmed == "True" {
        return Ok(true);
    }
    if trimmed == "false" || trimmed == "False" {
        return Ok(false);
    }

    // Try integer conversion (Python's bool(int(value)))
    if let Ok(i) = trimmed.parse::<i64>() {
        return Ok(i != 0);
    }

    // Everything else is an error (STRICT mode)
    Err(EvalError::InvalidBoolean {
        condition: original.to_string(),
        evaluated: s.to_string(),
    })
}

/// Evaluate expression as boolean following Python xacro's STRICT rules
///
/// Python xacro's get_boolean_value() logic (ref/xacro/src/xacro/__init__.py:856):
/// - Accepts: "true", "True", "false", "False"
/// - Accepts: Any string convertible to int: "1", "0", "42", "-5"
/// - REJECTS: "nonsense", empty string, anything else → Error
///
/// CRITICAL: This preserves type information from pyisheval!
/// ${3*0.1} evaluates to float 0.3 (truthy), NOT string "0.3" (would error)
///
/// Examples:
///   eval_boolean("true", &props) → Ok(true)
///   eval_boolean("${3*0.1}", &props) → Ok(true)  // Float 0.3 != 0.0
///   eval_boolean("${0}", &props) → Ok(false)     // Integer 0
///   eval_boolean("nonsense", &props) → Err(InvalidBoolean)
pub fn eval_boolean(
    text: &str,
    properties: &HashMap<String, String>,
) -> Result<bool, EvalError> {
    let interp = Interpreter::new();

    // Build context for pyisheval
    let context = build_pyisheval_context(properties);

    // Tokenize input to detect structure
    let lexer = Lexer::new(text);
    let tokens: Vec<_> = lexer.collect();

    // CASE 1: Single ${expr} token → Preserve type, apply truthiness on Value
    // This is CRITICAL for float truthiness: ${3*0.1} → float 0.3 → true
    if tokens.len() == 1 && tokens[0].0 == TokenType::Expr {
        let value = interp
            .eval_with_context(&tokens[0].1, &context)
            .map_err(|e| EvalError::PyishEval {
                expr: text.to_string(),
                source: e,
            })?;

        // Apply Python truthiness based on Value type
        return match value {
            Value::Number(n) => Ok(n != 0.0), // Float/int truthiness (includes bools: True=1.0, False=0.0)
            Value::StringLit(s) => {
                // String: must be "true"/"false" or parseable as int
                apply_string_truthiness(&s, text)
            }
            // Other types (Lambda, List, etc.) - error for now
            _ => Err(EvalError::InvalidBoolean {
                condition: text.to_string(),
                evaluated: format!("{:?}", value),
            }),
        };
    }

    // CASE 2: Multiple tokens or plain text → Evaluate to string, then parse
    // Example: "text ${expr} more" or just "true"
    let evaluated = eval_text_with_interpreter(text, properties, &interp)?;
    apply_string_truthiness(&evaluated, text)
}

#[cfg(test)]
mod tests {
    use super::*;

    // TEST 1: Backward compatibility - simple property substitution
    #[test]
    fn test_simple_property_substitution() {
        let mut props = HashMap::new();
        props.insert("width".to_string(), "0.5".to_string());

        let result = eval_text("${width}", &props).unwrap();
        assert_eq!(result, "0.5");
    }

    // TEST 2: Property in text
    #[test]
    fn test_property_in_text() {
        let mut props = HashMap::new();
        props.insert("width".to_string(), "0.5".to_string());

        let result = eval_text("The width is ${width} meters", &props).unwrap();
        assert_eq!(result, "The width is 0.5 meters");
    }

    // TEST 3: Multiple properties
    #[test]
    fn test_multiple_properties() {
        let mut props = HashMap::new();
        props.insert("width".to_string(), "0.5".to_string());
        props.insert("height".to_string(), "1.0".to_string());

        let result = eval_text("${width} x ${height}", &props).unwrap();
        // Note: pyisheval formats 1.0 as "1" since it's an integer value
        assert_eq!(result, "0.5 x 1");
    }

    // TEST 4: NEW - Simple arithmetic
    #[test]
    fn test_arithmetic_expression() {
        let mut props = HashMap::new();
        props.insert("width".to_string(), "0.5".to_string());

        let result = eval_text("${width * 2}", &props).unwrap();
        assert_eq!(result, "1");
    }

    // TEST 5: NEW - Arithmetic without properties
    #[test]
    fn test_pure_arithmetic() {
        let props = HashMap::new();

        let result = eval_text("${2 + 3}", &props).unwrap();
        assert_eq!(result, "5");
    }

    // TEST 6: NEW - Complex expression
    #[test]
    fn test_complex_expression() {
        let mut props = HashMap::new();
        props.insert("width".to_string(), "0.5".to_string());
        props.insert("height".to_string(), "2.0".to_string());

        let result = eval_text("${width * height + 1}", &props).unwrap();
        assert_eq!(result, "2");
    }

    // TEST 7: NEW - String concatenation with literals
    // Note: pyisheval doesn't currently support string concatenation with +
    // This is documented as a known limitation. Use property substitution instead.
    #[test]
    #[ignore]
    fn test_string_concatenation() {
        let props = HashMap::new();

        // String concatenation with string literals (quoted in expression)
        let result = eval_text("${'link' + '_' + 'base'}", &props).unwrap();
        assert_eq!(result, "link_base");
    }

    // TEST 8: NEW - Built-in functions
    #[test]
    fn test_builtin_functions() {
        let props = HashMap::new();

        let result = eval_text("${abs(-5)}", &props).unwrap();
        assert_eq!(result, "5");

        let result = eval_text("${max(2, 5, 3)}", &props).unwrap();
        assert_eq!(result, "5");
    }

    // TEST 9: NEW - Conditional expressions
    #[test]
    fn test_conditional_expression() {
        let mut props = HashMap::new();
        props.insert("width".to_string(), "0.5".to_string());

        let result = eval_text("${width if width > 0.3 else 0.3}", &props).unwrap();
        assert_eq!(result, "0.5");
    }

    // TEST 10: Text without expressions (pass through)
    #[test]
    fn test_no_expressions() {
        let props = HashMap::new();
        let result = eval_text("hello world", &props).unwrap();
        assert_eq!(result, "hello world");
    }

    // TEST 11: Empty string
    #[test]
    fn test_empty_string() {
        let props = HashMap::new();
        let result = eval_text("", &props).unwrap();
        assert_eq!(result, "");
    }

    // TEST 12: Error case - undefined property
    #[test]
    fn test_undefined_property() {
        let props = HashMap::new();
        let result = eval_text("${undefined}", &props);
        assert!(result.is_err());
    }

    // TEST 13: String property substitution (non-numeric values)
    #[test]
    fn test_string_property() {
        let mut props = HashMap::new();
        props.insert("link_name".to_string(), "base_link".to_string());
        props.insert("joint_type".to_string(), "revolute".to_string());

        // Test single property
        let result = eval_text("${link_name}", &props).unwrap();
        assert_eq!(result, "base_link");

        // Test property in text
        let result = eval_text("name_${link_name}_suffix", &props).unwrap();
        assert_eq!(result, "name_base_link_suffix");

        // Test multiple string properties
        let result = eval_text("${link_name} ${joint_type}", &props).unwrap();
        assert_eq!(result, "base_link revolute");
    }

    #[test]
    fn test_double_dollar_escape() {
        let props = HashMap::new();

        // Test $$ escape with brace - should produce literal ${
        let result = eval_text("$${expr}", &props).unwrap();
        assert_eq!(result, "${expr}");

        // Test $$ escape with paren - should produce literal $(
        let result = eval_text("$$(command)", &props).unwrap();
        assert_eq!(result, "$(command)");

        // Test $$ escape in context
        let result = eval_text("prefix_$${literal}_suffix", &props).unwrap();
        assert_eq!(result, "prefix_${literal}_suffix");
    }

    // ===== NEW TESTS FOR eval_boolean =====

    // Test from Python xacro: test_boolean_if_statement (line 715)
    #[test]
    fn test_eval_boolean_literals() {
        let props = HashMap::new();

        // Boolean string literals
        assert_eq!(eval_boolean("true", &props).unwrap(), true);
        assert_eq!(eval_boolean("false", &props).unwrap(), false);
        assert_eq!(eval_boolean("True", &props).unwrap(), true);
        assert_eq!(eval_boolean("False", &props).unwrap(), false);
    }

    // Test from Python xacro: test_integer_if_statement (line 735)
    #[test]
    fn test_eval_boolean_integer_truthiness() {
        let props = HashMap::new();

        // Integer literals as strings
        assert_eq!(eval_boolean("0", &props).unwrap(), false);
        assert_eq!(eval_boolean("1", &props).unwrap(), true);
        assert_eq!(eval_boolean("42", &props).unwrap(), true);
        assert_eq!(eval_boolean("-5", &props).unwrap(), true);

        // Integer expressions
        assert_eq!(eval_boolean("${0*42}", &props).unwrap(), false); // 0
        assert_eq!(eval_boolean("${0}", &props).unwrap(), false);
        assert_eq!(eval_boolean("${1*2+3}", &props).unwrap(), true); // 5
    }

    // Test from Python xacro: test_float_if_statement (line 755)
    #[test]
    fn test_eval_boolean_float_truthiness() {
        let props = HashMap::new();

        // CRITICAL: Float expressions must preserve type
        assert_eq!(eval_boolean("${3*0.0}", &props).unwrap(), false); // 0.0
        assert_eq!(eval_boolean("${3*0.1}", &props).unwrap(), true); // 0.3 (non-zero float)
        assert_eq!(eval_boolean("${0.5}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${-0.1}", &props).unwrap(), true);
    }

    // Test from Python xacro: test_property_if_statement (line 769)
    #[test]
    fn test_eval_boolean_with_properties() {
        let mut props = HashMap::new();
        props.insert("condT".to_string(), "1".to_string()); // True as number
        props.insert("condF".to_string(), "0".to_string()); // False as number
        props.insert("num".to_string(), "5".to_string());

        assert_eq!(eval_boolean("${condT}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${condF}", &props).unwrap(), false);
        assert_eq!(eval_boolean("${num}", &props).unwrap(), true); // 5 != 0

        // Note: pyisheval doesn't have True/False as built-in constants
        // They would need to be defined as properties with value 1/0
    }

    // Test from Python xacro: test_equality_expression_in_if_statement (line 788)
    #[test]
    fn test_eval_boolean_expressions() {
        let mut props = HashMap::new();
        props.insert("var".to_string(), "useit".to_string());

        // Equality
        assert_eq!(eval_boolean("${var == 'useit'}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${var == 'other'}", &props).unwrap(), false);

        // Comparison
        props.insert("x".to_string(), "5".to_string());
        assert_eq!(eval_boolean("${x > 3}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${x < 3}", &props).unwrap(), false);

        // Note: pyisheval doesn't support 'in' operator for strings yet
        // That would require extending pyisheval or using a different evaluator
    }

    /// Test that pyisheval returns Value::Number for boolean expressions
    ///
    /// CRITICAL: This test documents that pyisheval v0.9.0 does NOT have Value::Bool.
    /// Boolean comparison expressions like ${1 == 1} return Value::Number(1.0), not Value::Bool(true).
    /// This is similar to Python where bool is a subclass of int (True == 1, False == 0).
    ///
    /// This test exists to:
    /// 1. Verify our Number-based truthiness handling works for comparisons
    /// 2. Document pyisheval's current behavior (see notes/PYISHEVAL_ISSUES.md)
    /// 3. Catch if pyisheval adds Value::Bool in future (this would fail, prompting us to update)
    #[test]
    fn test_eval_boolean_comparison_expressions() {
        let mut props = HashMap::new();
        props.insert("x".to_string(), "5".to_string());
        props.insert("y".to_string(), "10".to_string());

        // Equality comparisons
        assert_eq!(eval_boolean("${1 == 1}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${1 == 2}", &props).unwrap(), false);
        assert_eq!(eval_boolean("${x == 5}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${x == y}", &props).unwrap(), false);

        // Inequality comparisons
        assert_eq!(eval_boolean("${1 != 2}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${1 != 1}", &props).unwrap(), false);

        // Less than / greater than
        assert_eq!(eval_boolean("${x < y}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${x > y}", &props).unwrap(), false);
        assert_eq!(eval_boolean("${x <= 5}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${y >= 10}", &props).unwrap(), true);

        // NOTE: pyisheval v0.9.0 does NOT support `and`/`or` operators
        // These would fail: ${1 and 1}, ${x > 3 and y < 15}
        // Fortunately, real xacro files don't use these - they use simpler expressions
        // See notes/PYISHEVAL_ISSUES.md for details

        // Note: All these comparison expressions are evaluated by pyisheval as Value::Number(1.0) or Value::Number(0.0)
        // Our eval_boolean correctly applies != 0.0 truthiness to convert to bool
    }

    // Test from Python xacro: test_invalid_if_statement (line 729)
    #[test]
    fn test_eval_boolean_invalid_values() {
        let props = HashMap::new();

        // STRICT mode: "nonsense" should error
        let result = eval_boolean("nonsense", &props);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("not a boolean expression"));

        // Empty string should error
        let result = eval_boolean("", &props);
        assert!(result.is_err());

        // Random text should error
        let result = eval_boolean("random text", &props);
        assert!(result.is_err());
    }

    // Test edge case: whitespace handling
    #[test]
    fn test_eval_boolean_whitespace() {
        let props = HashMap::new();

        // Should trim whitespace
        assert_eq!(eval_boolean(" true ", &props).unwrap(), true);
        assert_eq!(eval_boolean("\tfalse\n", &props).unwrap(), false);
        assert_eq!(eval_boolean("  0  ", &props).unwrap(), false);
        assert_eq!(eval_boolean("  1  ", &props).unwrap(), true);
    }

    // Test case sensitivity
    #[test]
    fn test_eval_boolean_case_sensitivity() {
        let props = HashMap::new();

        // "true" and "True" are accepted
        assert_eq!(eval_boolean("true", &props).unwrap(), true);
        assert_eq!(eval_boolean("True", &props).unwrap(), true);

        // But not other cases (should error)
        assert!(eval_boolean("TRUE", &props).is_err());
        assert!(eval_boolean("tRuE", &props).is_err());
    }

    // Test type preservation: the key feature!
    #[test]
    fn test_eval_boolean_type_preservation() {
        let props = HashMap::new();

        // Single expression: type preserved
        // ${3*0.1} → Value::Number(0.3) → != 0.0 → true
        assert_eq!(eval_boolean("${3*0.1}", &props).unwrap(), true);

        // Multiple tokens: becomes string
        // "result: ${3*0.1}" → "result: 0.3" → can't parse as int → error
        let result = eval_boolean("result: ${3*0.1}", &props);
        assert!(result.is_err());
    }

    // Test Boolean value type from pyisheval
    #[test]
    fn test_eval_boolean_bool_values() {
        let props = HashMap::new();

        // pyisheval returns Value::Bool directly
        assert_eq!(eval_boolean("${1 == 1}", &props).unwrap(), true);
        assert_eq!(eval_boolean("${1 == 2}", &props).unwrap(), false);
        assert_eq!(eval_boolean("${5 > 3}", &props).unwrap(), true);
    }
}

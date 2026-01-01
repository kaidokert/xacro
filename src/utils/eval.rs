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
    // Build context for pyisheval
    let mut context = HashMap::new();
    for (name, value) in properties {
        // Try to parse as number, otherwise treat as string
        if let Ok(num) = value.parse::<f64>() {
            context.insert(name.clone(), Value::Number(num));
        } else {
            // Store strings without quotes - pyisheval's StringLit handles this
            context.insert(name.clone(), Value::StringLit(value.clone()));
        }
    }

    // Create interpreter once, reuse for all expressions (performance optimization)
    let interp = Interpreter::new();

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

/// Evaluate expression as boolean (for conditionals)
pub fn eval_boolean(
    text: &str,
    properties: &HashMap<String, String>,
) -> Result<bool, EvalError> {
    let text_result = eval_text(text, properties)?;
    let interp = Interpreter::new();

    interp
        .eval_boolean(&text_result)
        .map_err(|e| EvalError::PyishEval {
            expr: text.to_string(),
            source: e,
        })
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
}

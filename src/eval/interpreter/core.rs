use super::init::{format_value_python_style, init_interpreter, EvalError};
use super::math::preprocess_math_functions;
use super::parsing::{escape_python_string, eval_literal, remove_quotes};
use super::yaml_utils::preprocess_load_yaml;
use crate::eval::lexer::{Lexer, TokenType};
use log;
use pyisheval::{EvalError as PyEvalError, Interpreter, Value};
use std::collections::HashMap;

/// Build a pyisheval context HashMap from properties
///
/// Converts string properties to pyisheval Values, parsing numbers when possible.
/// For lambda expressions, evaluates them to callable lambda values using the
/// provided interpreter. This ensures lambdas capture the correct environment.
///
/// # Arguments
/// * `properties` - Property name-value pairs to convert to pyisheval Values
/// * `interp` - The interpreter to use for evaluating lambda expressions
///
/// # Errors
/// Returns `EvalError` if a lambda expression fails to evaluate.
pub fn build_pyisheval_context(
    properties: &HashMap<String, String>,
    interp: &mut Interpreter,
) -> Result<HashMap<String, Value>, EvalError> {
    // First pass: Load all constants and non-lambda properties into the interpreter
    // This ensures that lambda expressions can reference them during evaluation
    // Note: We skip inf/nan/NaN as they can't be created via arithmetic in pyisheval
    // (10**400 creates inf but 0.0/0.0 fails with DivisionByZero)

    // WORKAROUND: Add Python built-in constant None as numeric 0
    //
    // This is needed for load_yaml() which returns the string "None" for null YAML values.
    // We inject it into the interpreter environment so lambda expressions can reference it.
    //
    // LIMITATION: This is a bandaid that violates Python semantics:
    // - In Python: None + 5 raises TypeError
    // - In our implementation: None + 5 = 5 (because None=0)
    //
    // This workaround is necessary because pyisheval doesn't support None as a proper type.
    // Proper fix requires adding None type support to pyisheval itself, which would allow:
    // - Type checking: None + 5 -> TypeError
    // - Boolean context: None -> False
    // - Identity checks: x is None
    //
    // Until pyisheval adds None type support, we use this numeric approximation.
    // It handles the common cases (mostly boolean checks) but masks real type errors.
    interp.eval("None = 0").map_err(|e| EvalError::PyishEval {
        expr: "None = 0".to_string(),
        source: e,
    })?;

    for (name, value) in properties.iter() {
        let trimmed = value.trim();
        if !trimmed.starts_with("lambda ") {
            // Load dict/list/tuple literals into interpreter so lambdas can reference them
            // They'll be properly evaluated as Python expressions in the second pass
            if trimmed.starts_with('{') || trimmed.starts_with('[') || trimmed.starts_with('(') {
                // Try to load as Python literal into interpreter environment for lambda closure
                if let Err(e) = interp.eval(&format!("{} = {}", name, trimmed)) {
                    log::warn!(
                        "Could not load property '{}' with value '{}' into interpreter as Python literal: {}. \
                         Loading as string fallback to prevent 'Undefined variable' in lambdas.",
                        name, value, e
                    );
                    // Fallback: load as string literal so the variable is at least defined
                    // This prevents "Undefined variable" errors in lambdas that reference it
                    let escaped = escape_python_string(value);
                    let _ = interp.eval(&format!("{} = '{}'", name, escaped));
                }
                continue;
            }

            // Apply Python xacro's type coercion logic
            match eval_literal(value) {
                Value::Number(num) => {
                    // Special handling for inf/nan so lambdas can reference them
                    if num.is_infinite() {
                        // Use 10**400 to create infinity (pyisheval can't parse "inf" literal)
                        let sign = if num.is_sign_negative() { "-" } else { "" };
                        let expr = format!("{} = {}10 ** 400", name, sign);
                        interp
                            .eval(&expr)
                            .map_err(|e| EvalError::PyishEval { expr, source: e })?;
                        continue;
                    }
                    if num.is_nan() {
                        // LIMITATION: Cannot create NaN in pyisheval (0.0/0.0 triggers DivisionByZero)
                        // Lambdas that reference NaN properties will fail with "undefined variable"
                        log::warn!(
                            "Property '{}' has NaN value, which cannot be loaded into interpreter. \
                             Lambda expressions referencing this property will fail.",
                            name
                        );
                        continue;
                    }
                    // Numeric property (including boolean True=1.0, False=0.0): load as number
                    interp.eval(&format!("{} = {}", name, num)).map_err(|e| {
                        EvalError::PyishEval {
                            expr: format!("{} = {}", name, num),
                            source: e,
                        }
                    })?;
                }
                Value::StringLit(s) if !s.is_empty() => {
                    // String property: load as quoted string literal
                    // Skip empty strings as pyisheval can't parse ''
                    let escaped_value = escape_python_string(&s);
                    interp
                        .eval(&format!("{} = '{}'", name, escaped_value))
                        .map_err(|e| EvalError::PyishEval {
                            expr: format!("{} = '{}'", name, escaped_value),
                            source: e,
                        })?;
                }
                Value::StringLit(_) => {
                    // Empty string - skip in first pass (pyisheval can't handle '')
                    // Will be stored as Value::StringLit("") in the second pass
                }
                _ => {
                    // eval_literal only returns Value::Number or Value::StringLit
                    // Any other variant indicates a logic error
                    unreachable!(
                        "eval_literal returned unexpected value type for property '{}': {:?}",
                        name, value
                    );
                }
            }
        }
    }

    // Second pass: Build the actual context, evaluating lambdas
    let mut context: HashMap<String, Value> = properties
        .iter()
        .map(|(name, value)| -> Result<(String, Value), EvalError> {
            // Check if it's a lambda expression (check before eval_literal to avoid treating it as string)
            let trimmed = value.trim();
            if trimmed.starts_with("lambda ") {
                // Evaluate and assign the lambda expression to the variable name
                // The interpreter now has all constants and properties loaded from first pass
                let assignment = format!("{} = {}", name, trimmed);
                interp.eval(&assignment).map_err(|e| EvalError::PyishEval {
                    expr: assignment.clone(),
                    source: e,
                })?;

                // Retrieve the lambda value to store in context
                let lambda_value = interp.eval(name).map_err(|e| EvalError::PyishEval {
                    expr: name.clone(),
                    source: e,
                })?;

                return Ok((name.clone(), lambda_value));
            }

            // Try to evaluate as Python expression (lists, dicts, etc.)
            // This allows properties like "[1, 2, 3]" to be stored as actual lists,
            // so that indexing like arr[0] works correctly
            //
            // Optimization: Only attempt evaluation if value looks like a Python literal
            // to avoid unnecessary parse errors on common string values
            // (reuse trimmed from lambda check above)
            if trimmed.starts_with('[') || trimmed.starts_with('{') || trimmed.starts_with('(') {
                match interp.eval(trimmed) {
                    Ok(evaluated_value) => {
                        Ok((name.clone(), evaluated_value))
                    }
                    Err(e) => {
                        // Distinguish expected parse failures from unexpected runtime errors
                        match &e {
                            // Unexpected: runtime/type errors may indicate issues with property definitions
                            PyEvalError::TypeError
                            | PyEvalError::DivisionByZero
                            | PyEvalError::LambdaCallError
                            | PyEvalError::ArgError(_)
                            | PyEvalError::DictKeyError => {
                                log::warn!(
                                    "Property '{}' with value '{}' failed evaluation: {}. Treating as string literal.",
                                    name, value, e
                                );
                            }
                            // Expected: parse errors or undefined variables
                            PyEvalError::ParseError(_) | PyEvalError::UndefinedVar(_) => {}
                        }
                        // In all error cases, fall back to treating value as string literal
                        Ok((name.clone(), Value::StringLit(value.clone())))
                    }
                }
            } else {
                // Apply Python xacro's type coercion for literals (int/float/boolean)
                let literal_value = eval_literal(value);
                Ok((name.clone(), literal_value))
            }
        })
        .collect::<Result<HashMap<_, _>, _>>()?;

    // Manually inject inf, nan, and None constants (Strategy 3: bypass parsing)
    // Python xacro provides these via float('inf') and math.inf, but they're also
    // used as bare identifiers in expressions. Pyisheval cannot parse these as
    // literals, so we inject them directly into the context.
    //
    // WHY INJECT TWICE? (both into interpreter above AND into context map here):
    // 1. Interpreter injection (line ~928): Makes constants available to lambda expressions
    //    e.g., "lambda x: x if x != None else 0" needs None in interpreter environment
    // 2. Context map injection (here): Makes constants available to eval_with_context()
    //    e.g., "${None + 5}" needs None in the context passed to eval_with_context()
    //
    // Both injections are necessary for complete coverage of all expression types.
    //
    // WORKAROUND for None: Modeled as 0.0 (see comment above for limitations).
    // This violates Python semantics (None + 5 should be TypeError, not 5) but is
    // necessary until pyisheval adds proper None type support.
    context.insert("inf".to_string(), Value::Number(f64::INFINITY));
    context.insert("nan".to_string(), Value::Number(f64::NAN));
    context.insert("None".to_string(), Value::Number(0.0));

    Ok(context)
}

/// Evaluate a single expression string, handling Xacro-specific special cases
///
/// This centralizes handling of special functions like xacro.print_location()
/// that don't fit into the generic pyisheval evaluation model.
///
/// # Arguments
/// * `interp` - The pyisheval interpreter to use
/// * `expr` - The expression string to evaluate
/// * `context` - The variable context for evaluation
///
/// # Returns
/// * `Ok(Some(value))` - Normal expression evaluated successfully
/// * `Ok(None)` - Special case that produces no output (e.g., xacro.print_location())
/// * `Err(e)` - Evaluation error
///
/// # Special Cases
/// * `xacro.print_location()` - Debug function that prints stack trace to stderr in Python.
///   We stub it out by returning None (no output). This is handled as a special case because:
///   1. pyisheval doesn't support object.method syntax
///   2. This is a debug-only function with no production use
///   3. We're not implementing the full debug functionality
pub(crate) fn evaluate_expression_impl(
    interp: &mut Interpreter,
    expr: &str,
    context: &HashMap<String, Value>,
    #[cfg(feature = "yaml")] yaml_tag_handler_registry: Option<
        &crate::eval::yaml_tag_handler::YamlTagHandlerRegistry,
    >,
) -> Result<Option<Value>, pyisheval::EvalError> {
    let trimmed_expr = expr.trim();
    if trimmed_expr == "xacro.print_location()" {
        // Special case: stub debug function returns no output
        return Ok(None);
    }

    // Preprocess math functions (cos, sin, tan, etc.) before evaluation
    // This converts native math calls into computed values since pyisheval
    // doesn't support calling native Rust functions
    // Pass context to allow property references in function arguments
    let preprocessed = preprocess_math_functions(expr, interp, context).map_err(|e| match e {
        EvalError::PyishEval { source, .. } => source,
        _ => pyisheval::EvalError::ParseError(e.to_string()),
    })?;

    // Preprocess load_yaml() calls.
    // If the 'yaml' feature is enabled, this loads YAML files and replaces load_yaml()
    // with dict literals. Otherwise, it returns an error if load_yaml() is used.
    let preprocessed = preprocess_load_yaml(
        &preprocessed,
        interp,
        context,
        #[cfg(feature = "yaml")]
        yaml_tag_handler_registry,
    )
    .map_err(|e| match e {
        EvalError::PyishEval { source, .. } => source,
        _ => pyisheval::EvalError::ParseError(e.to_string()),
    })?;

    interp.eval_with_context(&preprocessed, context).map(Some)
}

/// Internal implementation of text evaluation with optional YAML tag handler registry
fn eval_text_with_interpreter_impl(
    text: &str,
    properties: &HashMap<String, String>,
    interp: &mut Interpreter,
    #[cfg(feature = "yaml")] yaml_tag_handler_registry: Option<
        &crate::eval::yaml_tag_handler::YamlTagHandlerRegistry,
    >,
) -> Result<String, EvalError> {
    // Build context for pyisheval (may fail if lambdas have errors)
    // This loads properties into the interpreter and evaluates lambda expressions
    let context = build_pyisheval_context(properties, interp)?;

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
                // Evaluate expression using centralized helper
                match evaluate_expression_impl(
                    interp,
                    &token_value,
                    &context,
                    #[cfg(feature = "yaml")]
                    yaml_tag_handler_registry,
                ) {
                    Ok(Some(value)) => {
                        #[cfg(feature = "compat")]
                        let value_str = format_value_python_style(&value, false);
                        #[cfg(not(feature = "compat"))]
                        let value_str = format_value_python_style(&value, true);
                        result.push(remove_quotes(&value_str).to_string());
                    }
                    Ok(None) => {
                        // Special case returned no output (e.g., xacro.print_location())
                        continue;
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
                // $(extension) - handle later
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

/// Evaluate text containing ${...} expressions using a provided interpreter
///
/// This version allows reusing an Interpreter instance for better performance
/// when processing multiple text blocks with the same properties context.
///
/// Takes a mutable reference to ensure lambdas are created in the same
/// interpreter context where they'll be evaluated.
pub fn eval_text_with_interpreter(
    text: &str,
    properties: &HashMap<String, String>,
    interp: &mut Interpreter,
) -> Result<String, EvalError> {
    eval_text_with_interpreter_impl(
        text,
        properties,
        interp,
        #[cfg(feature = "yaml")]
        None,
    )
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

    // Try float conversion (for values like "1.0")
    if let Ok(f) = trimmed.parse::<f64>() {
        return Ok(f != 0.0);
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
/// Important: This preserves type information from pyisheval.
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
    let mut interp = init_interpreter();

    // Build context for pyisheval (may fail if lambdas have errors)
    let context = build_pyisheval_context(properties, &mut interp)?;

    // Tokenize input to detect structure
    let lexer = Lexer::new(text);
    let tokens: Vec<_> = lexer.collect();

    // CASE 1: Single ${expr} token → Preserve type, apply truthiness on Value
    // This is important for float truthiness: ${3*0.1} → float 0.3 → true
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
    let evaluated = eval_text_with_interpreter(text, properties, &mut interp)?;
    apply_string_truthiness(&evaluated, text)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eval::interpreter::parsing::{
        find_matching_paren, split_args_balanced, SUPPORTED_MATH_FUNCS,
    };

    /// Test helper: Evaluate text containing ${...} expressions
    ///
    /// Examples:
    ///   "hello ${name}" with {name: "world"} → "hello world"
    ///   "${2 + 3}" → "5"
    ///   "${width * 2}" with {width: "0.5"} → "1"
    fn eval_text(
        text: &str,
        properties: &HashMap<String, String>,
    ) -> Result<String, EvalError> {
        let mut interp = init_interpreter();
        eval_text_with_interpreter(text, properties, &mut interp)
    }

    /// Test helper: Evaluate a single expression using the given interpreter and context
    ///
    /// # Arguments
    /// * `interp` - The interpreter instance
    /// * `expr` - The expression to evaluate
    /// * `context` - The evaluation context (properties as pyisheval Values)
    ///
    /// # Returns
    /// * `Ok(Some(value))` - Normal expression evaluated successfully
    /// * `Ok(None)` - Special case that produces no output (e.g., xacro.print_location())
    /// * `Err(e)` - Evaluation error
    fn evaluate_expression(
        interp: &mut Interpreter,
        expr: &str,
        context: &HashMap<String, Value>,
    ) -> Result<Option<Value>, pyisheval::EvalError> {
        evaluate_expression_impl(
            interp,
            expr,
            context,
            #[cfg(feature = "yaml")]
            None,
        )
    }

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
        // Whole numbers format without .0 (Python int behavior)
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

        // Float expressions must preserve type
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
    /// Note: pyisheval v0.9.0 does NOT have Value::Bool.
    /// Boolean comparison expressions like ${1 == 1} return Value::Number(1.0), not Value::Bool(true).
    /// This is similar to Python where bool is a subclass of int (True == 1, False == 0).
    ///
    /// This test exists to:
    /// 1. Verify our Number-based truthiness handling works for comparisons
    /// 2. Document pyisheval's current behavior
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

        // NOTE: `and`/`or` operators are now supported in pyisheval v0.13+
        // See tests/test_logical_operators.rs for integration tests

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

    // Test evaluate_expression special case handling directly
    #[test]
    fn test_evaluate_expression_special_cases() {
        let mut interp = init_interpreter();
        let context = HashMap::new();

        // Test xacro.print_location() special case
        let result = evaluate_expression(&mut interp, "xacro.print_location()", &context).unwrap();
        assert!(
            result.is_none(),
            "xacro.print_location() should return None"
        );

        // Test with surrounding whitespace
        let result =
            evaluate_expression(&mut interp, "  xacro.print_location()  ", &context).unwrap();
        assert!(
            result.is_none(),
            "xacro.print_location() with whitespace should return None"
        );

        // Test a normal expression to ensure it's not affected
        let result = evaluate_expression(&mut interp, "1 + 1", &context).unwrap();
        assert!(
            matches!(result, Some(Value::Number(n)) if n == 2.0),
            "Normal expression should evaluate correctly"
        );
    }

    // Test xacro.print_location() stub function via integration
    #[test]
    fn test_xacro_print_location_stub() {
        let props = HashMap::new();

        // xacro.print_location() should return empty string
        let result = eval_text("${xacro.print_location()}", &props).unwrap();
        assert_eq!(result, "");

        // Should work in text context too
        let result = eval_text("before${xacro.print_location()}after", &props).unwrap();
        assert_eq!(result, "beforeafter");

        // With whitespace in expression
        let result = eval_text("${ xacro.print_location() }", &props).unwrap();
        assert_eq!(result, "");
    }

    // Test that inf and nan are available via direct context injection
    #[test]
    fn test_inf_nan_direct_injection() {
        let props = HashMap::new();
        let mut interp = init_interpreter();

        // Build context with direct inf/nan injection
        let context = build_pyisheval_context(&props, &mut interp).unwrap();

        // Verify inf and nan are in the context
        assert!(
            context.contains_key("inf"),
            "Context should contain 'inf' key"
        );
        assert!(
            context.contains_key("nan"),
            "Context should contain 'nan' key"
        );

        // Test 1: inf should be positive infinity
        if let Some(Value::Number(n)) = context.get("inf") {
            assert!(
                n.is_infinite() && n.is_sign_positive(),
                "inf should be positive infinity, got: {}",
                n
            );
        } else {
            panic!("inf should be a Number value");
        }

        // Test 2: nan should be NaN
        if let Some(Value::Number(n)) = context.get("nan") {
            assert!(n.is_nan(), "nan should be NaN, got: {}", n);
        } else {
            panic!("nan should be a Number value");
        }

        // Test 3: inf should be usable in expressions
        let result = interp.eval_with_context("inf * 2", &context);
        assert!(
            matches!(result, Ok(Value::Number(n)) if n.is_infinite() && n.is_sign_positive()),
            "inf * 2 should return positive infinity, got: {:?}",
            result
        );

        // Test 4: nan should be usable in expressions
        let result = interp.eval_with_context("nan + 1", &context);
        assert!(
            matches!(result, Ok(Value::Number(n)) if n.is_nan()),
            "nan + 1 should return NaN, got: {:?}",
            result
        );
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

    // Lambda expression tests
    #[test]
    fn test_basic_lambda_works() {
        let mut props = HashMap::new();
        props.insert("f".to_string(), "lambda x: x * 2".to_string());
        assert_eq!(eval_text("${f(5)}", &props).unwrap(), "10");
    }

    // NOTE: pyisheval doesn't support multi-parameter lambdas (parser limitation)
    // lambda x, y: x + y fails with "Unexpected trailing input"
    // This is a known pyisheval limitation - single parameter lambdas only

    // Python-style number formatting tests
    #[test]
    fn test_format_value_python_style_whole_numbers() {
        // Whole numbers format without .0 (Python int behavior)
        assert_eq!(format_value_python_style(&Value::Number(0.0), false), "0");
        assert_eq!(format_value_python_style(&Value::Number(1.0), false), "1");
        assert_eq!(format_value_python_style(&Value::Number(2.0), false), "2");
        assert_eq!(format_value_python_style(&Value::Number(-1.0), false), "-1");
        assert_eq!(
            format_value_python_style(&Value::Number(100.0), false),
            "100"
        );
    }

    #[test]
    fn test_format_value_python_style_fractional() {
        //Fractional numbers use default formatting (no trailing zeros)
        assert_eq!(format_value_python_style(&Value::Number(1.5), false), "1.5");
        assert_eq!(format_value_python_style(&Value::Number(0.5), false), "0.5");
        assert_eq!(
            format_value_python_style(&Value::Number(0.4235294117647059), false),
            "0.4235294117647059"
        );
    }

    #[test]
    fn test_format_value_python_style_special() {
        //Special values
        assert_eq!(
            format_value_python_style(&Value::Number(f64::INFINITY), false),
            "inf"
        );
        assert_eq!(
            format_value_python_style(&Value::Number(f64::NEG_INFINITY), false),
            "-inf"
        );
        assert_eq!(
            format_value_python_style(&Value::Number(f64::NAN), false),
            "NaN"
        );
    }

    #[test]
    fn test_eval_with_python_number_formatting() {
        let mut props = HashMap::new();
        props.insert("height".to_string(), "1.0".to_string());

        // Whole numbers format without .0 (mimics Python int behavior)
        assert_eq!(eval_text("${height}", &props).unwrap(), "1");
        assert_eq!(eval_text("${1.0 + 0.0}", &props).unwrap(), "1");
        assert_eq!(eval_text("${2.0 * 1.0}", &props).unwrap(), "2");
    }

    #[test]
    fn test_lambda_referencing_property() {
        let mut props = HashMap::new();
        props.insert("offset".to_string(), "10".to_string());
        props.insert("add_offset".to_string(), "lambda x: x + offset".to_string());
        assert_eq!(eval_text("${add_offset(5)}", &props).unwrap(), "15");
    }

    #[test]
    fn test_lambda_referencing_multiple_properties() {
        let mut props = HashMap::new();
        props.insert("a".to_string(), "2".to_string());
        props.insert("b".to_string(), "3".to_string());
        props.insert("scale".to_string(), "lambda x: x * a + b".to_string());
        assert_eq!(eval_text("${scale(5)}", &props).unwrap(), "13");
    }

    #[test]
    fn test_lambda_with_conditional() {
        let mut props = HashMap::new();
        props.insert(
            "sign".to_string(),
            "lambda x: 1 if x > 0 else -1".to_string(),
        );
        assert_eq!(eval_text("${sign(5)}", &props).unwrap(), "1");
        assert_eq!(eval_text("${sign(-3)}", &props).unwrap(), "-1");
    }

    #[test]
    fn test_multiple_lambdas() {
        let mut props = HashMap::new();
        props.insert("double".to_string(), "lambda x: x * 2".to_string());
        props.insert("triple".to_string(), "lambda x: x * 3".to_string());
        assert_eq!(
            eval_text("${double(5)} ${triple(5)}", &props).unwrap(),
            "10 15"
        );
    }

    #[test]
    fn test_lambda_referencing_inf_property() {
        let mut props = HashMap::new();
        props.insert("my_inf".to_string(), "inf".to_string());
        props.insert("is_inf".to_string(), "lambda x: x == my_inf".to_string());
        // inf == inf should be true (1)
        assert_eq!(eval_text("${is_inf(inf)}", &props).unwrap(), "1");
    }

    // ===== Math Function Tests =====

    #[test]
    fn test_math_functions_cos_sin() {
        let mut props = HashMap::new();
        props.insert("pi".to_string(), "3.141592653589793".to_string());

        let result = eval_text("${cos(0)}", &props).unwrap();
        assert_eq!(result, "1");

        let result = eval_text("${sin(0)}", &props).unwrap();
        assert_eq!(result, "0");

        let result = eval_text("${cos(pi)}", &props).unwrap();
        assert_eq!(result, "-1");
    }

    #[test]
    fn test_math_functions_nested() {
        let mut props = HashMap::new();
        props.insert("radius".to_string(), "0.5".to_string());

        // radians() is a lambda defined in init_interpreter
        let result = eval_text("${radius*cos(radians(0))}", &props).unwrap();
        assert_eq!(result, "0.5");

        let result = eval_text("${radius*cos(radians(60))}", &props).unwrap();
        // cos(60°) = 0.5, so 0.5 * 0.5 = 0.25 (with floating point rounding)
        let value: f64 = result.parse().unwrap();
        assert!(
            (value - 0.25).abs() < 1e-10,
            "Expected ~0.25, got {}",
            value
        );
    }

    #[test]
    fn test_math_functions_sqrt_abs() {
        let props = HashMap::new();

        let result = eval_text("${sqrt(16)}", &props).unwrap();
        assert_eq!(result, "4");

        let result = eval_text("${abs(-5)}", &props).unwrap();
        assert_eq!(result, "5");

        let result = eval_text("${abs(5)}", &props).unwrap();
        assert_eq!(result, "5");
    }

    #[test]
    fn test_math_functions_floor_ceil() {
        let props = HashMap::new();

        let result = eval_text("${floor(3.7)}", &props).unwrap();
        assert_eq!(result, "3");

        let result = eval_text("${ceil(3.2)}", &props).unwrap();
        assert_eq!(result, "4");

        let result = eval_text("${floor(-2.3)}", &props).unwrap();
        assert_eq!(result, "-3");

        let result = eval_text("${ceil(-2.3)}", &props).unwrap();
        assert_eq!(result, "-2");
    }

    #[test]
    fn test_math_functions_trig() {
        let props = HashMap::new();

        // tan(0) = 0
        let result = eval_text("${tan(0)}", &props).unwrap();
        assert_eq!(result, "0");

        // asin(0) = 0
        let result = eval_text("${asin(0)}", &props).unwrap();
        assert_eq!(result, "0");

        // acos(1) = 0
        let result = eval_text("${acos(1)}", &props).unwrap();
        assert_eq!(result, "0");

        // atan(0) = 0
        let result = eval_text("${atan(0)}", &props).unwrap();
        assert_eq!(result, "0");
    }

    #[test]
    fn test_math_functions_multiple_in_expression() {
        let mut props = HashMap::new();
        props.insert("x".to_string(), "3".to_string());
        props.insert("y".to_string(), "4".to_string());

        // sqrt(x^2 + y^2) = sqrt(9 + 16) = sqrt(25) = 5
        let result = eval_text("${sqrt(x**2 + y**2)}", &props).unwrap();
        assert_eq!(result, "5");
    }

    /// Test to prevent divergence between regex pattern and match statement
    ///
    /// This ensures all functions in SUPPORTED_MATH_FUNCS have corresponding implementations,
    /// catching bugs at test time rather than runtime.
    #[test]
    fn test_math_functions_regex_match_consistency() {
        let props = HashMap::new();

        // Test each function in SUPPORTED_MATH_FUNCS to ensure it's implemented
        for func in SUPPORTED_MATH_FUNCS {
            // atan2 and pow require two arguments, others require one
            let expr = if *func == "atan2" || *func == "pow" {
                format!("${{{}(0, 1)}}", func)
            } else {
                format!("${{{}(0)}}", func)
            };
            let result = eval_text(&expr, &props);

            // Ensure evaluation succeeds - unreachable!() would panic if function is missing
            result.expect("Evaluation should succeed for all supported math functions");
        }
    }

    #[test]
    fn test_find_matching_paren_with_brackets() {
        // Regression test: find_matching_paren should handle brackets inside parens
        let text = "pow([1,2][0], 3)";
        let result = find_matching_paren(text, 3); // Start at '(' after 'pow'
        assert_eq!(result, Some(15)); // Position of final ')'
    }

    #[test]
    fn test_find_matching_paren_with_braces() {
        // Regression test: find_matching_paren should handle braces (dict literals)
        let text = "func({a:1,b:2}, 3)";
        let result = find_matching_paren(text, 4); // Start at '(' after 'func'
        assert_eq!(result, Some(17)); // Position of final ')'
    }

    #[test]
    fn test_split_args_with_array_literal() {
        // Regression test: split_args_balanced should not split on commas inside arrays
        let args = "[1,2][0], 3";
        let result = split_args_balanced(args);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], "[1,2][0]");
        assert_eq!(result[1], " 3");
    }

    #[test]
    fn test_split_args_with_dict_literal() {
        // Regression test: split_args_balanced should not split on commas inside dicts
        let args = "{a:1,b:2}, 3";
        let result = split_args_balanced(args);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], "{a:1,b:2}");
        assert_eq!(result[1], " 3");
    }

    #[test]
    fn test_split_args_with_nested_structures() {
        // Complex case: nested arrays, dicts, and function calls
        let args = "[[1,2],[3,4]], {x:[5,6]}, max(7,8)";
        let result = split_args_balanced(args);
        assert_eq!(result.len(), 3);
        assert_eq!(result[0], "[[1,2],[3,4]]");
        assert_eq!(result[1], " {x:[5,6]}");
        assert_eq!(result[2], " max(7,8)");
    }

    #[test]
    fn test_math_pi_not_substring_match() {
        // Regression test: math.pi should not match identifiers like math_pi_value
        let mut props = HashMap::new();
        props.insert("math_pi_value".to_string(), "42".to_string());

        // math_pi_value should remain as a property lookup, not be replaced
        let result = eval_text("${math_pi_value}", &props).unwrap();
        assert_eq!(result, "42");

        // But math.pi should be replaced with pi (pyisheval built-in)
        let result = eval_text("${math.pi * 2}", &props).unwrap();
        let value: f64 = result.parse().unwrap();
        assert!((value - (std::f64::consts::PI * 2.0)).abs() < 1e-9);
    }

    #[test]
    fn test_math_functions_not_in_string_literals() {
        // CRITICAL: Math functions inside string literals should NOT be evaluated
        let props = HashMap::new();

        // Single quoted string literal containing cos(0)
        let result = eval_text("${'Print cos(0)'}", &props).unwrap();
        assert_eq!(
            result, "Print cos(0)",
            "cos(0) in single-quoted string should not be evaluated"
        );

        // Another example with sin
        let result = eval_text("${'The function sin(x) is useful'}", &props).unwrap();
        assert_eq!(
            result, "The function sin(x) is useful",
            "sin(x) in string should not be evaluated"
        );

        // Actual cos(0) usage (not in string) should be evaluated
        let result = eval_text("${cos(0)}", &props).unwrap();
        let value: f64 = result.parse().unwrap();
        assert!(
            (value - 1.0).abs() < 1e-9,
            "cos(0) outside strings should be evaluated to 1"
        );
    }

    #[test]
    fn test_math_pi_not_in_string_literals() {
        // CRITICAL: math.pi inside string literals should NOT be replaced
        let props = HashMap::new();

        // Single quoted string literal containing math.pi
        let result = eval_text("${'Use math.pi for calculations'}", &props).unwrap();
        assert_eq!(
            result, "Use math.pi for calculations",
            "math.pi in single-quoted string should not be replaced"
        );

        // Another single quoted example
        let result = eval_text("${'The constant math.pi is useful'}", &props).unwrap();
        assert_eq!(
            result, "The constant math.pi is useful",
            "math.pi in string should not be replaced"
        );

        // Actual math.pi usage (not in string) should be replaced
        let result = eval_text("${math.pi}", &props).unwrap();
        let value: f64 = result.parse().unwrap();
        assert!(
            (value - std::f64::consts::PI).abs() < 1e-9,
            "math.pi outside strings should be replaced with pi constant"
        );

        // Mixed: comparison with string containing math.pi vs actual math.pi
        let result = eval_text("${'math.pi' == 'math.pi'}", &props).unwrap();
        assert_eq!(result, "1", "String comparison should work");

        let result = eval_text("${math.pi > 3}", &props).unwrap();
        assert_eq!(
            result, "1",
            "Numeric math.pi should be replaced and evaluated"
        );
    }

    #[test]
    fn test_pow_with_nested_function_args() {
        // Regression test: pow(max(1, 2), 3) should handle nested commas correctly
        let props = HashMap::new();

        // This previously failed because split_once(',') would split at the first comma.
        // Now split_args_balanced handles nested function calls correctly.
        // `max` is a pyisheval builtin, so `max(1, 2)` will be evaluated to 2
        // before `pow` is pre-processed.
        let result = eval_text("${pow(max(1, 2), 3)}", &props).unwrap();
        assert_eq!(result, "8");

        // Also test with nested arithmetic expressions
        let result_arith = eval_text("${pow((1 + 1), 3)}", &props).unwrap();
        assert_eq!(result_arith, "8");
    }

    #[test]
    fn test_pow_with_array_indexing() {
        // Regression test: pow([1,2][0], 3) should handle commas inside array literals
        let mut props = HashMap::new();
        props.insert("values".to_string(), "[2,3,4]".to_string());

        // Array indexing with commas inside the array literal
        // This tests that split_args_balanced correctly handles brackets
        let result = eval_text("${pow([1,2][1], 3)}", &props).unwrap();
        assert_eq!(result, "8"); // 2^3 = 8

        // Property containing array literal
        let result = eval_text("${pow(values[0], 3)}", &props).unwrap();
        assert_eq!(result, "8"); // 2^3 = 8
    }

    #[test]
    fn test_custom_namespace_not_hijacked() {
        // Regression test: custom.sin() should not be treated as math function
        let mut props = HashMap::new();
        props.insert("custom".to_string(), "unused".to_string());

        // custom.sin(0) should fail (custom namespace not supported)
        // It should NOT be preprocessed as a math function
        let result = eval_text("${custom.sin(0)}", &props);

        // Should error because custom.sin is not defined, not because we tried to preprocess it
        assert!(result.is_err());
        // The error should be from pyisheval, not from our preprocessing
        let err_msg = format!("{:?}", result.unwrap_err());
        assert!(
            err_msg.contains("UndefinedVar") || err_msg.contains("AttributeError"),
            "Expected undefined variable error, got: {}",
            err_msg
        );
    }

    #[test]
    fn test_context_can_shadow_len_builtin() {
        let mut interp = Interpreter::new();
        let mut properties = HashMap::new();
        properties.insert("len".to_string(), "0.2".to_string());

        // Build context
        let context = build_pyisheval_context(&properties, &mut interp).unwrap();

        // Check that context has "len" with correct value
        assert_eq!(
            context.get("len"),
            Some(&Value::Number(0.2)),
            "len should be 0.2"
        );

        // Try to evaluate an expression with it
        let result = evaluate_expression(&mut interp, "len", &context).unwrap();
        assert_eq!(
            result,
            Some(Value::Number(0.2)),
            "Should return 0.2, not builtin"
        );

        // Try in a real expression
        let result2 = evaluate_expression(&mut interp, "len * 2", &context).unwrap();
        assert_eq!(
            result2,
            Some(Value::Number(0.4)),
            "Should be able to use len in expressions"
        );
    }

    #[test]
    fn test_context_can_shadow_other_builtins() {
        let mut interp = Interpreter::new();
        let mut properties = HashMap::new();
        properties.insert("min".to_string(), "42".to_string());
        properties.insert("max".to_string(), "100".to_string());

        // Build context
        let context = build_pyisheval_context(&properties, &mut interp).unwrap();

        // Check that context has the shadowable built-ins with correct values
        assert_eq!(
            context.get("min"),
            Some(&Value::Number(42.0)),
            "min should be 42.0"
        );
        assert_eq!(
            context.get("max"),
            Some(&Value::Number(100.0)),
            "max should be 100.0"
        );

        // Try to evaluate expressions with them
        let result = evaluate_expression(&mut interp, "min + max", &context).unwrap();
        assert_eq!(
            result,
            Some(Value::Number(142.0)),
            "Should be able to use min and max in expressions"
        );
    }

    // TEST: Type coercion for True/False constants
    #[test]
    fn test_eval_literal_boolean_true() {
        let result = eval_literal("True");
        assert!(
            matches!(result, Value::Number(n) if (n - 1.0).abs() < 1e-10),
            "True should convert to 1.0, got: {:?}",
            result
        );
    }

    #[test]
    fn test_eval_literal_boolean_false() {
        let result = eval_literal("False");
        assert!(
            matches!(result, Value::Number(n) if n.abs() < 1e-10),
            "False should convert to 0.0, got: {:?}",
            result
        );
    }

    #[test]
    fn test_eval_literal_boolean_lowercase_true() {
        let result = eval_literal("true");
        assert!(
            matches!(result, Value::Number(n) if (n - 1.0).abs() < 1e-10),
            "true should convert to 1.0, got: {:?}",
            result
        );
    }

    #[test]
    fn test_eval_literal_boolean_lowercase_false() {
        let result = eval_literal("false");
        assert!(
            matches!(result, Value::Number(n) if n.abs() < 1e-10),
            "false should convert to 0.0, got: {:?}",
            result
        );
    }

    #[test]
    fn test_eval_literal_int() {
        let result = eval_literal("123");
        assert!(
            matches!(result, Value::Number(n) if (n - 123.0).abs() < 1e-10),
            "Integer string should convert to float, got: {:?}",
            result
        );
    }

    #[test]
    fn test_eval_literal_float() {
        let result = eval_literal("3.14");
        assert!(
            matches!(result, Value::Number(n) if (n - 3.14).abs() < 1e-10),
            "Float string should convert to float, got: {:?}",
            result
        );
    }

    #[test]
    fn test_eval_literal_quoted_string() {
        let result = eval_literal("'hello'");
        assert_eq!(
            result,
            Value::StringLit("hello".to_string()),
            "Quoted string should strip quotes"
        );
    }

    #[test]
    fn test_eval_literal_underscore_string() {
        let result = eval_literal("foo_bar");
        assert_eq!(
            result,
            Value::StringLit("foo_bar".to_string()),
            "String with underscore should remain string (likely variable name)"
        );
    }

    #[test]
    fn test_eval_literal_numeric_looking_underscore_string() {
        // Regression test: "36_11" should NOT be parsed as number (which would become 3611)
        // Python xacro explicitly skips numeric parsing for ANY value with underscores
        // to preserve identifiers like tag36_11_00333 in filenames
        let result = eval_literal("36_11");
        assert_eq!(
            result,
            Value::StringLit("36_11".to_string()),
            "Numeric-looking string with underscore should remain string, not be parsed as number"
        );
    }

    #[test]
    fn test_eval_literal_unparseable_string() {
        let result = eval_literal("hello");
        assert_eq!(
            result,
            Value::StringLit("hello".to_string()),
            "Unparseable string should remain string"
        );
    }

    #[test]
    fn test_eval_literal_empty_string() {
        let result = eval_literal("");
        assert_eq!(
            result,
            Value::StringLit("".to_string()),
            "Empty string should remain empty string"
        );
    }

    // Integration test: True/False in properties
    #[test]
    fn test_true_false_in_properties() {
        let mut props = HashMap::new();
        props.insert("flag".to_string(), "True".to_string());
        props.insert("disabled".to_string(), "False".to_string());

        // Test that properties are converted to numbers
        let result = eval_text("${flag}", &props).unwrap();
        assert_eq!(result, "1", "True converts to 1");

        let result = eval_text("${disabled}", &props).unwrap();
        assert_eq!(result, "0", "False converts to 0");

        // Test numeric comparisons (True=1.0, False=0.0)
        let result = eval_text("${flag == 1}", &props).unwrap();
        assert_eq!(result, "1", "True should equal 1 (returns 1 for true)");

        let result = eval_text("${disabled == 0}", &props).unwrap();
        assert_eq!(result, "1", "False should equal 0 (returns 1 for true)");

        // Test boolean in conditional
        let result = eval_text("${1 if flag else 0}", &props).unwrap();
        assert_eq!(result, "1", "True (1.0) should evaluate as truthy");

        let result = eval_text("${1 if disabled else 0}", &props).unwrap();
        assert_eq!(result, "0", "False (0.0) should evaluate as falsy");
    }

    // Integration test: Comparing boolean properties
    #[test]
    fn test_true_false_property_comparison() {
        let mut props = HashMap::new();
        props.insert("enabled".to_string(), "True".to_string());
        props.insert("also_enabled".to_string(), "True".to_string());
        props.insert("disabled".to_string(), "False".to_string());

        // Compare two properties with same value (returns 1 for true comparison)
        let result = eval_text("${enabled == also_enabled}", &props).unwrap();
        assert_eq!(result, "1", "1.0 should equal 1.0");

        // Compare properties with different values (returns 0 for false comparison)
        let result = eval_text("${enabled == disabled}", &props).unwrap();
        assert_eq!(result, "0", "1.0 should not equal 0.0");
    }

    // Tests for pow() function
    #[test]
    fn test_pow_function() {
        let props = HashMap::new();
        let result = eval_text("${pow(2, 3)}", &props).expect("pow should work");
        assert_eq!(result, "8");

        let result = eval_text("${pow(10, 0.5)}", &props).expect("pow with fractional exp");
        let value: f64 = result.parse().expect("parse float");
        assert!((value - 10.0_f64.sqrt()).abs() < 1e-10, "sqrt(10) mismatch");
    }

    // Tests for log() function
    #[test]
    fn test_log_function() {
        let props = HashMap::new();
        let result = eval_text("${log(1)}", &props).expect("log should work");
        assert_eq!(result, "0", "ln(1) = 0");

        // ln(e) should be 1, using the built-in 'e' constant
        let result = eval_text("${log(e)}", &props).expect("log(e)");
        let value: f64 = result.parse().expect("parse float");
        assert!((value - 1.0).abs() < 1e-10, "ln(e) = 1");

        // Test log with base (log(100, 10) = 2)
        let result = eval_text("${log(100, 10)}", &props).expect("log(100, 10)");
        let value: f64 = result.parse().expect("parse float");
        assert!((value - 2.0).abs() < 1e-10, "log_10(100) = 2");

        // Test log with base 2 (log(8, 2) = 3)
        let result = eval_text("${log(8, 2)}", &props).expect("log(8, 2)");
        let value: f64 = result.parse().expect("parse float");
        assert!((value - 3.0).abs() < 1e-10, "log_2(8) = 3");
    }

    // Tests for math. prefix functions
    #[test]
    fn test_math_prefix_functions() {
        let props = HashMap::new();

        // Test math.pow
        let result = eval_text("${math.pow(2, 3)}", &props).expect("math.pow");
        assert_eq!(result, "8");

        // Test math.log
        let result = eval_text("${math.log(1)}", &props).expect("math.log");
        assert_eq!(result, "0");

        // Test math.atan2
        let result = eval_text("${math.atan2(1, 0)}", &props).expect("math.atan2");
        let value: f64 = result.parse().expect("parse float");
        assert!(
            (value - std::f64::consts::FRAC_PI_2).abs() < 1e-10,
            "atan2(1,0) = π/2"
        );

        // Test math.sqrt
        let result = eval_text("${math.sqrt(4)}", &props).expect("math.sqrt");
        assert_eq!(result, "2");
    }

    // Tests for math.pi constant
    #[test]
    fn test_math_pi_constant_access() {
        let props = HashMap::new();

        // Test math.pi
        let result = eval_text("${math.pi}", &props).expect("math.pi");
        let value: f64 = result.parse().expect("parse float");
        assert!((value - std::f64::consts::PI).abs() < 1e-10, "math.pi = π");

        // Test math.pi in expression
        let result = eval_text("${-math.pi / 2}", &props).expect("-math.pi / 2");
        let value: f64 = result.parse().expect("parse float");
        assert!((value + std::f64::consts::FRAC_PI_2).abs() < 1e-10, "-π/2");
    }

    // ========================================================================
    // Tests for load_yaml() functionality
    // ========================================================================

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_nested_dict() {
        let props = HashMap::new();

        // Access nested dict values
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['robot']['chassis']['length']}",
            &props,
        )
        .expect("load_yaml nested access should succeed");

        assert_eq!(value, "0.5", "chassis length should be 0.5");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_with_xacro_prefix() {
        let props = HashMap::new();

        // Test xacro.load_yaml() syntax
        let value = eval_text(
            "${xacro.load_yaml('tests/data/test_config.yaml')['count']}",
            &props,
        )
        .expect("xacro.load_yaml should succeed");

        assert_eq!(value, "5", "count should be 5");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_array_access() {
        let props = HashMap::new();

        // Access array elements
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['joints'][0]}",
            &props,
        )
        .expect("load_yaml array access should succeed");

        assert_eq!(value, "joint1", "first joint should be joint1");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_deep_nesting() {
        let props = HashMap::new();

        // Access deeply nested value
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['nested']['level1']['level2']['value']}",
            &props,
        )
        .expect("load_yaml deep nesting should succeed");

        assert_eq!(value, "deep_value", "deep nested value should match");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_in_arithmetic() {
        let props = HashMap::new();

        // Use loaded value in arithmetic expression
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['robot']['wheel']['radius'] * 2}",
            &props,
        )
        .expect("load_yaml in arithmetic should succeed");

        assert_eq!(value, "0.2", "radius * 2 should be 0.2");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_multiple_calls() {
        let props = HashMap::new();

        // Multiple load_yaml calls in same expression
        let value = eval_text(
            "${load_yaml('tests/data/test_config.yaml')['robot']['chassis']['length'] + \
             load_yaml('tests/data/test_config.yaml')['robot']['chassis']['width']}",
            &props,
        )
        .expect("multiple load_yaml calls should succeed");

        assert_eq!(value, "0.8", "0.5 + 0.3 should be 0.8");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_extract_and_store() {
        let mut props = HashMap::new();
        let mut interp = init_interpreter();

        // Extract a specific value from YAML and store it
        let wheel_base = eval_text_with_interpreter(
            "${load_yaml('tests/data/test_config.yaml')['robot']['wheel']['base']}",
            &props,
            &mut interp,
        )
        .expect("load_yaml should succeed");

        // Store the extracted value
        props.insert("wheel_base".to_string(), wheel_base);

        // Now use the stored value in calculations
        let value = eval_text_with_interpreter("${wheel_base * 2}", &props, &mut interp)
            .expect("stored value calculation should succeed");

        assert_eq!(value, "0.8", "wheel_base * 2 should be 0.8");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_file_not_found() {
        let props = HashMap::new();

        // Try to load non-existent file
        let result = eval_text("${load_yaml('tests/data/nonexistent.yaml')}", &props);

        assert!(result.is_err(), "should error on missing file");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("Failed to load YAML") || err_msg.contains("No such file"),
            "error should mention file loading failure, got: {}",
            err_msg
        );
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_invalid_yaml() {
        use std::io::Write;
        use tempfile::NamedTempFile;

        // Create temporary file with invalid YAML
        let mut temp_file = NamedTempFile::new().expect("create temp file");
        write!(temp_file, "invalid: yaml:\n  - bad\n  syntax").expect("write temp file");
        let temp_path = temp_file.path().to_string_lossy().replace('\\', "/");

        let props = HashMap::new();
        let result = eval_text(&format!("${{load_yaml('{}')}}", temp_path), &props);

        assert!(result.is_err(), "should error on invalid YAML");
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("Failed to parse YAML") || err_msg.contains("parse"),
            "error should mention YAML parsing failure, got: {}",
            err_msg
        );
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_with_property_filename() {
        let mut props = HashMap::new();
        props.insert(
            "config_file".to_string(),
            "tests/data/test_config.yaml".to_string(),
        );

        // Variable filenames are now supported
        let value = eval_text("${load_yaml(config_file)['count']}", &props)
            .expect("variable filename should work");

        assert_eq!(value, "5", "count should be 5");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_argument_with_parentheses_in_string() {
        use std::io::Write;
        use tempfile::Builder;

        let props = HashMap::new();

        // Create a temp YAML file whose path literal includes parentheses
        // This truly tests that find_matching_paren handles parens in the argument
        let mut temp = Builder::new()
            .prefix("config(")
            .suffix(").yaml")
            .tempfile()
            .expect("create temp yaml");
        write!(temp, "robot:\n  chassis:\n    width: 0.3\n").expect("write temp yaml");
        let path = temp.path().to_string_lossy().replace('\\', "/");

        // The fix ensures we use find_matching_paren instead of regex capture [^()]+?
        // This allows proper handling when the argument contains parentheses
        let expr = format!("${{load_yaml('{}')['robot']['chassis']['width']}}", path);
        let value = eval_text(&expr, &props).expect("load_yaml argument parsing should succeed");

        assert_eq!(
            value, "0.3",
            "should correctly parse load_yaml argument even with potential paren complexity"
        );
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_null_value() {
        use std::io::Write;
        use tempfile::NamedTempFile;

        let mut temp_file = NamedTempFile::new().expect("create temp file");
        write!(temp_file, "~").expect("write temp file");
        let temp_path = temp_file.path().to_string_lossy().replace('\\', "/");

        let props = HashMap::new();
        let value = eval_text(&format!("${{load_yaml('{}') + 5}}", &temp_path), &props)
            .expect("load_yaml with null should succeed");
        assert_eq!(value, "5", "null (None) + 5 should be 5");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_null_in_dict() {
        use std::io::Write;
        use tempfile::NamedTempFile;

        let mut temp_file = NamedTempFile::new().expect("create temp file");
        write!(temp_file, "value: null\nother: 10").expect("write temp file");
        let temp_path = temp_file.path().to_string_lossy().replace('\\', "/");

        let props = HashMap::new();
        let value = eval_text(
            &format!("${{load_yaml('{}')['value']}}", &temp_path),
            &props,
        )
        .expect("load_yaml null value access should succeed");
        assert_eq!(value, "0", "null value should evaluate to 0 (None)");
    }

    #[cfg(feature = "yaml")]
    #[test]
    fn test_load_yaml_inf_nan_values() {
        let props = HashMap::new();

        // Test positive infinity - evaluates to Python inf
        let result = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['positive_inf']}",
            &props,
        );
        // Python's float('inf') evaluates to inf, which is a valid value
        assert!(
            result.is_ok(),
            "positive_inf should evaluate successfully, got: {:?}",
            result
        );

        // Test negative infinity
        let result = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['negative_inf']}",
            &props,
        );
        assert!(result.is_ok(), "negative_inf should evaluate successfully");

        // Test NaN
        let result = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['not_a_number']}",
            &props,
        );
        assert!(result.is_ok(), "not_a_number should evaluate successfully");

        // Test normal float still works
        let value = eval_text(
            "${load_yaml('tests/data/test_inf_nan.yaml')['normal_float']}",
            &props,
        )
        .expect("normal_float should succeed");
        assert_eq!(value, "3.14", "normal float should be '3.14'");
    }
}

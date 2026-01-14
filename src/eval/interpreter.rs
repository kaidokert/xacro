use super::lexer::{Lexer, TokenType};
use super::scope::BUILTIN_CONSTANTS;
use pyisheval::{Interpreter, Value};
use regex::Regex;
use std::collections::HashMap;
use std::sync::OnceLock;

/// Evaluate string literal to appropriate type (Python xacro compatibility)
///
/// Implements Python xacro's `Table._eval_literal()` type coercion logic
/// (ref/xacro/src/xacro/__init__.py:333-349).
///
/// Conversion order (same as Python xacro):
/// 1. Strip surrounding single quotes from quoted strings (e.g., "'hello'" → "hello")
/// 2. Skip strings with underscores (likely variable names, not literals)
/// 3. Try numeric parsing (e.g., "123" → 123.0, "3.14" → 3.14)
/// 4. Try boolean parsing (e.g., "True" → 1.0, "False" → 0.0)
/// 5. Fall back to string
///
/// # Arguments
/// * `value` - String value to convert
///
/// # Returns
/// Converted value as pyisheval::Value
///
/// # Examples
/// ```text
/// eval_literal("123") → Value::Number(123.0)
/// eval_literal("3.14") → Value::Number(3.14)
/// eval_literal("True") → Value::Number(1.0)
/// eval_literal("False") → Value::Number(0.0)
/// eval_literal("hello") → Value::StringLit("hello")
/// eval_literal("'quoted'") → Value::StringLit("quoted")  // strip quotes
/// ```
fn eval_literal(value: &str) -> Value {
    let value = value.trim();

    // Strip surrounding single quotes from quoted strings
    if let Some(unquoted) = value.strip_prefix('\'').and_then(|s| s.strip_suffix('\'')) {
        return Value::StringLit(unquoted.to_string());
    }

    // Skip strings with underscores (likely variable names, not literals)
    if value.contains('_') {
        return Value::StringLit(value.to_string());
    }

    // Try float parsing (handles both integers and floats)
    if let Ok(f) = value.parse::<f64>() {
        return Value::Number(f);
    }

    // Try boolean (matches Python xacro's get_boolean_value logic)
    match value {
        "True" | "true" => Value::Number(1.0),
        "False" | "false" => Value::Number(0.0),
        _ => Value::StringLit(value.to_string()),
    }
}

/// Find matching closing parenthesis, handling nested parentheses
///
/// # Arguments
/// * `text` - String to search
/// * `start` - Byte index of opening '('
///
/// # Returns
/// Byte index of matching ')', or None if not found
///
/// Note: Uses byte-based iteration since parentheses are ASCII characters
/// and will never appear as continuation bytes in UTF-8.
fn find_matching_paren(
    text: &str,
    start: usize,
) -> Option<usize> {
    let bytes = text.as_bytes();
    if start >= bytes.len() || bytes[start] != b'(' {
        return None;
    }

    let mut depth = 0;
    for (i, &ch) in bytes.iter().enumerate().skip(start) {
        match ch {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return Some(i);
                }
            }
            _ => {}
        }
    }
    None
}

/// List of supported math functions for preprocessing
///
/// These functions are preprocessed before expression evaluation since pyisheval
/// doesn't provide native math functions. Functions are ordered by length (longest first)
/// for defensive regex alternation, though with word boundaries this isn't strictly necessary.
///
/// Note: `radians()` and `degrees()` are NOT in this list because they are implemented as
/// lambda functions in pyisheval (see `init_interpreter()`), not as Rust native functions.
pub(crate) const SUPPORTED_MATH_FUNCS: &[&str] = &[
    "floor", "acos", "asin", "atan", "ceil", "sqrt", "cos", "sin", "tan", "abs",
];

/// Regex pattern for matching math function calls with word boundaries
static MATH_FUNCS_REGEX: OnceLock<Regex> = OnceLock::new();

/// Get the math functions regex, initializing it on first access
///
/// Matches function names at word boundaries followed by optional whitespace and '('.
fn get_math_funcs_regex() -> &'static Regex {
    MATH_FUNCS_REGEX.get_or_init(|| {
        // Use \b for word boundaries, capture function name, allow optional whitespace before '('
        let pattern = format!(r"\b({})\s*\(", SUPPORTED_MATH_FUNCS.join("|"));
        Regex::new(&pattern).expect("Math functions regex should be valid")
    })
}

/// Preprocess an expression to evaluate native math functions
///
/// pyisheval doesn't support native math functions like cos(), sin(), tan().
/// This function finds math function calls, evaluates them using Rust's f64 methods,
/// and substitutes the results back into the expression.
///
/// Supported functions: cos, sin, tan, acos, asin, atan, sqrt, abs, floor, ceil
///
/// # Limitations
/// **Does not distinguish function calls inside string literals** (e.g., `'cos(0)'`).
/// This can cause incorrect evaluation: an expression like `'cos(0)'` will be
/// preprocessed to `'1.0'` instead of remaining as the string `"cos(0)"`.
/// A full fix would require a proper parser to track string literal context.
/// For now, users should avoid math function names inside string literals.
///
/// # Arguments
/// * `expr` - Expression that may contain math function calls
/// * `interp` - Interpreter for evaluating function arguments
///
/// # Returns
/// Expression with math function calls replaced by their computed values
fn preprocess_math_functions(
    expr: &str,
    interp: &mut Interpreter,
) -> Result<String, EvalError> {
    let mut result = expr.to_string();

    // Keep replacing until no more matches (handle nested calls from inside out)
    let mut iteration = 0;
    const MAX_ITERATIONS: usize = 100;

    loop {
        iteration += 1;
        if iteration > MAX_ITERATIONS {
            return Err(EvalError::PyishEval {
                expr: expr.to_string(),
                source: pyisheval::EvalError::ParseError(
                    "Too many nested math function calls (possible infinite loop)".to_string(),
                ),
            });
        }

        // Collect all function matches to iterate right-to-left (innermost first)
        let captures: Vec<_> = get_math_funcs_regex().captures_iter(&result).collect();
        if captures.is_empty() {
            break;
        }

        let mut made_replacement = false;
        // Iterate from right to left to find the innermost, evaluatable function call
        for caps in captures.iter().rev() {
            // Safe extraction of capture groups (should always succeed due to regex structure)
            let (whole_match, func_name) = match (caps.get(0), caps.get(1)) {
                (Some(m), Some(f)) => (m, f.as_str()),
                _ => continue, // Skip malformed captures
            };
            let paren_pos = whole_match.end() - 1; // Position of '(' after optional whitespace

            // Find matching closing parenthesis
            let close_pos = match find_matching_paren(&result, paren_pos) {
                Some(pos) => pos,
                None => continue, // Skip if no matching paren
            };

            let arg = &result[paren_pos + 1..close_pos];

            // Try to evaluate the argument - only replace if successful
            if let Ok(Value::Number(n)) = interp.eval(arg) {
                // Validate domain for inverse trig functions (matches Python xacro behavior)
                if (func_name == "acos" || func_name == "asin") && !(-1.0..=1.0).contains(&n) {
                    log::warn!(
                        "{}({}) domain error: argument must be in [-1, 1], got {}",
                        func_name,
                        arg,
                        n
                    );
                    continue; // Skip this match, try next one
                }

                // Call the appropriate Rust math function
                let computed = match func_name {
                    "cos" => n.cos(),
                    "sin" => n.sin(),
                    "tan" => n.tan(),
                    "acos" => n.acos(),
                    "asin" => n.asin(),
                    "atan" => n.atan(),
                    "sqrt" => n.sqrt(),
                    "abs" => n.abs(),
                    "floor" => n.floor(),
                    "ceil" => n.ceil(),
                    _ => unreachable!(
                        "Function '{}' matched regex but not in match statement",
                        func_name
                    ),
                };

                let replacement = format!("{}", computed);
                result.replace_range(whole_match.start()..=close_pos, &replacement);
                made_replacement = true;
                // A replacement was made, restart loop to rescan the string
                break;
            }
            // If eval fails or returns non-number, continue to next match
        }

        if !made_replacement {
            // No successful replacements possible, done processing
            break;
        }
    }

    Ok(result)
}

/// Initialize an Interpreter with builtin constants and math functions
///
/// This ensures all interpreters have access to:
/// - Math constants: pi, e, tau, M_PI
/// - Math functions: radians(), degrees()
///
/// Note: inf and nan are NOT initialized here - they are injected directly into
/// the context HashMap in build_pyisheval_context() to bypass parsing issues.
///
/// Note: Native math functions (cos, sin, tan, etc.) are handled via preprocessing
/// in evaluate_expression() rather than as lambda functions, because pyisheval
/// cannot call native Rust functions.
///
/// # Returns
/// A fully initialized Interpreter ready for expression evaluation
pub fn init_interpreter() -> Interpreter {
    let mut interp = Interpreter::new();

    // Initialize math constants in the interpreter
    // These are loaded directly into the interpreter's environment for use in expressions
    // Note: inf and nan are NOT in BUILTIN_CONSTANTS (pyisheval can't parse them as literals)
    // They are injected directly into the context map in build_pyisheval_context()
    for (name, value) in BUILTIN_CONSTANTS {
        if let Err(e) = interp.eval(&format!("{} = {}", name, value)) {
            log::warn!(
                "Could not initialize built-in constant '{}': {}. \
                 This constant will not be available in expressions.",
                name,
                e
            );
        }
    }

    // Add math conversion functions as lambda expressions directly in the interpreter
    // This makes them available as callable functions in all expressions
    if let Err(e) = interp.eval("radians = lambda x: x * pi / 180") {
        log::warn!(
            "Could not define built-in function 'radians': {}. \
             This function will not be available in expressions. \
             (May be due to missing 'pi' constant)",
            e
        );
    }
    if let Err(e) = interp.eval("degrees = lambda x: x * 180 / pi") {
        log::warn!(
            "Could not define built-in function 'degrees': {}. \
             This function will not be available in expressions. \
             (May be due to missing 'pi' constant)",
            e
        );
    }

    interp
}

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

/// Format a pyisheval Value to match Python xacro's output format
///
/// Python xacro uses Python's int/float distinction for formatting:
/// - Integer arithmetic (2+3) produces int → formats as "5" (no decimal)
/// - Float arithmetic (2.5*2) produces float → formats as "5.0" (with decimal)
///
/// Since pyisheval only has f64 (no int type), we approximate this by checking
/// if the f64 value is mathematically a whole number using fract() == 0.0.
///
/// # Arguments
/// * `value` - The pyisheval Value to format
/// * `force_float` - Whether to keep .0 for whole numbers (true for float context)
pub fn format_value_python_style(
    value: &Value,
    force_float: bool,
) -> String {
    match value {
        Value::Number(n) if n.is_finite() => {
            // Python's str() for floats switches to scientific notation at 1e16
            const PYTHON_SCIENTIFIC_THRESHOLD: f64 = 1e16;

            if n.fract() == 0.0 && n.abs() < PYTHON_SCIENTIFIC_THRESHOLD {
                // Whole number
                if force_float {
                    // Float context: keep .0 for whole numbers
                    format!("{:.1}", n) // "1.0" not "1"
                } else {
                    // Int context: strip .0
                    format!("{:.0}", n) // "1" not "1.0"
                }
            } else {
                // Has fractional part or is a large number: use default formatting
                n.to_string()
            }
        }
        _ => value.to_string(),
    }
}

/// Remove quotes from string values (handles both single and double quotes)
pub fn remove_quotes(s: &str) -> &str {
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
    let mut interp = init_interpreter();
    eval_text_with_interpreter(text, properties, &mut interp)
}

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
    for (name, value) in properties.iter() {
        let trimmed = value.trim();
        if !trimmed.starts_with("lambda ") {
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
                    // Escape backslashes first, then single quotes (order matters!)
                    // This handles Windows paths (C:\Users), regex patterns, etc.
                    let escaped_value = s.replace('\\', "\\\\").replace('\'', "\\'");
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
                    // Other value types (shouldn't happen with eval_literal, but be safe)
                    log::warn!("Unexpected value type for property '{}': {:?}", name, value);
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
                    Ok(evaluated_value) => Ok((name.clone(), evaluated_value)),
                    Err(e) => {
                        // Distinguish expected parse failures from unexpected runtime errors
                        use pyisheval::EvalError as PyEvalError;
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
                Ok((name.clone(), eval_literal(value)))
            }
        })
        .collect::<Result<HashMap<_, _>, _>>()?;

    // Manually inject inf and nan constants (Strategy 3: bypass parsing)
    // Python xacro provides these via float('inf') and math.inf, but they're also
    // used as bare identifiers in expressions. Pyisheval cannot parse these as
    // literals, so we inject them directly into the context.
    context.insert("inf".to_string(), Value::Number(f64::INFINITY));
    context.insert("nan".to_string(), Value::Number(f64::NAN));

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
pub fn evaluate_expression(
    interp: &mut Interpreter,
    expr: &str,
    context: &HashMap<String, Value>,
) -> Result<Option<Value>, pyisheval::EvalError> {
    let trimmed_expr = expr.trim();
    if trimmed_expr == "xacro.print_location()" {
        // Special case: stub debug function returns no output
        return Ok(None);
    }

    // Preprocess math functions (cos, sin, tan, etc.) before evaluation
    // This converts native math calls into computed values since pyisheval
    // doesn't support calling native Rust functions
    let preprocessed = preprocess_math_functions(expr, interp).map_err(|e| match e {
        EvalError::PyishEval { source, .. } => source,
        _ => pyisheval::EvalError::ParseError(e.to_string()),
    })?;

    interp.eval_with_context(&preprocessed, context).map(Some)
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
                match evaluate_expression(interp, &token_value, &context) {
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

        // NOTE: pyisheval v0.9.0 does NOT support `and`/`or` operators
        // These would fail: ${1 and 1}, ${x > 3 and y < 15}
        // Fortunately, real xacro files don't use these - they use simpler expressions

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
        use pyisheval::Value;

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
        use pyisheval::Value;

        // Fractional numbers use default formatting (no trailing zeros)
        assert_eq!(format_value_python_style(&Value::Number(1.5), false), "1.5");
        assert_eq!(format_value_python_style(&Value::Number(0.5), false), "0.5");
        assert_eq!(
            format_value_python_style(&Value::Number(0.4235294117647059), false),
            "0.4235294117647059"
        );
    }

    #[test]
    fn test_format_value_python_style_special() {
        use pyisheval::Value;

        // Special values
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
            let expr = format!("${{{}(0)}}", func);
            let result = eval_text(&expr, &props);

            // Ensure evaluation succeeds - unreachable!() would panic if function is missing
            result.expect("Evaluation should succeed for all supported math functions");
        }
    }

    #[test]
    fn test_context_can_shadow_len_builtin() {
        use std::collections::HashMap;

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
        use std::collections::HashMap;

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
}

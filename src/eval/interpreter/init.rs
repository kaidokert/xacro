use super::constants::BUILTIN_CONSTANTS;
use pyisheval::{Interpreter, Value};

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
pub(crate) fn init_interpreter() -> Interpreter {
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
pub(crate) fn format_value_python_style(
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

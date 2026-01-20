use super::parsing::{
    find_matching_paren, split_args_balanced, DelimiterTracker, SUPPORTED_MATH_FUNCS,
};
use pyisheval::{Interpreter, Value};
use std::collections::HashMap;

/// Replace math.pi constants with pi outside string literals
///
/// Scans the expression and replaces `math.pi` with `pi` at word boundaries,
/// respecting string literal boundaries using DelimiterTracker.
fn replace_math_pi_constants(expr: &str) -> String {
    let bytes = expr.as_bytes();
    let mut result = String::with_capacity(expr.len());
    let mut tracker = DelimiterTracker::new();
    let mut i = 0;

    while i < bytes.len() {
        // Check for math.pi (7 bytes)
        if i + 7 <= bytes.len() && &bytes[i..i + 7] == b"math.pi" {
            // Verify it's at a word boundary (not part of larger identifier)
            let after_ok = i + 7 >= bytes.len()
                || !bytes[i + 7].is_ascii_alphanumeric() && bytes[i + 7] != b'_';
            let before_ok = i == 0 || !bytes[i - 1].is_ascii_alphanumeric() && bytes[i - 1] != b'_';

            // Only replace if we're not in a string literal and at word boundary
            if !tracker.in_string() && before_ok && after_ok {
                result.push_str("pi");
                i += 7; // Skip "math.pi"
                continue;
            }
        }

        // Track delimiter state and append character
        let ch = bytes[i];
        tracker.process(ch);
        result.push(ch as char);
        i += 1;
    }

    result
}

/// Scan for math function calls in an expression
///
/// Returns a list of (match_start, func_name, paren_pos) tuples for all
/// math function calls found outside string literals.
fn scan_math_functions(expr: &str) -> Vec<(usize, &'static str, usize)> {
    let result_bytes = expr.as_bytes();
    let mut function_matches = Vec::new();
    let mut scan_tracker = DelimiterTracker::new();
    let mut i = 0;

    while i < result_bytes.len() {
        scan_tracker.process(result_bytes[i]);

        log::trace!(
            "[scan] At pos {}, char: {}, in_string: {}",
            i,
            result_bytes[i] as char,
            scan_tracker.in_string()
        );

        // Only look for functions when not inside string literals
        if scan_tracker.in_string() {
            i += 1;
            continue;
        }

        // Check for optional "math." prefix
        let has_prefix = i + 5 <= result_bytes.len() && &result_bytes[i..i + 5] == b"math.";
        let func_start = if has_prefix { i + 5 } else { i };

        // Check word boundary before function name
        if func_start > 0 {
            let prev_ch = result_bytes[func_start - 1];
            // Skip if part of larger identifier OR custom namespace (e.g., custom.sin)
            if prev_ch.is_ascii_alphanumeric()
                || prev_ch == b'_'
                || (!has_prefix && prev_ch == b'.')
            {
                i += 1;
                continue;
            }
        }

        // Try to match supported functions
        let mut found = None;
        for &func in SUPPORTED_MATH_FUNCS {
            let func_bytes = func.as_bytes();
            let func_end = func_start + func_bytes.len();

            if func_end <= result_bytes.len() && &result_bytes[func_start..func_end] == func_bytes {
                log::trace!("[scan] Potential match '{}' at pos {}", func, func_start);

                // Look for '(' after function name (skip whitespace)
                let mut paren_pos = func_end;
                while paren_pos < result_bytes.len()
                    && result_bytes[paren_pos].is_ascii_whitespace()
                {
                    paren_pos += 1;
                }

                if paren_pos < result_bytes.len() && result_bytes[paren_pos] == b'(' {
                    log::trace!("[scan] Confirmed match '{}(' at pos {}", func, func_start);
                    found = Some((func, paren_pos, i));
                    break;
                } else {
                    log::trace!(
                        "[scan] No '(' after '{}' at pos {}, char at paren_pos: {:?}",
                        func,
                        func_start,
                        result_bytes.get(paren_pos).map(|&b| b as char)
                    );
                }
            }
        }

        if let Some((func_name, paren_pos, match_start)) = found {
            function_matches.push((match_start, func_name, paren_pos));
            i = paren_pos + 1;
        } else {
            i += 1;
        }
    }

    function_matches
}

/// Preprocess an expression to evaluate native math functions
///
/// pyisheval doesn't support native math functions like cos(), sin(), tan().
/// This function finds math function calls, evaluates them using Rust's f64 methods,
/// and substitutes the results back into the expression.
///
/// Supported functions: cos, sin, tan, acos, asin, atan, atan2, sqrt, abs, floor, ceil, pow, log
/// All functions can be called with or without `math.` prefix (e.g., `cos(x)` or `math.cos(x)`).
/// Also handles `math.pi` constant by replacing with `pi` (only outside string literals).
///
/// Uses DelimiterTracker to respect string boundaries for all replacements.
///
/// # Arguments
/// * `expr` - Expression that may contain math function calls
/// * `interp` - Interpreter for evaluating function arguments
///
/// # Returns
/// Expression with math function calls replaced by their computed values
pub(super) fn preprocess_math_functions(
    expr: &str,
    interp: &mut Interpreter,
    context: &HashMap<String, Value>,
) -> Result<String, super::EvalError> {
    // Local macro to reduce duplication in argument evaluation
    macro_rules! eval_math_arg {
        ($interp:expr, $context:expr, $arg_expr:expr, $func_name:expr, $arg_template:expr) => {
            match $interp.eval_with_context($arg_expr, $context) {
                Ok(Value::Number(n)) => n,
                Ok(val) => {
                    // Non-numeric value - log warning with actual value for debugging
                    log::warn!(
                        "[eval_math_arg] Expected numeric argument for {}({}), got {:?}. Skipping this match.",
                        $func_name, $arg_expr, val
                    );
                    continue;
                }
                Err(e) => {
                    // Argument evaluation failed - propagate error instead of masking it
                    return Err(super::EvalError::PyishEval {
                        expr: format!($arg_template, $func_name, $arg_expr),
                        source: e,
                    });
                }
            }
        };
    }

    // First pass: Replace math.pi with pi
    let mut result = replace_math_pi_constants(expr);

    // Keep replacing until no more matches (handle nested calls from inside out)
    let mut iteration = 0;
    const MAX_ITERATIONS: usize = 100;

    loop {
        iteration += 1;
        if iteration > MAX_ITERATIONS {
            return Err(super::EvalError::PyishEval {
                expr: expr.to_string(),
                source: pyisheval::EvalError::ParseError(
                    "Too many nested math function calls (possible infinite loop)".to_string(),
                ),
            });
        }

        // Find all function matches
        let function_matches = scan_math_functions(&result);

        if log::log_enabled!(log::Level::Debug) {
            // Avoid logging excessively large expressions: log length and a truncated preview
            let result_char_len = result.chars().count();
            let max_preview_chars = 200;
            let preview: String = result.chars().take(max_preview_chars).collect();
            let truncated = result_char_len > max_preview_chars;

            log::debug!(
                "[preprocess_math_functions] Found {} function matches (len={}{}). Preview: {}",
                function_matches.len(),
                result_char_len,
                if truncated { ", truncated" } else { "" },
                preview,
            );
        }

        if function_matches.is_empty() {
            break;
        }

        let mut made_replacement = false;
        // Process matches right-to-left (innermost first)
        for &(match_start, func_name, paren_pos) in function_matches.iter().rev() {
            // Find matching closing parenthesis
            let close_pos = match find_matching_paren(&result, paren_pos) {
                Some(pos) => pos,
                None => continue, // Skip if no matching paren
            };

            let arg = &result[paren_pos + 1..close_pos];

            // Special handling for 2-argument functions (atan2, pow, log)
            // Note: log can take 1 or 2 arguments (natural log or log with base)
            if func_name == "atan2" || func_name == "pow" || func_name == "log" {
                log::debug!(
                    "[preprocess_math_functions] Found {}({}) at pos {}",
                    func_name,
                    arg,
                    match_start
                );

                // Split arguments on comma while respecting nested parentheses
                // This handles cases like pow(max(1, 2), 3) correctly
                let args = split_args_balanced(arg);

                // Handle 2-argument case
                if args.len() == 2 {
                    log::debug!(
                        "[preprocess_math_functions] Evaluating {} arg1: '{}' with context: {:?}",
                        func_name,
                        args[0].trim(),
                        context.keys().collect::<Vec<_>>()
                    );

                    // Evaluate first argument - propagate errors immediately
                    let first =
                        eval_math_arg!(interp, context, args[0].trim(), func_name, "{}({}, ...)");

                    // Evaluate second argument - propagate errors immediately
                    let second =
                        eval_math_arg!(interp, context, args[1].trim(), func_name, "{}(..., {})");

                    let computed = match func_name {
                        "atan2" => first.atan2(second),
                        "pow" => first.powf(second),
                        "log" => first.log(second), // log(x, base)
                        _ => unreachable!(),
                    };
                    let replacement = format!("{}", computed);
                    result.replace_range(match_start..=close_pos, &replacement);
                    made_replacement = true;
                    break;
                }

                // Validate argument count based on function requirements
                if func_name == "log" {
                    // log() accepts 1 or 2 arguments
                    if args.len() == 1 {
                        // Fall through to single-arg handling below (natural logarithm)
                    } else if args.len() != 2 {
                        log::warn!("log() expects 1 or 2 arguments, but got {}.", args.len());
                        continue;
                    }
                } else {
                    // atan2() and pow() require exactly 2 arguments
                    if args.len() != 2 {
                        log::warn!(
                            "{}() expects 2 arguments, but got {}.",
                            func_name,
                            args.len()
                        );
                        continue;
                    }
                }
            }

            // Evaluate the argument with context - propagate errors immediately
            let n = eval_math_arg!(interp, context, arg, func_name, "{}({})");

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
                "log" => n.ln(), // Natural logarithm (Python's math.log defaults to ln)
                _ => unreachable!(
                    "Function '{}' in SUPPORTED_MATH_FUNCS but not in match statement",
                    func_name
                ),
            };

            let replacement = format!("{}", computed);
            result.replace_range(match_start..=close_pos, &replacement);
            made_replacement = true;
            // A replacement was made, restart loop to rescan the string
            break;
        }

        if !made_replacement {
            // No successful replacements possible, done processing
            break;
        }
    }

    log::debug!("[preprocess_math_functions] Output: {}", result);
    Ok(result)
}

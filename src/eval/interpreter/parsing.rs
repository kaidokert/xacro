use pyisheval::Value;

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
pub(super) fn eval_literal(value: &str) -> Value {
    let value = value.trim();

    // Strip surrounding single quotes from quoted strings
    if let Some(unquoted) = value.strip_prefix('\'').and_then(|s| s.strip_suffix('\'')) {
        return Value::StringLit(unquoted.to_string());
    }

    // Try float parsing (handles both integers and floats, including with underscores)
    // Python allows numeric literals with underscores like 1_000 or 1_000_000
    // PEP 515: Underscores must be between digits only (no leading/trailing/consecutive)
    if value.contains('_') {
        // Validate underscore placement per PEP 515
        let bytes = value.as_bytes();
        let mut valid = true;
        for i in 0..bytes.len() {
            if bytes[i] == b'_' {
                // Leading/trailing underscore is invalid
                if i == 0 || i + 1 == bytes.len() {
                    valid = false;
                    break;
                }
                // Consecutive underscores or underscores not between digits are invalid
                // Note: This simplified check requires underscores between ASCII digits
                // More complex cases (hex, binary, after exponent) are rare in xacro
                if !bytes[i - 1].is_ascii_digit() || !bytes[i + 1].is_ascii_digit() {
                    valid = false;
                    break;
                }
            }
        }
        if !valid {
            // Invalid underscore placement - treat as variable name
            return Value::StringLit(value.to_string());
        }
    }

    // Rust's f64::parse() doesn't support underscores, so strip them first
    let numeric_candidate = value.replace('_', "");
    if let Ok(f) = numeric_candidate.parse::<f64>() {
        return Value::Number(f);
    }

    // Skip strings with underscores that weren't valid numbers
    if value.contains('_') {
        return Value::StringLit(value.to_string());
    }

    // Try boolean (matches Python xacro's get_boolean_value logic)
    // Case-insensitive matching to handle true, True, TRUE, etc.
    if value.eq_ignore_ascii_case("true") {
        Value::Number(1.0)
    } else if value.eq_ignore_ascii_case("false") {
        Value::Number(0.0)
    } else {
        Value::StringLit(value.to_string())
    }
}

/// Tracks delimiter depth and quote state while parsing expressions
///
/// Maintains nesting depth for parentheses, brackets, and braces,
/// along with quote state and escape handling. Used to find matching
/// delimiters and split arguments correctly.
pub(super) struct DelimiterTracker {
    paren_depth: usize,
    bracket_depth: usize,
    brace_depth: usize,
    in_single_quote: bool,
    in_double_quote: bool,
    escape_next: bool,
}

impl DelimiterTracker {
    pub(super) fn new() -> Self {
        Self {
            paren_depth: 0,
            bracket_depth: 0,
            brace_depth: 0,
            in_single_quote: false,
            in_double_quote: false,
            escape_next: false,
        }
    }

    /// Process a byte, updating internal state
    pub(super) fn process(
        &mut self,
        ch: u8,
    ) {
        // Handle escape sequences - next character is literal
        if self.escape_next {
            self.escape_next = false;
            return;
        }

        match ch {
            b'\\' => self.escape_next = true,
            b'\'' if !self.in_double_quote => self.in_single_quote = !self.in_single_quote,
            b'"' if !self.in_single_quote => self.in_double_quote = !self.in_double_quote,
            b'(' if !self.in_single_quote && !self.in_double_quote => self.paren_depth += 1,
            b')' if !self.in_single_quote && !self.in_double_quote && self.paren_depth > 0 => {
                self.paren_depth -= 1
            }
            b'[' if !self.in_single_quote && !self.in_double_quote => self.bracket_depth += 1,
            b']' if !self.in_single_quote && !self.in_double_quote && self.bracket_depth > 0 => {
                self.bracket_depth -= 1
            }
            b'{' if !self.in_single_quote && !self.in_double_quote => self.brace_depth += 1,
            b'}' if !self.in_single_quote && !self.in_double_quote && self.brace_depth > 0 => {
                self.brace_depth -= 1
            }
            _ => {}
        }
    }

    /// Check if we're at top level (no nesting, no quotes)
    pub(super) fn at_top_level(&self) -> bool {
        self.paren_depth == 0
            && self.bracket_depth == 0
            && self.brace_depth == 0
            && !self.in_single_quote
            && !self.in_double_quote
    }

    /// Check if we're inside string literals (quotes only, not structural delimiters)
    ///
    /// Used for math function preprocessing where we want to process functions inside
    /// arithmetic parentheses like `m*(3*pow(r,2))` but skip functions inside strings
    /// like `'print pow(2,3)'`.
    pub(super) fn in_string(&self) -> bool {
        self.in_single_quote || self.in_double_quote
    }
}

/// Find matching closing parenthesis, handling nested delimiters
///
/// Handles nested parentheses, brackets, braces, quotes, and escape sequences.
///
/// # Arguments
/// * `text` - String to search
/// * `start` - Byte index of opening '('
///
/// # Returns
/// Byte index of matching ')', or None if not found
///
/// Note: Uses byte-based iteration since delimiters are ASCII characters
/// and will never appear as continuation bytes in UTF-8.
pub fn find_matching_paren(
    text: &str,
    start: usize,
) -> Option<usize> {
    let bytes = text.as_bytes();
    if start >= bytes.len() || bytes[start] != b'(' {
        return None;
    }

    let mut tracker = DelimiterTracker::new();

    for (i, &ch) in bytes.iter().enumerate().skip(start) {
        tracker.process(ch);

        // Check if we closed the opening parenthesis
        if ch == b')'
            && tracker.paren_depth == 0
            && !tracker.in_single_quote
            && !tracker.in_double_quote
        {
            return Some(i);
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
    "atan2", "floor", "acos", "asin", "atan", "ceil", "sqrt", "cos", "sin", "tan", "pow", "log",
    "abs",
];

/// Split arguments on commas while respecting all nested delimiters
///
/// Correctly handles nested parentheses, brackets, and braces, along with
/// quotes and escape sequences. Only splits on commas at the top level
/// (no nesting, no quotes).
///
/// # Examples
/// - `"max(1, 2), 3"` -> `["max(1, 2)", " 3"]`
/// - `"[1,2][0], 3"` -> `["[1,2][0]", " 3"]`
/// - `"{a:1,b:2}, 3"` -> `["{a:1,b:2}", " 3"]`
///
/// This is the standard helper for parsing multi-argument function calls.
pub(super) fn split_args_balanced(args: &str) -> Vec<&str> {
    let mut result = Vec::new();
    let mut start = 0;
    let bytes = args.as_bytes();
    let mut tracker = DelimiterTracker::new();

    for (i, &ch) in bytes.iter().enumerate() {
        tracker.process(ch);

        // Split on comma only at top level (no nesting, no quotes)
        if ch == b',' && tracker.at_top_level() {
            result.push(&args[start..i]);
            start = i + 1;
        }
    }

    // Push the last segment
    if start < args.len() {
        result.push(&args[start..]);
    }

    result
}

/// Escape special characters for Python string literals
///
/// Converts Rust strings to Python-compatible string literals by escaping
/// backslashes, quotes, newlines, and other special characters.
pub(super) fn escape_python_string(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('\'', "\\'")
        .replace('\n', "\\n")
        .replace('\r', "\\r")
        .replace('\t', "\\t")
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

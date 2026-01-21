/// Built-in math constants (name, value) that are pre-initialized
/// Users can override these, but will receive a warning when re-assigning
///
/// Note: `inf` and `nan` are NOT included because pyisheval cannot parse them:
/// - `inf` is not a valid Python literal (Python uses `float('inf')`)
/// - `nan` is not a valid Python literal (Python uses `float('nan')`)
/// - Large exponents like `9e999` fail parsing ("Unexpected trailing input")
/// - pyisheval doesn't expose an API to inject values without parsing
///
/// Instead, `inf` and `nan` are injected directly into the pyisheval context
/// HashMap in `build_pyisheval_context()` to bypass parsing limitations.
///
/// LIMITATION: Lambda expressions that reference properties with `nan` values will
/// fail with "undefined variable" errors because pyisheval cannot create NaN
/// (0.0/0.0 triggers DivisionByZero). Properties with `inf` values work correctly
/// (created using 10**400 arithmetic).
pub const BUILTIN_CONSTANTS: &[(&str, f64)] = &[
    ("pi", core::f64::consts::PI),
    ("e", core::f64::consts::E),
    ("tau", core::f64::consts::TAU),
    ("M_PI", core::f64::consts::PI), // Legacy alias
];

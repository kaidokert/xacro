// Common test helpers for xacro integration tests
//
// This module provides helpers to reduce boilerplate in tests:
// - Processing helpers: run_xacro(), test_xacro(), etc.
// - XML navigation: get_attr(), find_child(), etc.
// - Assertion macros: assert_xacro_contains!, assert_xacro_attr!, etc.
//
// Design decisions:
// - Macros (not functions) for assertions to preserve call-site line numbers
// - Action verb naming (run_xacro, not xacro_ok)
// - Modern Rust idioms (tests/common.rs, not tests/common/mod.rs)

use std::collections::HashMap;
use std::sync::Once;
use xacro::{CompatMode, XacroError, XacroProcessor};
use xmltree::Element;

static INIT: Once = Once::new();

/// Initialize test environment (logging, etc.).
///
/// Call this at the start of tests that need logging output.
/// Safe to call multiple times - initialization happens only once.
#[allow(dead_code)]
pub fn test_init() {
    INIT.call_once(|| {
        let _ = env_logger::builder().is_test(true).try_init();
    });
}

// =============================================================================
// Phase 1: Core Processing Helpers
// =============================================================================

/// Process xacro string, return Result.
///
/// Use this when you need to check for errors.
/// For success cases, use `run_xacro()` instead.
#[allow(dead_code)]
pub fn test_xacro(input: &str) -> Result<String, XacroError> {
    XacroProcessor::new().run_from_string(input)
}

/// Process xacro string and expect success.
///
/// Panics with a clear message if processing fails.
/// Use this for tests that expect successful processing.
#[allow(dead_code)]
pub fn run_xacro(input: &str) -> String {
    test_xacro(input).expect("Xacro processing should succeed")
}

/// Process xacro string and expect success with custom error message.
///
/// Use this when you want to provide specific context about what should succeed.
///
/// # Example
///
/// ```ignore
/// let output = run_xacro_expect(input, "Macro definition should not evaluate parameters");
/// ```
#[allow(dead_code)]
pub fn run_xacro_expect(
    input: &str,
    msg: &str,
) -> String {
    test_xacro(input).expect(msg)
}

/// Process xacro string and parse to XML root element.
///
/// Panics if processing or XML parsing fails.
/// Use this when you need to inspect the XML structure.
#[allow(dead_code)]
pub fn run_xacro_to_xml(input: &str) -> Element {
    let output = run_xacro(input);
    Element::parse(output.as_bytes()).expect("Output should be valid XML")
}

/// Process xacro with custom CLI arguments.
#[allow(dead_code)]
pub fn test_xacro_with_args(
    input: &str,
    args: HashMap<String, String>,
) -> Result<String, XacroError> {
    XacroProcessor::new_with_args(args).run_from_string(input)
}

/// Process xacro with custom CLI arguments, expect success.
#[allow(dead_code)]
pub fn run_xacro_with_args(
    input: &str,
    args: HashMap<String, String>,
) -> String {
    test_xacro_with_args(input, args).expect("Xacro processing with args should succeed")
}

/// Process xacro with compatibility mode.
#[allow(dead_code)]
pub fn test_xacro_with_compat(
    input: &str,
    compat: CompatMode,
) -> Result<String, XacroError> {
    XacroProcessor::new_with_compat_mode(HashMap::new(), compat).run_from_string(input)
}

/// Process xacro with compatibility mode, expect success.
#[allow(dead_code)]
pub fn run_xacro_with_compat(
    input: &str,
    compat: CompatMode,
) -> String {
    test_xacro_with_compat(input, compat).expect("Xacro processing with compat mode should succeed")
}

/// Process xacro file from path, return Result.
///
/// Use this when you need to check for errors.
/// For success cases, use `run_xacro_file()` instead.
#[allow(dead_code)]
pub fn test_xacro_file<P: AsRef<std::path::Path>>(path: P) -> Result<String, XacroError> {
    XacroProcessor::new().run(path)
}

/// Process xacro file from path and expect success.
///
/// Panics with a clear message if processing fails.
/// Use this for tests that expect successful file processing.
#[allow(dead_code)]
pub fn run_xacro_file<P: AsRef<std::path::Path>>(path: P) -> String {
    let path_ref = path.as_ref();
    test_xacro_file(path_ref).unwrap_or_else(|e| {
        panic!(
            "Xacro file processing should succeed for {}: {}",
            path_ref.display(),
            e
        )
    })
}

// =============================================================================
// Phase 2: XML Parsing Helpers
// =============================================================================

/// Get attribute value by local name.
///
/// Panics if attribute is not found (use `get_attr_opt()` for optional attributes).
#[allow(dead_code)]
pub fn get_attr<'a>(
    elem: &'a Element,
    name: &str,
) -> &'a str {
    elem.attributes
        .iter()
        .find(|(attr_name, _)| attr_name.local_name == name)
        .map(|(_, value)| value.as_str())
        .expect(&format!("Expected '{}' attribute on <{}>", name, elem.name))
}

/// Get attribute value by local name, returning None if not found.
#[allow(dead_code)]
pub fn get_attr_opt<'a>(
    elem: &'a Element,
    name: &str,
) -> Option<&'a str> {
    elem.attributes
        .iter()
        .find(|(attr_name, _)| attr_name.local_name == name)
        .map(|(_, value)| value.as_str())
}

/// Find child element by name.
///
/// Panics if child is not found (use `find_child_opt()` for optional children).
#[allow(dead_code)]
pub fn find_child<'a>(
    parent: &'a Element,
    name: &str,
) -> &'a Element {
    parent.get_child(name).expect(&format!(
        "Expected <{}> child element in <{}>",
        name, parent.name
    ))
}

/// Find child element by name, returning None if not found.
#[allow(dead_code)]
pub fn find_child_opt<'a>(
    parent: &'a Element,
    name: &str,
) -> Option<&'a Element> {
    parent.get_child(name)
}

/// Find child element by prefix and name.
///
/// Example: `find_child_prefixed(root, "xacro", "property")` finds `<xacro:property>`
///
/// Panics if child is not found.
#[allow(dead_code)]
pub fn find_child_prefixed<'a>(
    parent: &'a Element,
    prefix: &str,
    name: &str,
) -> &'a Element {
    parent
        .children
        .iter()
        .find_map(|node| {
            node.as_element()
                .filter(|elem| elem.name == name && elem.prefix.as_deref() == Some(prefix))
        })
        .expect(&format!(
            "Expected <{}:{}> element in <{}>",
            prefix, name, parent.name
        ))
}

/// Parse XML string to Element.
///
/// Panics if XML is malformed.
#[allow(dead_code)]
pub fn parse_xml(xml: &str) -> Element {
    Element::parse(xml.as_bytes()).expect("Should parse valid XML")
}

// =============================================================================
// Phase 3: Assertion Macros (preserve call-site line numbers)
// =============================================================================

/// Assert that output contains expected string.
///
/// Provides clear failure message showing actual output.
/// Preserves call-site line number for debugging.
///
/// # Usage
///
/// ```ignore
/// assert_xacro_contains!(output, "link");
/// assert_xacro_contains!(output, "link", "Should contain link when macro is enabled");
/// ```
#[macro_export]
macro_rules! assert_xacro_contains {
    ($output:expr, $expected:expr) => {
        if !$output.contains($expected) {
            panic!(
                "\nAssertion failed: output does not contain expected string\n\
                 Expected substring: \"{}\"\n\
                 Actual output:\n{}\n",
                $expected, $output
            );
        }
    };
    ($output:expr, $expected:expr, $($arg:tt)+) => {
        if !$output.contains($expected) {
            panic!(
                "\nAssertion failed: {}\n\
                 Expected substring: \"{}\"\n\
                 Actual output:\n{}\n",
                format_args!($($arg)+), $expected, $output
            );
        }
    };
}

/// Assert that output does NOT contain unexpected string.
///
/// # Usage
///
/// ```ignore
/// assert_xacro_not_contains!(output, "xacro:property");
/// assert_xacro_not_contains!(output, "${width}", "Property should be expanded");
/// ```
#[macro_export]
macro_rules! assert_xacro_not_contains {
    ($output:expr, $unexpected:expr) => {
        if $output.contains($unexpected) {
            panic!(
                "\nAssertion failed: output contains unexpected string\n\
                 Unexpected substring: \"{}\"\n\
                 Actual output:\n{}\n",
                $unexpected, $output
            );
        }
    };
    ($output:expr, $unexpected:expr, $($arg:tt)+) => {
        if $output.contains($unexpected) {
            panic!(
                "\nAssertion failed: {}\n\
                 Unexpected substring: \"{}\"\n\
                 Actual output:\n{}\n",
                format_args!($($arg)+), $unexpected, $output
            );
        }
    };
}

/// Assert that attribute has expected string value.
///
/// Provides better error message than raw assert_eq!, including element context.
#[macro_export]
macro_rules! assert_xacro_attr {
    ($elem:expr, $name:expr, $expected:expr) => {
        match $crate::common::get_attr_opt($elem, $name) {
            Some(actual) => {
                if actual != $expected {
                    panic!(
                        "\nAssertion failed: attribute mismatch\n\
                         Element: <{}>\n\
                         Attribute: {}\n\
                         Expected: \"{}\"\n\
                         Actual: \"{}\"\n",
                        $elem.name, $name, $expected, actual
                    );
                }
            }
            None => {
                panic!(
                    "\nAssertion failed: attribute missing\n\
                     Element: <{}>\n\
                     Attribute: {}\n",
                    $elem.name, $name
                );
            }
        }
    };
}

/// Assert that attribute has expected float value (with tolerance).
///
/// Useful for numeric comparisons that may have floating-point precision differences.
#[macro_export]
macro_rules! assert_attr_float {
    ($elem:expr, $attr:expr, $expected:expr, $tolerance:expr) => {{
        let value_str = $crate::common::get_attr($elem, $attr);
        let value: f64 = value_str.parse().unwrap_or_else(|_| {
            panic!(
                "Attribute '{}' on <{}> is not a valid float: '{}'",
                $attr,
                ($elem).name,
                value_str
            )
        });
        assert!(
            (value - $expected).abs() < $tolerance,
            "Attribute '{}' on <{}>: expected {}, got {} (tolerance: {})",
            $attr,
            ($elem).name,
            $expected,
            value,
            $tolerance
        );
    }};
}

// =============================================================================
// Phase 4: Error Checking Macros (preserve call-site line numbers)
// =============================================================================

/// Assert that processing fails with specific error message.
#[macro_export]
macro_rules! assert_xacro_error {
    ($input:expr, $expected_msg:expr) => {{
        let result = $crate::common::test_xacro($input);
        match result {
            Err(e) => {
                let err_msg = e.to_string();
                if !err_msg.contains($expected_msg) {
                    panic!(
                        "\nAssertion failed: error message mismatch\n\
                         Expected substring: \"{}\"\n\
                         Actual error:\n{}\n",
                        $expected_msg, err_msg
                    );
                }
            }
            Ok(_) => {
                panic!(
                    "\nAssertion failed: expected error but processing succeeded\n\
                     Expected error containing: \"{}\"\n",
                    $expected_msg
                );
            }
        }
    }};
    ($input:expr, $expected_msg:expr, $($arg:tt)+) => {{
        let result = $crate::common::test_xacro($input);
        match result {
            Err(e) => {
                let err_msg = e.to_string();
                if !err_msg.contains($expected_msg) {
                    panic!(
                        "\nAssertion failed: {}\n\
                         Expected substring: \"{}\"\n\
                         Actual error:\n{}\n",
                        format_args!($($arg)+), $expected_msg, err_msg
                    );
                }
            }
            Ok(_) => {
                panic!(
                    "\nAssertion failed: {}\n\
                     Expected error containing: \"{}\"\n",
                    format_args!($($arg)+), $expected_msg
                );
            }
        }
    }};
}

/// Assert that processing fails with specific error variant.
///
/// Example: `assert_xacro_error_variant!(input, |e| matches!(e, XacroError::MissingNamespace(_)))`
#[macro_export]
macro_rules! assert_xacro_error_variant {
    ($input:expr, $predicate:expr, $($arg:tt)+) => {{
        let result = $crate::common::test_xacro($input);
        match result {
            Err(ref e) => {
                if !$predicate(e) {
                    panic!(
                        "\nAssertion failed: {}\n\
                         Actual error: {:?}\n",
                        format_args!($($arg)+), e
                    );
                }
            }
            Ok(_) => {
                panic!(
                    "\nAssertion failed: {}\n\
                     Expected error but processing succeeded\n",
                    format_args!($($arg)+)
                );
            }
        }
    }};
    ($input:expr, $predicate:expr) => {{
        let result = $crate::common::test_xacro($input);
        match result {
            Err(ref e) => {
                if !$predicate(e) {
                    panic!(
                        "\nAssertion failed: error variant mismatch\n\
                         Actual error: {:?}\n",
                        e
                    );
                }
            }
            Ok(_) => {
                panic!("\nAssertion failed: expected error but processing succeeded\n");
            }
        }
    }};
}

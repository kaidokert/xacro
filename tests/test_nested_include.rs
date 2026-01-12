mod common;
use crate::common::*;
use std::fs;
use std::path::PathBuf;
use xacro::XacroProcessor;

/// Test cross-namespace macro expansion
///
/// Reproduces the bug where macro calls inside macro bodies weren't expanded
/// when files used different xacro namespace URI variants.
///
/// Test structure:
/// - Main file: xmlns:xacro="http://www.ros.org/wiki/xacro" (with www.)
/// - Level1 file: xmlns:xacro="http://ros.org/wiki/xacro" (without www.)
///   - Defines outer_macro which calls my_macro
/// - Level2 file: xmlns:xacro="http://ros.org/wiki/xacro" (without www.)
///   - Defines my_macro
///
/// Without the fix, when outer_macro is expanded, the nested <xacro:my_macro>
/// call would not be recognized because is_macro_call() only checked for exact
/// namespace URI matches.
#[test]
fn test_nested_include_cross_namespace() {
    // Use absolute paths to avoid mutating global process state
    let test_data_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let input_path = test_data_dir.join("nested_include_main.xacro");
    let expected_path = test_data_dir.join("nested_include_main_expected.urdf");

    let expected = fs::read_to_string(&expected_path)
        .expect("Should read expected URDF fixture from tests/data");

    let processor = XacroProcessor::new();
    let output = processor
        .run(&input_path)
        .expect("Xacro processor should handle nested includes across namespace variants");

    // Check for unexpanded macro calls (the bug symptom)
    assert_xacro_not_contains!(
        output,
        "<xacro:",
        "Output contains unexpanded xacro tags - cross-namespace macro expansion failed"
    );

    // Verify output matches expected
    assert_eq!(
        normalize(&output),
        normalize(&expected),
        "Output should match expected URDF"
    );
}

/// Helper to normalize XML for comparison (remove extra whitespace)
fn normalize(xml: &str) -> String {
    xml.lines()
        .map(|line| line.trim())
        .filter(|line| !line.is_empty())
        .collect::<Vec<_>>()
        .join("")
}

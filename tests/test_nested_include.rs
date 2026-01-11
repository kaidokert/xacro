use std::env;
use std::fs;
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
    // Change to tests/data directory so relative includes work
    let original_dir = env::current_dir().unwrap();
    let test_data_dir = original_dir.join("tests/data");
    env::set_current_dir(&test_data_dir).expect("Failed to change to test data directory");

    let input = fs::read_to_string("nested_include_main.xacro").unwrap();
    let expected = fs::read_to_string("nested_include_main_expected.urdf").unwrap();

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(&input).unwrap();

    // Restore original directory
    env::set_current_dir(&original_dir).unwrap();

    // Check for unexpanded macro calls (the bug symptom)
    assert!(
        !result.contains("<xacro:my_macro"),
        "Output contains unexpanded <xacro:my_macro> tag - cross-namespace macro expansion failed"
    );
    assert!(
        !result.contains("<xacro:outer_macro"),
        "Output contains unexpanded <xacro:outer_macro> tag"
    );

    // Verify output matches expected
    let normalize = |s: &str| {
        s.lines()
            .map(|l| l.trim())
            .filter(|l| !l.is_empty())
            .collect::<Vec<_>>()
            .join("\n")
    };

    assert_eq!(
        normalize(&result),
        normalize(&expected),
        "Output should match expected URDF"
    );
}

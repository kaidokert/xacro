// Integration tests for missing include file handling
//
// Python xacro has 3 different behaviors for missing includes:
// 1. Glob patterns (*, [, ?) with no matches → warn, continue
// 2. optional="true" attribute → silent skip, continue
// 3. Regular missing file → error
//
// We must match this behavior exactly.

use xacro::XacroProcessor;

// ============================================================================
// Case 1: Glob Pattern with No Matches (warn, continue)
// ============================================================================

#[test]
fn test_include_glob_pattern_no_matches_asterisk() {
    // Glob pattern with * wildcard and no matches
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/nonexistent_*.xacro" />
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should succeed (warn but continue)
    assert!(
        result.is_ok(),
        "Glob pattern with no matches should warn but continue, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<link name="base""#),
        "Should process rest of file after glob warning"
    );
}

#[test]
fn test_include_glob_pattern_no_matches_bracket() {
    // Glob pattern with [ character (like template placeholders)
    // This is the case from testdata/bugs/eb954ca7/
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="[URDF_LOCATION]" />
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should succeed (warn but continue)
    assert!(
        result.is_ok(),
        "Template placeholder [URDF_LOCATION] should be treated as glob pattern and warn but continue, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<link name="base""#),
        "Should process rest of file after glob warning"
    );
}

#[test]
fn test_include_glob_pattern_no_matches_question() {
    // Glob pattern with ? wildcard
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/file?.xacro" />
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should succeed (warn but continue)
    assert!(
        result.is_ok(),
        "Glob pattern with ? should warn but continue, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<link name="base""#),
        "Should process rest of file after glob warning"
    );
}

// ============================================================================
// Case 2: Optional Include (silent skip, continue)
// ============================================================================

#[test]
fn test_include_optional_missing_file() {
    // Missing file with optional="true" attribute
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/nonexistent.xacro" optional="true" />
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should succeed (silent skip, no error, no warning)
    assert!(
        result.is_ok(),
        "Optional include should silently skip, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<link name="base""#),
        "Should process rest of file after optional include skip"
    );
}

#[test]
fn test_include_optional_false_missing_file() {
    // Missing file with optional="false" (explicit)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/nonexistent.xacro" optional="false" />
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should error (optional="false" means NOT optional)
    assert!(
        result.is_err(),
        "optional=\"false\" with missing file should error"
    );

    let error = result.unwrap_err().to_string();
    assert!(
        error.to_lowercase().contains("nonexistent.xacro"),
        "Error should mention the missing filename"
    );
}

// ============================================================================
// Case 3: Regular Missing File (error)
// ============================================================================

#[test]
fn test_include_regular_missing_file() {
    // Regular missing file without glob pattern or optional attribute
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/nonexistent.xacro" />
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should error
    assert!(result.is_err(), "Regular missing file should error");

    let error = result.unwrap_err().to_string();
    assert!(
        error.to_lowercase().contains("nonexistent.xacro"),
        "Error should mention the missing filename"
    );
}

// ============================================================================
// Edge Cases
// ============================================================================

#[test]
fn test_include_optional_glob_both_attributes() {
    // Both optional="true" AND glob pattern
    // Python: optional takes precedence (silent skip, no warning)
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/no_match_*.xacro" optional="true" />
  <link name="base"/>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Should succeed (silent skip due to optional="true")
    assert!(
        result.is_ok(),
        "Optional glob with no matches should silently skip, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<link name="base""#),
        "Should process rest of file"
    );
}

#[test]
fn test_include_glob_with_actual_matches() {
    use std::io::Write;
    use tempfile::NamedTempFile;

    // Create a temporary test file with automatic cleanup
    let mut temp_file = NamedTempFile::new().unwrap();
    writeln!(
        temp_file,
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="included_link"/>
</robot>"#
    )
    .unwrap();

    // Construct glob pattern that matches our temp file
    let temp_path = temp_file.path();
    let temp_dir = temp_path.parent().unwrap();
    let file_name = temp_path.file_name().unwrap().to_str().unwrap();
    let prefix = file_name.get(..8).unwrap_or(file_name);
    let glob_pattern = temp_dir.join(format!("{}*", prefix));

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="{}" />
  <link name="base"/>
</robot>"#,
        glob_pattern.display()
    );

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(&input);

    // Should succeed and include the matched file
    assert!(
        result.is_ok(),
        "Glob with matches should succeed, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains(r#"<link name="included_link""#),
        "Should include content from matched file"
    );
    assert!(
        output.contains(r#"<link name="base""#),
        "Should also include base content"
    );
    // temp_file automatically cleaned up when dropped
}

// ============================================================================
// Compatibility verification with Python xacro
// ============================================================================

#[test]
fn test_python_xacro_compatibility_template_placeholder() {
    // This is the exact case from testdata/bugs/eb954ca7/
    // Python xacro succeeds with warning, we must too
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="[ROBOT_NAME]">
[MODIFIED_XACRO_ARGS]
    <!-- Import [ROBOT_NAME] urdf file -->
    <xacro:include filename="[URDF_LOCATION]" />

[MODIFIED_XACRO_IMPORTS]
[MODIFIED_XACRO_COMMANDS]
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    // Must succeed to match Python xacro behavior
    assert!(
        result.is_ok(),
        "Template with [URDF_LOCATION] must succeed like Python xacro, got error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    assert!(
        output.contains("[ROBOT_NAME]"),
        "Should preserve template placeholders in output"
    );
}

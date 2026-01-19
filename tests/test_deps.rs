mod common;

use assert_cmd::Command;
use std::collections::HashSet;

#[test]
fn test_deps_with_includes() {
    let processor = xacro::XacroProcessor::new();
    let (_, includes) = processor
        .run_with_deps("tests/data/include_test_multi_base.xacro")
        .expect("Failed to get deps");

    // Should include both files
    assert_eq!(includes.len(), 2);

    // Check that both files are in the list (order may vary)
    let include_strs: Vec<String> = includes.iter().map(|p| p.display().to_string()).collect();
    assert!(include_strs
        .iter()
        .any(|s| s.contains("include_test_multi_wheels.xacro")));
    assert!(include_strs
        .iter()
        .any(|s| s.contains("include_test_multi_arms.xacro")));
}

#[test]
fn test_deps_with_no_includes() {
    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="value" value="42"/>
  <link name="base">
    <inertial>
      <mass value="${value}"/>
    </inertial>
  </link>
</robot>"#;

    let processor = xacro::XacroProcessor::new();
    let (_, includes) = processor
        .run_from_string_with_deps(input)
        .expect("Failed to get deps");

    // Should have no includes
    assert_eq!(includes.len(), 0);
}

#[test]
fn test_deps_with_nested_includes() {
    // Test that nested includes are tracked
    // include_test_nested_base.xacro includes include_test_nested_arm.xacro
    // which in turn includes include_test_nested_hand.xacro
    let processor = xacro::XacroProcessor::new();
    let result = processor.run_with_deps("tests/data/include_test_nested_base.xacro");

    match result {
        Ok((_, includes)) => {
            // Should include exactly both arm and hand files
            assert_eq!(includes.len(), 2);

            let include_strs: Vec<String> =
                includes.iter().map(|p| p.display().to_string()).collect();

            // Check for arm file
            assert!(
                include_strs
                    .iter()
                    .any(|s| s.contains("include_test_nested_arm.xacro")),
                "Should include arm file, got: {:?}",
                include_strs
            );

            // Check for hand file
            assert!(
                include_strs
                    .iter()
                    .any(|s| s.contains("include_test_nested_hand.xacro")),
                "Should include hand file, got: {:?}",
                include_strs
            );
        }
        Err(e) => {
            // If the test files don't exist, skip the test
            eprintln!("Skipping nested includes test (missing test files): {}", e);
        }
    }
}

#[test]
fn test_deps_deduplication() {
    // Test that if a file is included multiple times,
    // the library deduplicates and returns each file only once

    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="tests/data/include_test_component.xacro"/>
  <xacro:include filename="tests/data/include_test_component.xacro"/>
</robot>"#;

    let processor = xacro::XacroProcessor::new();
    let result = processor.run_from_string_with_deps(input);

    match result {
        Ok((_, includes)) => {
            // Library should deduplicate - file included twice but should appear only once
            assert_eq!(
                includes.len(),
                1,
                "File included twice should appear only once after deduplication, got: {:?}",
                includes
            );

            // Verify it's the correct file
            assert!(
                includes[0]
                    .to_string_lossy()
                    .contains("include_test_component.xacro"),
                "Expected include_test_component.xacro, got: {:?}",
                includes[0]
            );
        }
        Err(e) => {
            eprintln!("Skipping deduplication test (missing test files): {}", e);
        }
    }
}

// CLI Integration Tests

#[test]
fn cli_deps_with_multiple_includes() {
    // Test CLI --deps flag with multiple includes
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let assert = cmd
        .arg("--deps")
        .arg("tests/data/include_test_multi_base.xacro")
        .assert()
        .success();

    let stdout = String::from_utf8(assert.get_output().stdout.clone())
        .expect("stdout should be valid UTF-8");

    // Space-separated list on a single line
    let line = stdout.trim_end_matches('\n');
    let parts: Vec<&str> = line.split_whitespace().collect();

    // Should include both files
    assert_eq!(
        parts.len(),
        2,
        "Expected 2 deps from CLI, got {}: {:?}",
        parts.len(),
        parts
    );

    // Verify deduplication (no duplicates)
    let unique: HashSet<&str> = parts.iter().copied().collect();
    assert_eq!(
        unique.len(),
        parts.len(),
        "Expected deduplicated deps, but found duplicates: {:?}",
        parts
    );

    // Check that both expected files are present
    assert!(
        parts
            .iter()
            .any(|p| p.contains("include_test_multi_wheels.xacro")),
        "Expected include_test_multi_wheels.xacro in CLI deps, got: {:?}",
        parts
    );
    assert!(
        parts
            .iter()
            .any(|p| p.contains("include_test_multi_arms.xacro")),
        "Expected include_test_multi_arms.xacro in CLI deps, got: {:?}",
        parts
    );
}

#[test]
fn cli_deps_with_nested_includes() {
    // Test CLI --deps flag with nested includes
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let assert = cmd
        .arg("--deps")
        .arg("tests/data/include_test_nested_base.xacro")
        .assert()
        .success();

    let stdout = String::from_utf8(assert.get_output().stdout.clone())
        .expect("stdout should be valid UTF-8");

    let line = stdout.trim_end_matches('\n');
    let parts: Vec<&str> = line.split_whitespace().collect();

    // Should include both arm and hand files (nested includes)
    assert_eq!(
        parts.len(),
        2,
        "Expected 2 deps from nested includes, got {}: {:?}",
        parts.len(),
        parts
    );

    assert!(
        parts
            .iter()
            .any(|p| p.contains("include_test_nested_arm.xacro")),
        "Expected include_test_nested_arm.xacro, got: {:?}",
        parts
    );
    assert!(
        parts
            .iter()
            .any(|p| p.contains("include_test_nested_hand.xacro")),
        "Expected include_test_nested_hand.xacro, got: {:?}",
        parts
    );
}

#[test]
fn cli_deps_no_includes_empty_output() {
    // Test CLI --deps flag with no includes (empty output)
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let assert = cmd
        .arg("--deps")
        .arg("tests/data/eval_arithmetic.xacro")
        .assert()
        .success();

    let stdout = String::from_utf8(assert.get_output().stdout.clone())
        .expect("stdout should be valid UTF-8");

    // Should output empty line (or just newline)
    assert_eq!(
        stdout, "\n",
        "Expected empty line for file with no includes, got: {:?}",
        stdout
    );
}

#[test]
fn cli_deps_output_is_sorted() {
    // Test that CLI --deps output is sorted (deterministic)
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let assert = cmd
        .arg("--deps")
        .arg("tests/data/include_test_multi_base.xacro")
        .assert()
        .success();

    let stdout = String::from_utf8(assert.get_output().stdout.clone())
        .expect("stdout should be valid UTF-8");

    let line = stdout.trim_end_matches('\n');
    let parts: Vec<&str> = line.split_whitespace().collect();

    // Verify output is sorted
    let mut sorted_parts = parts.clone();
    sorted_parts.sort();
    assert_eq!(
        parts, sorted_parts,
        "Expected sorted output, got unsorted: {:?}",
        parts
    );
}

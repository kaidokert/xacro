use assert_cmd::Command;
use predicates::prelude::*;
use std::collections::HashSet;

#[test]
fn test_deps_with_includes() {
    let processor = xacro::XacroProcessor::new();
    let (_, includes) = processor
        .run_with_deps("tests/data/include_test_multi_base.xacro")
        .expect("Failed to get deps");

    // Should include both files in sorted order (deterministic)
    assert_eq!(includes.len(), 2);

    // Assert exact sorted order (library guarantees sorted output)
    let include_strs: Vec<String> = includes.iter().map(|p| p.display().to_string()).collect();
    assert!(
        include_strs[0].contains("include_test_multi_arms.xacro"),
        "First include should be arms file, got: {:?}",
        include_strs
    );
    assert!(
        include_strs[1].contains("include_test_multi_wheels.xacro"),
        "Second include should be wheels file, got: {:?}",
        include_strs
    );
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
    let (_, includes) = processor
        .run_with_deps("tests/data/include_test_nested_base.xacro")
        .expect("Failed to get deps for nested includes test");

    // Should include exactly both arm and hand files in sorted order (deterministic)
    assert_eq!(includes.len(), 2);

    // Assert exact sorted order (library guarantees sorted output)
    let include_strs: Vec<String> = includes.iter().map(|p| p.display().to_string()).collect();
    assert!(
        include_strs[0].contains("include_test_nested_arm.xacro"),
        "First include should be arm file, got: {:?}",
        include_strs
    );
    assert!(
        include_strs[1].contains("include_test_nested_hand.xacro"),
        "Second include should be hand file, got: {:?}",
        include_strs
    );
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
    let (_, includes) = processor
        .run_from_string_with_deps(input)
        .expect("Failed to get deps for deduplication test");

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

    // Should output empty string (no trailing newline, matching Python xacro)
    assert_eq!(
        stdout, "",
        "Expected empty output for file with no includes, got: {:?}",
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

// Stdin Integration Tests

#[test]
fn cli_stdin_with_dash() {
    // Test reading from stdin using '-' argument
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="value" value="42"/>
  <link name="base_${value}"/>
</robot>"#;

    let assert = cmd.arg("-").write_stdin(input).assert().success();

    let stdout = String::from_utf8(assert.get_output().stdout.clone())
        .expect("stdout should be valid UTF-8");

    assert!(
        stdout.contains(r#"<link name="base_42"/>"#),
        "Expected property substitution in output, got: {}",
        stdout
    );
}

#[test]
fn cli_stdin_no_argument() {
    // Test reading from stdin when no argument is provided
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="width" value="2.5"/>
  <box size="${width}"/>
</robot>"#;

    let assert = cmd.write_stdin(input).assert().success();

    let stdout = String::from_utf8(assert.get_output().stdout.clone())
        .expect("stdout should be valid UTF-8");

    assert!(
        stdout.contains(r#"<box size="2.5"/>"#),
        "Expected property substitution in output, got: {}",
        stdout
    );
}

#[test]
fn cli_stdin_with_deps_flag_errors() {
    // Test that --deps with stdin produces an error
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let input = r#"<?xml version="1.0"?><robot name="test"/>"#;

    cmd.arg("--deps")
        .arg("-")
        .write_stdin(input)
        .assert()
        .failure()
        .stderr(predicates::str::contains(
            "--deps flag is not supported when reading from stdin",
        ));
}

#[test]
fn cli_stdin_with_deps_no_arg_errors() {
    // Test that --deps with no argument (stdin) produces an error
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let input = r#"<?xml version="1.0"?><robot name="test"/>"#;

    cmd.arg("--deps")
        .write_stdin(input)
        .assert()
        .failure()
        .stderr(predicates::str::contains(
            "--deps flag is not supported when reading from stdin",
        ));
}

// xacro.print_location() Tests

#[test]
fn cli_print_location_outputs_to_stderr() {
    // Test that xacro.print_location() builtin function outputs location info to stderr
    let mut cmd = Command::cargo_bin("xacro").expect("xacro binary not found");

    let assert = cmd
        .arg("tests/data/print_location_test.xacro")
        .assert()
        .success();

    let stderr = String::from_utf8(assert.get_output().stderr.clone())
        .expect("stderr should be valid UTF-8");

    // Verify stderr contains location output (at minimum "when processing file:")
    assert!(
        stderr.contains("when processing file:"),
        "Expected 'when processing file:' in stderr from print_location(), got: {:?}",
        stderr
    );

    // Verify stderr is not empty (print_location() should print something)
    assert!(
        !stderr.trim().is_empty(),
        "Expected non-empty stderr from print_location(), got empty"
    );
}

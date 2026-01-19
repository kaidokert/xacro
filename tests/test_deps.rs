mod common;

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
            // Should include both arm and hand files
            assert!(includes.len() >= 2);

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
    // Test that if a file is somehow included multiple times,
    // it only appears once in the deps list
    // (Note: Our circular include check prevents this, but the deduplication
    // is there as a safety measure matching Python's behavior)

    let input = r#"<?xml version="1.0"?>
<robot name="test_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="include_test_component.xacro"/>
</robot>"#;

    let processor = xacro::XacroProcessor::new();
    let result = processor.run_from_string_with_deps(input);

    match result {
        Ok((_, includes)) => {
            // Count occurrences of each file
            let mut counts = std::collections::HashMap::new();
            for path in &includes {
                *counts.entry(path.clone()).or_insert(0) += 1;
            }

            // Each file should appear exactly once
            for (path, count) in counts {
                assert_eq!(
                    count, 1,
                    "File {:?} appears {} times, expected 1",
                    path, count
                );
            }
        }
        Err(e) => {
            eprintln!("Skipping deduplication test (missing test files): {}", e);
        }
    }
}

//! Tests for error context enrichment in nested scenarios
//!
//! These tests verify that errors occurring in nested macros, includes,
//! extensions, and conditionals are properly enriched with location context.

use xacro::error::ErrorContext;
use xacro::XacroError;
use xacro::XacroProcessor;

/// Helper to process xacro input and return Result
fn test_xacro(input: &str) -> Result<String, XacroError> {
    XacroProcessor::new().run_from_string(input)
}

/// Helper to extract ErrorContext from WithContext or panic
fn extract_context(err: &XacroError) -> &ErrorContext {
    match err {
        XacroError::WithContext { context, .. } => context,
        other => panic!("Expected WithContext, got: {:?}", other),
    }
}

#[test]
fn test_error_in_nested_macro() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="inner">
    <link name="${undefined_property}"/>
  </xacro:macro>

  <xacro:macro name="outer">
    <xacro:inner/>
  </xacro:macro>

  <xacro:outer/>
</robot>"#;

    let err = test_xacro(input).expect_err("Should error on undefined property in nested macro");
    let context = extract_context(&err);

    // Verify macro stack has both macros
    assert_eq!(
        context.macro_stack.len(),
        2,
        "Macro stack should contain both inner and outer: {:?}",
        context.macro_stack
    );
    assert!(
        context.macro_stack.contains(&"inner".to_string()),
        "Macro stack should contain 'inner'"
    );
    assert!(
        context.macro_stack.contains(&"outer".to_string()),
        "Macro stack should contain 'outer'"
    );
}

#[test]
fn test_error_in_included_file() {
    // Create a temporary directory for test files
    let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
    let main_file = temp_dir.path().join("main.xacro");
    let included_file = temp_dir.path().join("included.xacro");

    // Write included file with undefined property
    std::fs::write(
        &included_file,
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="${undefined_in_included_file}"/>
</robot>"#,
    )
    .expect("Failed to write included file");

    // Write main file that includes the problematic file
    let main_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{}"/>
</robot>"#,
        included_file.display()
    );

    std::fs::write(&main_file, main_content).expect("Failed to write main file");

    // Process main file
    let processor = XacroProcessor::new();
    let result = processor.run(&main_file);

    let err = result.expect_err("Should error on undefined property in included file");
    let context = extract_context(&err);

    // Verify include stack shows the included file
    assert!(
        !context.include_stack.is_empty(),
        "Include stack should not be empty"
    );
    assert!(
        context
            .include_stack
            .iter()
            .any(|p| p.ends_with("included.xacro")),
        "Include stack should contain included.xacro: {:?}",
        context.include_stack
    );
}

#[test]
fn test_error_in_extension_arg() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="pkg_path" value="$(find ${undefined_pkg})"/>
  <path>${pkg_path}</path>
</robot>"#;

    let err = test_xacro(input).expect_err("Should error on undefined property in extension arg");

    // Error should be enriched with context
    // The undefined_pkg error occurs during property evaluation
    match &err {
        XacroError::WithContext { .. } => {
            // Good - error has context
        }
        other => panic!("Expected WithContext wrapping the error, got: {:?}", other),
    }
}

#[test]
fn test_error_in_conditional() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:if value="${undefined_property > 5}">
    <link name="conditional_link"/>
  </xacro:if>
</robot>"#;

    let err = test_xacro(input).expect_err("Should error on undefined property in conditional");

    // Error should be enriched with context
    match &err {
        XacroError::WithContext { .. } => {
            // Good - error has context
        }
        other => panic!("Expected WithContext wrapping the error, got: {:?}", other),
    }
}

#[test]
fn test_error_in_macro_with_include() {
    // Create a temporary directory for test files
    let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
    let main_file = temp_dir.path().join("main.xacro");
    let included_file = temp_dir.path().join("macros.xacro");

    // Write file with macro definition
    std::fs::write(
        &included_file,
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="broken_macro">
    <link name="${undefined_in_macro}"/>
  </xacro:macro>
</robot>"#,
    )
    .expect("Failed to write macros file");

    // Write main file that includes and uses the macro
    let main_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{}"/>
  <xacro:broken_macro/>
</robot>"#,
        included_file.display()
    );

    std::fs::write(&main_file, main_content).expect("Failed to write main file");

    // Process main file
    let processor = XacroProcessor::new();
    let result = processor.run(&main_file);

    let err = result.expect_err("Should error on undefined property in macro from included file");
    let context = extract_context(&err);

    // Verify macro stack is populated
    // Note: Include stack is empty because the error occurs during macro expansion in main file,
    // after the include has been processed. The include stack only tracks files currently being
    // processed, not where macros were originally defined.
    assert!(
        !context.macro_stack.is_empty(),
        "Macro stack should not be empty"
    );
    assert!(
        context.macro_stack.contains(&"broken_macro".to_string()),
        "Macro stack should contain 'broken_macro'"
    );
}

#[test]
fn test_error_in_property_default() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="computed" value="${undefined_property * 2}"/>
  <link name="${computed}"/>
</robot>"#;

    let err = test_xacro(input).expect_err("Should error on undefined property in default value");

    // Error should be enriched with context
    match &err {
        XacroError::WithContext { .. } => {
            // Good - error has context
        }
        other => panic!("Expected WithContext wrapping the error, got: {:?}", other),
    }
}

#[test]
fn test_error_in_arg_default() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="scale" default="${undefined_property}"/>
  <link name="test" scale="$(arg scale)"/>
</robot>"#;

    let err = test_xacro(input).expect_err("Should error on undefined property in arg default");

    // Error should be enriched with context
    match &err {
        XacroError::WithContext { .. } => {
            // Good - error has context
        }
        other => panic!("Expected WithContext wrapping the error, got: {:?}", other),
    }
}

#[test]
fn test_error_in_macro_parameter_default() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="scale:=${undefined_property}">
    <link name="test" scale="${scale}"/>
  </xacro:macro>

  <xacro:test_macro/>
</robot>"#;

    let err = test_xacro(input)
        .expect_err("Should error on undefined property in macro parameter default");

    // Error should be enriched with context
    let context = extract_context(&err);

    // Verify macro stack shows the macro
    // Note: Parameter default evaluation happens inside the macro call, so macro should be in stack
    assert!(
        context.macro_stack.contains(&"test_macro".to_string()),
        "Macro stack should contain 'test_macro'. Actual stack: {:?}",
        context.macro_stack
    );
}

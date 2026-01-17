mod common;
use crate::common::*;
use std::collections::HashMap;

#[test]
fn test_nested_extension_in_args() {
    // Test nested extension resolution: $(arg $(arg inner))
    // 1. $(arg inner) -> "outer"
    // 2. $(arg outer) -> "success"

    let input = r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:property name="result" value="$(arg $(arg inner))" />
            <test result="${result}"/>
        </robot>"#;

    let mut args = HashMap::new();
    args.insert("inner".to_string(), "outer".to_string());
    args.insert("outer".to_string(), "success".to_string());

    let output = run_xacro_with_args(input, args);

    let root = parse_xml(&output);
    let test_elem = find_child(&root, "test");
    assert_xacro_attr!(
        test_elem,
        "result",
        "success",
        "Failed to resolve nested extension"
    );
}

use xacro::XacroProcessor;

#[test]
fn test_nested_extension_in_args() {
    let processor = XacroProcessor::builder()
        .with_arg("inner", "outer")
        .with_arg("outer", "success")
        .build();

    // We want to evaluate $(arg $(arg inner))
    // 1. $(arg inner) -> "outer"
    // 2. $(arg outer) -> "success"

    let input = r#"<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:property name="result" value="$(arg $(arg inner))" />
            <test result="${result}"/>
        </robot>"#;

    let result = processor.run_from_string(input);

    if let Ok(xml) = &result {
        println!("Generated XML: {}", xml);
    } else {
        println!("Error: {:?}", result.as_ref().err());
    }

    let xml = result.expect("Processing failed");

    assert!(
        xml.contains("result=\"success\""),
        "Failed to resolve nested extension. XML: {}",
        xml
    );
}

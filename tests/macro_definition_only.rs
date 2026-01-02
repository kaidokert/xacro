use xacro::XacroProcessor;

/// Test that macro definitions (without calls) don't trigger undefined variable errors
///
/// This is a CRITICAL test - if a file only defines a macro but never calls it,
/// the macro parameters should never be evaluated during definition.
#[test]
fn test_macro_definition_without_call() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test_macro" params="name">
    <link name="${name}"/>
  </xacro:macro>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Macro definition should not evaluate parameters. Error: {:?}",
        result.err()
    );

    let output = result.unwrap();
    // Output should be empty robot (macro definition removed, no expansion)
    assert!(!output.contains("link"), "Should not contain any links");
    assert!(output.contains("<robot"), "Should have robot element");
}

/// Test the actual failing case from PR2 corpus
#[test]
fn test_pr2_head_gazebo_macro_only() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
       xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
       xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
       xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="pr2_head_gazebo_v0" params="name">
    <gazebo reference="${name}_plate_frame">
      <material value="Gazebo/Grey" />
    </gazebo>
  </xacro:macro>

</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "PR2 head gazebo macro (definition only) should process without error. Error: {:?}",
        result.err()
    );
}

/// Test macro with parameter used in attribute value
#[test]
fn test_macro_definition_param_in_attribute() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="prefix">
    <link name="${prefix}_link"/>
  </xacro:macro>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Macro param in attribute should not be evaluated during definition. Error: {:?}",
        result.err()
    );
}

/// Test macro with parameter used in text node
#[test]
fn test_macro_definition_param_in_text() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="value">
    <item>${value}</item>
  </xacro:macro>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Macro param in text node should not be evaluated during definition. Error: {:?}",
        result.err()
    );
}

/// Test macro with complex expression using parameter
#[test]
fn test_macro_definition_param_in_expression() {
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test" params="size">
    <box size="${size * 2} ${size} ${size / 2}"/>
  </xacro:macro>
</robot>"#;

    let processor = XacroProcessor::new();
    let result = processor.run_from_string(input);

    assert!(
        result.is_ok(),
        "Macro param in expression should not be evaluated during definition. Error: {:?}",
        result.err()
    );
}
#[test]
fn debug_xml_parsing() {
    use xmltree::Element;

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="test" params="name">
    <gazebo reference="${name}_plate"/>
  </xacro:macro>
</robot>"#;

    let elem = Element::parse(input.as_bytes()).unwrap();

    eprintln!("Root: name='{}', ns={:?}", elem.name, elem.namespace);

    for child in &elem.children {
        if let xmltree::XMLNode::Element(child_elem) = child {
            eprintln!(
                "  Child: name='{}', ns={:?}",
                child_elem.name, child_elem.namespace
            );
        }
    }
}

#[test]
fn explore_namespace_declarations() {
    use xmltree::Element;

    let xml = r#"<?xml version="1.0"?>
<robot xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
       xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#xacro">
  <xacro:macro name="test" params="foo">
    <link name="${foo}"/>
  </xacro:macro>
</robot>"#;

    let root = Element::parse(xml.as_bytes()).unwrap();

    eprintln!("Root element: {}", root.name);
    eprintln!("Root namespaces: {:?}", root.namespaces);

    // Check first child (the macro)
    if let Some(xmltree::XMLNode::Element(child)) = root.children.first() {
        eprintln!("\nChild element: {}", child.name);
        eprintln!("Child prefix: {:?}", child.prefix);
        eprintln!("Child namespace: {:?}", child.namespace);
        eprintln!("Child namespaces: {:?}", child.namespaces);
    }
}
#[test]
fn test_namespace_api() {
    use xmltree::Element;

    let xml = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://playerstage.sourceforge.net/gazebo/xmlschema/#xacro">
  <xacro:macro name="test"/>
</robot>"#;

    let root = Element::parse(xml.as_bytes()).unwrap();

    if let Some(ns_map) = &root.namespaces {
        // Try to get the "xacro" namespace URI
        eprintln!("Trying to get xacro namespace...");
        eprintln!("ns_map type: {}", std::any::type_name_of_val(ns_map));

        // Namespace might be a HashMap or BTreeMap
        // Try get() method
        if let Some(xacro_uri) = ns_map.get("xacro") {
            eprintln!("Found xacro namespace: {}", xacro_uri);
        }
    }
}

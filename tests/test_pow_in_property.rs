/// Test pow() function with load_yaml()
///
/// Reproduces corpus error: "Undefined variable: pow" when both load_yaml()
/// and pow() are used in the same file
mod common;
use crate::common::*;
use std::fs;
use std::path::Path;

#[test]
#[cfg(feature = "yaml")]
fn test_pow_with_load_yaml_minimal() {
    // HYPOTHESIS: load_yaml() + pow() together causes "Undefined variable: pow"
    // This reproduces the corpus case where measurements.xacro has load_yaml() line 4
    // and pow() on lines 61-70, then human_lower.xacro uses m*(3*pow(r,2))

    // Create minimal YAML file
    let yaml_content = "radius: 2.0\n";
    let yaml_path = "/tmp/test_pow_yaml.yaml";
    fs::write(yaml_path, yaml_content).expect("Failed to write YAML file");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="params" value="${load_yaml('/tmp/test_pow_yaml.yaml')}"/>
  <xacro:property name="r" value="${params['radius']}" />
  <xacro:property name="m" value="${pow(r,2)}" />

  <link name="test" value="${m*pow(r,2)}"/>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");

    // m = 2^2 = 4, expression = 4 * 4 = 16
    assert_eq!(get_attr(link, "value"), "16.0");

    // Cleanup
    let _ = fs::remove_file(yaml_path);
}

#[test]
#[cfg(feature = "yaml")]
fn test_pow_with_load_yaml_and_include() {
    // REAL corpus pattern: measurements.xacro has load_yaml() AND pow() in property definitions
    // Then human.urdf.xacro includes it and uses those properties with pow()

    // Create minimal YAML file
    let yaml_content = "length: 0.235\n";
    fs::write("/tmp/test_params.yaml", yaml_content).expect("Failed to write YAML");

    // Create included file with load_yaml() and pow() in property
    let included_content = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="measurements">
  <xacro:property name="params" value="${load_yaml('/tmp/test_params.yaml')}"/>
  <xacro:property name="l" value="${params['length']}" />
  <xacro:property name="r" value="${l}" />
  <xacro:property name="m" value="${pi*l*pow(r,2)}" />
</robot>"#;
    fs::write("/tmp/test_measurements.xacro", included_content).expect("Failed to write include");

    // Main file that includes and uses the properties with pow()
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/test_measurements.xacro" />

  <link name="test" value="${m*(3*pow(r,2)+pow(l,2))/12}"/>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");

    // m = pi * 0.235 * 0.235^2 ≈ 0.04
    // expression = m*(3*r^2+l^2)/12
    let value_str = get_attr(link, "value");
    let _value: f64 = value_str
        .parse()
        .expect("Should be a valid number, not 'Undefined variable: pow'");

    // Cleanup
    let _ = fs::remove_file("/tmp/test_params.yaml");
    let _ = fs::remove_file("/tmp/test_measurements.xacro");
}

#[test]
fn test_pow_in_include_without_yaml() {
    // Test if it's load_yaml() specifically or just includes

    // Create included file with pow() in property (NO load_yaml)
    let included_content = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="measurements">
  <xacro:property name="l" value="0.235" />
  <xacro:property name="r" value="${l}" />
  <xacro:property name="m" value="${pi*l*pow(r,2)}" />
</robot>"#;
    fs::write("/tmp/test_simple_include.xacro", included_content).expect("Failed to write include");

    // Main file that includes and uses the properties with pow()
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="/tmp/test_simple_include.xacro" />

  <link name="test" value="${m*(3*pow(r,2)+pow(l,2))/12}"/>
</robot>"#;

    let output = run_xacro(input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");

    let value_str = get_attr(link, "value");
    let _value: f64 = value_str
        .parse()
        .expect("Should be a valid number, not 'Undefined variable: pow'");

    // Cleanup
    let _ = fs::remove_file("/tmp/test_simple_include.xacro");
}

#[test]
fn test_pow_absolute_minimal() {
    // ABSOLUTE MINIMAL: 10 lines total
    // Property with pow() in included file, used with pow() in main file

    fs::write(
        "/tmp/inc.xacro",
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="m" value="${pow(2,2)}" />
</robot>"#,
    )
    .expect("write failed");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="/tmp/inc.xacro" />
  <link value="${m*pow(2,2)}"/>
</robot>"#;

    let output = run_xacro(input);
    assert!(output.contains("value=\"16"));

    fs::remove_file("/tmp/inc.xacro").ok();
}

#[test]
fn test_pow_with_property_args_in_include() {
    // THE KEY: pow() arguments are PROPERTIES, not literals
    // Include defines property using pow(PROPERTY), main uses pow(PROPERTY)

    fs::write(
        "/tmp/inc2.xacro",
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="r" value="2" />
  <xacro:property name="m" value="${pow(r,2)}" />
</robot>"#,
    )
    .expect("write failed");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="/tmp/inc2.xacro" />
  <link value="${m*pow(r,2)}"/>
</robot>"#;

    let output = run_xacro(input);
    assert!(output.contains("value=\"16"));

    fs::remove_file("/tmp/inc2.xacro").ok();
}

#[test]
fn test_pow_with_property_chain_and_pi() {
    use env_logger;
    let _ = env_logger::builder().is_test(true).try_init();

    // Add pi and property chain like corpus case
    fs::write(
        "/tmp/inc3.xacro",
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="l" value="0.235" />
  <xacro:property name="r" value="${l}" />
  <xacro:property name="m" value="${pi*l*pow(r,2)}" />
</robot>"#,
    )
    .expect("write failed");

    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="/tmp/inc3.xacro" />
  <link value="${m*(3*pow(r,2)+pow(l,2))/12}"/>
</robot>"#;

    let output = run_xacro(input);
    // Just check it doesn't crash
    assert!(!output.contains("Undefined"));

    fs::remove_file("/tmp/inc3.xacro").ok();
}

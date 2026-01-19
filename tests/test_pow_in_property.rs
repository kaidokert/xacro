/// Test pow() function with load_yaml()
///
/// Reproduces corpus error: "Undefined variable: pow" when both load_yaml()
/// and pow() are used in the same file
mod common;
use crate::common::*;
use std::fs;
use tempfile::NamedTempFile;

#[test]
#[cfg(feature = "yaml")]
fn test_pow_with_load_yaml_minimal() {
    // HYPOTHESIS: load_yaml() + pow() together causes "Undefined variable: pow"
    // This reproduces the corpus case where measurements.xacro has load_yaml() line 4
    // and pow() on lines 61-70, then human_lower.xacro uses m*(3*pow(r,2))

    // Create minimal YAML file
    let yaml_content = "radius: 2.0\n";
    let yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    fs::write(yaml_file.path(), yaml_content).expect("Failed to write YAML file");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:property name="params" value="${{load_yaml('{}')}}"/>
  <xacro:property name="r" value="${{params['radius']}}" />
  <xacro:property name="m" value="${{pow(r,2)}}" />

  <link name="test" value="${{m*pow(r,2)}}"/>
</robot>"#,
        yaml_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");

    // m = 2^2 = 4, expression = 4 * 4 = 16
    assert_eq!(get_attr(link, "value"), "16.0");

    // Temp file automatically cleaned up when yaml_file drops
}

#[test]
#[cfg(feature = "yaml")]
fn test_pow_with_load_yaml_and_include() {
    // REAL corpus pattern: measurements.xacro has load_yaml() AND pow() in property definitions
    // Then human.urdf.xacro includes it and uses those properties with pow()

    // Create minimal YAML file
    let yaml_content = "length: 0.235\n";
    let yaml_file = NamedTempFile::new().expect("Failed to create temp YAML file");
    fs::write(yaml_file.path(), yaml_content).expect("Failed to write YAML");

    // Create included file with load_yaml() and pow() in property
    let included_content = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="measurements">
  <xacro:property name="params" value="${{load_yaml('{}')}}"/>
  <xacro:property name="l" value="${{params['length']}}" />
  <xacro:property name="r" value="${{l}}" />
  <xacro:property name="m" value="${{pi*l*pow(r,2)}}" />
</robot>"#,
        yaml_file.path().display()
    );
    let include_file = NamedTempFile::new().expect("Failed to create temp xacro file");
    fs::write(include_file.path(), included_content).expect("Failed to write include");

    // Main file that includes and uses the properties with pow()
    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="{}" />

  <link name="test" value="${{m*(3*pow(r,2)+pow(l,2))/12}}"/>
</robot>"#,
        include_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");

    // m = pi * 0.235 * 0.235^2 ≈ 0.04
    // expression = m*(3*r^2+l^2)/12
    let value_str = get_attr(link, "value");
    let _value: f64 = value_str
        .parse()
        .expect("Should be a valid number, not 'Undefined variable: pow'");

    // Temp files automatically cleaned up when dropped
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
    let include_file = NamedTempFile::new().expect("Failed to create temp xacro file");
    fs::write(include_file.path(), included_content).expect("Failed to write include");

    // Main file that includes and uses the properties with pow()
    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
  <xacro:include filename="{}" />

  <link name="test" value="${{m*(3*pow(r,2)+pow(l,2))/12}}"/>
</robot>"#,
        include_file.path().display()
    );

    let output = run_xacro(&input);
    let root = parse_xml(&output);
    let link = find_child(&root, "link");

    let value_str = get_attr(link, "value");
    let _value: f64 = value_str
        .parse()
        .expect("Should be a valid number, not 'Undefined variable: pow'");

    // Temp file automatically cleaned up when dropped
}

#[test]
fn test_pow_absolute_minimal() {
    // ABSOLUTE MINIMAL: 10 lines total
    // Property with pow() in included file, used with pow() in main file

    let include_file = NamedTempFile::new().expect("Failed to create temp xacro file");
    fs::write(
        include_file.path(),
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="m" value="${pow(2,2)}" />
</robot>"#,
    )
    .expect("write failed");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{}" />
  <link value="${{m*pow(2,2)}}"/>
</robot>"#,
        include_file.path().display()
    );

    let output = run_xacro(&input);
    assert!(output.contains("value=\"16"));

    // Temp file automatically cleaned up when dropped
}

#[test]
fn test_pow_with_property_args_in_include() {
    // THE KEY: pow() arguments are PROPERTIES, not literals
    // Include defines property using pow(PROPERTY), main uses pow(PROPERTY)

    let include_file = NamedTempFile::new().expect("Failed to create temp xacro file");
    fs::write(
        include_file.path(),
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="r" value="2" />
  <xacro:property name="m" value="${pow(r,2)}" />
</robot>"#,
    )
    .expect("write failed");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{}" />
  <link value="${{m*pow(r,2)}}"/>
</robot>"#,
        include_file.path().display()
    );

    let output = run_xacro(&input);
    assert!(output.contains("value=\"16"));

    // Temp file automatically cleaned up when dropped
}

#[test]
fn test_pow_with_property_chain_and_pi() {
    use env_logger;
    let _ = env_logger::builder().is_test(true).try_init();

    // Add pi and property chain like corpus case
    let include_file = NamedTempFile::new().expect("Failed to create temp xacro file");
    fs::write(
        include_file.path(),
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="l" value="0.235" />
  <xacro:property name="r" value="${l}" />
  <xacro:property name="m" value="${pi*l*pow(r,2)}" />
</robot>"#,
    )
    .expect("write failed");

    let input = format!(
        r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{}" />
  <link value="${{m*(3*pow(r,2)+pow(l,2))/12}}"/>
</robot>"#,
        include_file.path().display()
    );

    let output = run_xacro(&input);
    // Just check it doesn't crash
    assert!(!output.contains("Undefined"));

    // Temp file automatically cleaned up when dropped
}

#[test]
fn test_pow_and_pi_in_string_literals_are_not_preprocessed() {
    use env_logger;
    let _ = env_logger::builder().is_test(true).try_init();

    // pow()/pi-like tokens inside quoted strings must not be rewritten by preprocessing
    let input = r#"<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- plain string containing pow(...) -->
  <xacro:property name="s1" value="print pow(2,3)" />
  <!-- pow(...) embedded via string interpolation, still as plain text -->
  <xacro:property name="s2" value="foo ${'pow(2,3)'}" />
  <xacro:property name="s3" value="pi is pi()" />
  <link name="test" s1="${s1}" s2="${s2}" s3="${s3}"/>
</robot>"#;

    let output = run_xacro(input);

    // Ensure the math-like tokens inside strings are emitted unchanged
    assert!(
        output.contains("print pow(2,3)"),
        "Expected pow(2,3) in plain string to be unchanged:\n{output}"
    );
    assert!(
        output.contains("foo pow(2,3)"),
        "Expected pow(2,3) coming from interpolation to be unchanged:\n{output}"
    );
    assert!(
        output.contains("pi is pi()"),
        "Expected pi() in string to be unchanged:\n{output}"
    );
}

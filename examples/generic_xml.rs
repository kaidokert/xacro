//! Generic XML macro processing (non-ROS example)
//!
//! This example demonstrates using xacro as a general-purpose XML macro
//! processor, without any ROS-specific features. Xacro can be used to add
//! properties, macros, and conditionals to any XML document.
//!
//! Run with:
//!   cargo run --example generic_xml

use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Example: Configuration file with repeated sections
    let config_template = r#"<?xml version="1.0"?>
<config xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Define reusable properties -->
  <xacro:property name="db_host" value="localhost"/>
  <xacro:property name="db_port" value="5432"/>
  <xacro:property name="max_connections" value="100"/>

  <!-- Macro for database configuration -->
  <xacro:macro name="database_config" params="name host port">
    <database>
      <name>${name}</name>
      <host>${host}</host>
      <port>${port}</port>
      <max_connections>${max_connections}</max_connections>
    </database>
  </xacro:macro>

  <!-- Generate multiple database configs -->
  <xacro:database_config name="primary" host="${db_host}" port="${db_port}"/>
  <xacro:database_config name="replica" host="${db_host}" port="${db_port + 1}"/>

  <!-- Conditional sections -->
  <xacro:property name="enable_cache" value="true"/>
  <xacro:if value="${enable_cache}">
    <cache>
      <enabled>true</enabled>
      <size>256MB</size>
    </cache>
  </xacro:if>
</config>"#;

    // Process the XML with xacro
    let expanded = xacro_rs::process_string(config_template)?;

    println!("Expanded XML configuration:");
    println!("{}", expanded);

    Ok(())
}

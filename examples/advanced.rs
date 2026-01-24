//! Advanced xacro processing with extensions and custom configuration
//!
//! This example demonstrates:
//! - Using ROS extension $(optenv) to read environment variables
//! - Loading configuration from YAML files with load_yaml()
//! - Passing arguments to xacro files
//! - Enabling YAML support with with_ros_yaml_units() (feature-gated)
//! - Compatibility modes for legacy URDFs
//! - Getting dependency information with run_with_deps()
//!
//! The example sets environment variables that are read by $(optenv) and
//! processes a xacro file that loads robot_config.yaml, demonstrating both
//! ROS extensions and YAML features working together.
//!
//! Run with:
//!   cargo run --example advanced --features yaml

use std::error::Error;
use xacro_rs::extensions::{FindExtension, OptEnvExtension};
use xacro_rs::{CompatMode, XacroProcessor};

fn main() -> Result<(), Box<dyn Error>> {
    // Set some environment variables for the $(optenv) extension to read
    std::env::set_var("ROBOT_VERSION", "2.5");
    std::env::set_var("ROBOT_ENV", "development");

    // Build a processor with all bells and whistles
    let mut builder = XacroProcessor::builder()
        .with_arg("robot_name", "advanced_robot")
        .with_compat_mode(CompatMode::all())
        .with_extension(Box::new(FindExtension::new()))
        .with_extension(Box::new(OptEnvExtension::new()));

    // Add YAML support if feature is enabled
    #[cfg(feature = "yaml")]
    {
        builder = builder.with_ros_yaml_units();
    }

    let processor = builder.build();

    // Process a file that uses ROS extensions and get its dependencies
    let (urdf, dependencies) = processor.run_with_deps("examples/robot_advanced.xacro")?;

    // Print the result
    println!("Generated URDF:");
    println!("{}", urdf);

    // Print dependencies (files that were included)
    if !dependencies.is_empty() {
        println!("\nDependencies:");
        for dep in dependencies {
            println!("  - {}", dep.display());
        }
    } else {
        println!("\nNo dependencies (no included files)");
    }

    Ok(())
}

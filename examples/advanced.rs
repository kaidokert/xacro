//! Advanced xacro processing with extensions and custom configuration
//!
//! This example demonstrates:
//! - Using ROS extensions ($(find), $(optenv))
//! - Enabling YAML support
//! - Compatibility modes
//! - Getting dependency information
//!
//! Run with:
//!   cargo run --example advanced --features yaml

use std::error::Error;
use xacro::extensions::{FindExtension, OptEnvExtension};
use xacro::{CompatMode, XacroProcessor};

fn main() -> Result<(), Box<dyn Error>> {
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

    // Process a file with arguments and also get its dependencies
    let (urdf, dependencies) = processor.run_with_deps("examples/robot_with_args.xacro")?;

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

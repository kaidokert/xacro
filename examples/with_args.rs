//! Processing xacro files with command-line arguments
//!
//! This example shows how to use the builder API to pass arguments to xacro,
//! similar to the CLI's `key:=value` syntax.
//!
//! Run with:
//!   cargo run --example with_args

use std::error::Error;
use xacro_rs::XacroProcessor;

fn main() -> Result<(), Box<dyn Error>> {
    // Build a processor with custom arguments
    let processor = XacroProcessor::builder()
        .with_arg("robot_name", "my_robot")
        .with_arg("wheel_radius", "0.05")
        .with_arg("debug_mode", "true")
        .build();

    // Process the file with these arguments available
    let urdf = processor.run("examples/robot_with_args.xacro")?;

    println!("{}", urdf);

    Ok(())
}

//! Basic xacro file processing
//!
//! This example demonstrates the simplest way to process a xacro file.
//!
//! Run with:
//!   cargo run --example basic

use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Process a xacro file and get the expanded URDF XML
    let urdf = xacro_rs::process_file("examples/robot.xacro")?;

    // Print the result
    println!("{}", urdf);

    Ok(())
}

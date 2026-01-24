//! Processing xacro content from stdin
//!
//! This example demonstrates reading xacro XML from standard input,
//! useful for pipeline processing or programmatic generation.
//!
//! Run with:
//!   echo '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test"/>' | cargo run --example stdin
//!   cargo run --example stdin < examples/robot.xacro

use std::error::Error;
use std::io::{self, Read};

fn main() -> Result<(), Box<dyn Error>> {
    // Read xacro XML from stdin
    let mut input = String::new();
    io::stdin().read_to_string(&mut input)?;

    // Process the xacro string
    let urdf = xacro_rs::process_string(&input)?;

    // Print the result
    println!("{}", urdf);

    Ok(())
}

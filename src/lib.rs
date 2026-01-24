//! [Xacro](https://wiki.ros.org/xacro) processor in Rust.
//!
//! `xacro` is an XML macro processor, that expands properties, macros, and conditionals.
//! The crate provides core extensions by default (cwd, env); ROS-specific extensions
//! are available via the builder when needed.
//!
//! # Quick Start
//!
//! Process a xacro file:
//!
//! ```no_run
//! # fn main() -> Result<(), xacro::XacroError> {
//! let urdf = xacro::process_file("robot.xacro")?;
//! # Ok(())
//! # }
//! ```
//!
//! Use [`process_string`] for string content. For more control (arguments, extensions, compatibility modes):
//!
//! ```no_run
//! # fn main() -> Result<(), xacro::XacroError> {
//! let processor = xacro::XacroProcessor::builder()
//!     .with_arg("robot_name", "my_robot")
//!     .build();
//! let urdf = processor.run("robot.xacro")?;
//! # Ok(())
//! # }
//! ```
//!
//! # Examples
//!
//! See the [`examples/`](https://github.com/kaidokert/xacro/tree/main/examples)
//! directory for complete examples including:
//! - Basic file processing
//! - Processing from stdin
//! - Using arguments and the builder API
//! - ROS extensions and YAML support
//! - Generic XML macros (non-ROS usage)
//!
//! # Feature Flags
//!
//! - `yaml` (default): Enable YAML loading with `load_yaml()`
//! - `compat` (default): Python Xacro compatibility mode.
//!
//! # Compatibility
//!
//! This crate aims for feature parity with [Python xacro](https://wiki.ros.org/xacro).
//! See the [README](https://github.com/kaidokert/xacro#readme) for status and limitations.

#![forbid(unsafe_code)]
// #![warn(clippy::pedantic)]
#![warn(clippy::alloc_instead_of_core)]
#![warn(clippy::std_instead_of_core)]

mod directives;
pub mod error;
mod eval;
mod expand;
mod expander;
pub mod extensions;
mod parse;
pub mod processor;

#[cfg(test)]
pub mod test_utils;

pub use error::XacroError;
pub use eval::scope::PropertyScope;
pub use processor::{CompatMode, XacroBuilder, XacroProcessor};

/// Process a xacro file from the filesystem.
///
/// This is a convenience function that creates a default [`XacroProcessor`]
/// and processes the given file path.
///
/// For more control over processing (arguments, compatibility modes, extensions),
/// use [`XacroProcessor::builder()`] instead.
///
/// # Examples
///
/// ```no_run
/// # fn main() -> Result<(), xacro::XacroError> {
/// let urdf = xacro::process_file("robot.xacro")?;
/// println!("{}", urdf);
/// # Ok(())
/// # }
/// ```
pub fn process_file<P: AsRef<std::path::Path>>(path: P) -> Result<String, XacroError> {
    let processor = XacroProcessor::new();
    processor.run(path)
}

/// Process xacro content from a string.
///
/// This is a convenience function that creates a default [`XacroProcessor`]
/// and processes the given xacro content string.
///
/// For more control over processing (arguments, compatibility modes, extensions),
/// use [`XacroProcessor::builder()`] instead.
///
/// # Examples
///
/// ```
/// # fn main() -> Result<(), xacro::XacroError> {
/// let xacro_content = r#"<?xml version="1.0"?>
/// <robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">
///   <xacro:property name="width" value="0.5"/>
///   <link name="base">
///     <visual>
///       <geometry>
///         <box size="${width} 0.5 0.5"/>
///       </geometry>
///     </visual>
///   </link>
/// </robot>"#;
///
/// let urdf = xacro::process_string(xacro_content)?;
/// assert!(urdf.contains("0.5 0.5 0.5"));
/// # Ok(())
/// # }
/// ```
pub fn process_string(content: &str) -> Result<String, XacroError> {
    let processor = XacroProcessor::new();
    processor.run_from_string(content)
}

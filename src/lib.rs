#![forbid(unsafe_code)]
// #![warn(clippy::pedantic)]
#![warn(clippy::alloc_instead_of_core)]
#![warn(clippy::std_instead_of_core)]

pub mod error;
pub mod expander;
pub mod features;
pub mod processor;
pub mod utils;

pub use error::XacroError;
pub use processor::{CompatMode, XacroProcessor};

pub fn process_file<P: AsRef<std::path::Path>>(path: P) -> Result<String, XacroError> {
    let processor = XacroProcessor::new();
    processor.run(path)
}

#![forbid(unsafe_code)]
// #![warn(clippy::pedantic)]
#![warn(clippy::alloc_instead_of_core)]
#![warn(clippy::std_instead_of_core)]

pub mod directives;
pub mod error;
pub mod eval;
pub mod expand;
pub mod expander;
pub mod extensions;
pub mod parse;
pub mod processor;

pub use error::XacroError;
pub use processor::{CompatMode, XacroBuilder, XacroProcessor};

pub fn process_file<P: AsRef<std::path::Path>>(path: P) -> Result<String, XacroError> {
    let processor = XacroProcessor::new();
    processor.run(path)
}

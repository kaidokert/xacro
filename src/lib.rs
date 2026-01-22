#![forbid(unsafe_code)]
// #![warn(clippy::pedantic)]
#![warn(clippy::alloc_instead_of_core)]
#![warn(clippy::std_instead_of_core)]

mod directives;
pub mod error;
mod eval;
mod expand;
mod expander;
pub mod extensions; // Public: ExtensionHandler trait for custom extensions
mod parse;
pub mod processor;

#[cfg(test)]
pub mod test_utils;

pub use error::XacroError;
pub use eval::scope::PropertyScope;
pub use processor::{CompatMode, XacroBuilder, XacroProcessor};

pub fn process_file<P: AsRef<std::path::Path>>(path: P) -> Result<String, XacroError> {
    let processor = XacroProcessor::new();
    processor.run(path)
}

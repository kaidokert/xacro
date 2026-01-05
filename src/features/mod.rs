pub mod macros;
pub mod properties;

/// Single source of truth for all xacro directive names
///
/// This ensures that directive lists stay synchronized across the codebase.
/// All other directive lists (in error.rs and expander.rs) are derived from these.
///
/// Implemented xacro directives (element names without "xacro:" prefix)
pub const IMPLEMENTED_DIRECTIVES: &[&str] = &[
    "property",
    "macro",
    "if",
    "unless",
    "include",
    "insert_block",
];

/// Unimplemented xacro directives (element names without "xacro:" prefix)
pub const UNIMPLEMENTED_DIRECTIVES: &[&str] = &["arg", "element", "attribute"];

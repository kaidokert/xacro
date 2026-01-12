//! Single source of truth for xacro directive names
//!
//! This module generates both unprefixed directive names (for internal matching)
//! and prefixed names (for user-facing error messages) from a single definition.

macro_rules! define_directives {
    ($name:ident, $prefixed_name:ident, [$($directive:literal),* $(,)?]) => {
        pub const $name: &[&str] = &[$($directive),*];
        pub const $prefixed_name: &[&str] = &[$(concat!("xacro:", $directive)),*];
    };
}

define_directives!(
    IMPLEMENTED_DIRECTIVES,
    IMPLEMENTED_FEATURES,
    [
        "property",
        "macro",
        "if",
        "unless",
        "include",
        "insert_block",
        "arg",
    ]
);

define_directives!(
    UNIMPLEMENTED_DIRECTIVES,
    UNIMPLEMENTED_FEATURES,
    ["element", "attribute",]
);

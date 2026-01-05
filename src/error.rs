#[derive(Debug, thiserror::Error)]
pub enum XacroError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("XML error: {0}")]
    Xml(#[from] xmltree::ParseError),

    #[error("Include error: {0}")]
    Include(String),

    #[error("Macro error: {0}")]
    UndefinedMacro(String),

    #[error("Missing parameter '{param}' in macro '{macro_name}'")]
    MissingParameter { macro_name: String, param: String },

    #[error("Missing attribute '{attribute}' in element '{element}'")]
    MissingAttribute { element: String, attribute: String },

    #[error("Macro error: {0}")]
    PropertyNotFound(String),

    #[error("Evaluation error in '{expr}': {source}")]
    EvalError {
        expr: String,
        #[source]
        source: crate::utils::eval::EvalError,
    },

    #[error("XML write error: {0}")]
    XmlWrite(#[from] xmltree::Error),

    #[error("UTF-8 conversion error: {0}")]
    Utf8(#[from] std::string::FromUtf8Error),

    #[error("Macro recursion limit exceeded: depth {depth} > {limit} (possible infinite loop)")]
    MacroRecursionLimit { depth: usize, limit: usize },

    #[error("Block parameter '{param}' cannot have a default value")]
    BlockParameterWithDefault { param: String },

    #[error("Invalid parameter name: '{param}' (parameter names cannot be empty)")]
    InvalidParameterName { param: String },

    #[error("Unbalanced quote in macro parameters: unclosed {quote_char} quote in '{params_str}'")]
    UnbalancedQuote {
        quote_char: char,
        params_str: String,
    },

    #[error("Missing block parameter '{param}' in macro '{macro_name}'")]
    MissingBlockParameter { macro_name: String, param: String },

    #[error("Unused block in macro '{macro_name}' (provided {extra_count} extra child elements)")]
    UnusedBlock {
        macro_name: String,
        extra_count: usize,
    },

    #[error("Undefined block '{name}'")]
    UndefinedBlock { name: String },

    #[error("Duplicate parameter declaration: '{param}' (parameter declared multiple times or with conflicting block/non-block forms)")]
    DuplicateParamDeclaration { param: String },

    #[error("Block parameter '{param}' cannot be specified as an attribute (it must be provided as a child element)")]
    BlockParameterAttributeCollision { param: String },

    #[error("Unimplemented xacro feature: {0}")]
    UnimplementedFeature(String),

    #[error("Missing xacro namespace declaration: {0}")]
    MissingNamespace(String),

    /// Circular property dependency detected during lazy evaluation
    ///
    /// The `chain` field contains the dependency path formatted as "a -> b -> c -> a"
    /// showing how the circular reference was formed.
    #[error("Circular property dependency detected: {chain}")]
    CircularPropertyDependency { chain: String },

    #[error("Undefined property: '{0}'")]
    UndefinedProperty(String),

    /// Undefined argument accessed via $(arg name)
    ///
    /// The user tried to access an argument that was not defined in XML
    /// and was not provided via CLI.
    #[error(
        "Undefined argument: '{name}'.\n\
             \n\
             To fix this:\n\
             1. Define it in XML: <xacro:arg name=\"{name}\" default=\"...\"/>\n\
             2. Or pass it via CLI: {name}:=value"
    )]
    UndefinedArgument { name: String },

    /// Unknown extension type (not arg/find/env)
    #[error(
        "Unknown extension type: '$({} ...)'.\n\
             \n\
             Supported extensions:\n\
             - $(arg name)  - Access xacro argument\n\
             - $(find pkg)  - Not yet implemented\n\
             - $(env VAR)   - Not yet implemented",
        ext_type
    )]
    UnknownExtension { ext_type: String },

    /// Invalid extension syntax
    #[error(
        "Invalid extension syntax: '$({})'.\n\
             \n\
             {}",
        content,
        reason
    )]
    InvalidExtension { content: String, reason: String },

    /// Property substitution exceeded maximum depth
    ///
    /// Indicates that iterative property substitution did not converge within the
    /// allowed number of iterations. This usually means circular or self-referential
    /// property definitions that cannot be fully resolved.
    #[error("Property substitution exceeded maximum depth of {depth} iterations. Remaining unresolved expressions in: {snippet}")]
    MaxSubstitutionDepth { depth: usize, snippet: String },

    /// Invalid root element after expansion
    ///
    /// The root element must expand to exactly one element node. This error indicates
    /// that expansion resulted in multiple nodes, zero nodes, or a non-element node.
    #[error("Invalid root element: {0}")]
    InvalidRoot(String),
}

// Feature lists for consistent error messages
// These are derived from the single source of truth in features/mod.rs
// with the "xacro:" prefix added for display purposes

pub const IMPLEMENTED_FEATURES: &[&str] = &[
    "xacro:property",
    "xacro:macro",
    "xacro:if",
    "xacro:unless",
    "xacro:include",
    "xacro:insert_block",
    "xacro:arg", // NEW - Phase 5
];

pub const UNIMPLEMENTED_FEATURES: &[&str] = &["xacro:element", "xacro:attribute"];

/// Helper function to create consistent UnimplementedFeature error messages
pub fn unimplemented_feature_error(feature: &str) -> XacroError {
    XacroError::UnimplementedFeature(format!(
        "<xacro:{}> is not implemented yet.\n\
         \n\
         Currently implemented: {}\n\
         Not yet implemented: {}",
        feature,
        IMPLEMENTED_FEATURES.join(", "),
        UNIMPLEMENTED_FEATURES.join(", ")
    ))
}

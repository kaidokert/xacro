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
        source: crate::eval::EvalError,
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

    #[error("Duplicate parameter declaration: '{param}'\n\nParameter names must be unique. Duplicate parameters are ambiguous and can lead to\nunexpected behavior in other xacro implementations.\n\nTo accept duplicates (last declaration wins), use:\n  xacro --compat <file>")]
    DuplicateParamDeclaration { param: String },

    #[error("Block parameter '{param}' cannot be specified as an attribute (it must be provided as a child element)")]
    BlockParameterAttributeCollision { param: String },

    #[error("Invalid macro parameter '{param}': {reason}")]
    InvalidMacroParameter { param: String, reason: String },

    #[error("Invalid forward syntax in parameter '{param}': {hint}")]
    InvalidForwardSyntax { param: String, hint: String },

    #[error("Macro '{macro_name}' parameter '{param}' declared with ^ to forward '{forward_name}' but not found in parent scope")]
    UndefinedPropertyToForward {
        macro_name: String,
        param: String,
        forward_name: String,
    },

    #[error(
        "Invalid scope attribute '{scope}' for property '{property}': must be 'parent' or 'global'"
    )]
    InvalidScopeAttribute { property: String, scope: String },

    /// YAML file loading failed
    #[cfg(feature = "yaml")]
    #[error("Failed to load YAML file '{path}': {source}")]
    YamlLoadError {
        path: String,
        #[source]
        source: std::io::Error,
    },

    /// YAML parsing failed
    #[cfg(feature = "yaml")]
    #[error("Failed to parse YAML file '{path}': {message}")]
    YamlParseError { path: String, message: String },

    /// YAML feature not enabled
    #[cfg(not(feature = "yaml"))]
    #[error(
        "load_yaml() requires 'yaml' feature.\n\
         \n\
         To enable YAML support, rebuild with:\n\
         cargo build --features yaml"
    )]
    YamlFeatureDisabled,

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

    /// Unknown extension type
    ///
    /// This error occurs when a $(command ...) substitution uses an unrecognized command.
    /// The set of available extensions depends on how the processor was configured.
    ///
    /// Core extensions (always available):
    /// - $(arg name)  - Access xacro argument
    /// - $(cwd)       - Get current working directory
    /// - $(env VAR)   - Get environment variable
    ///
    /// Additional extensions may be available if explicitly added via builder pattern.
    #[error("Unknown extension type: '$({} ...)'", ext_type)]
    UnknownExtension { ext_type: String },

    /// Extension resolution failed
    #[error(
        "Failed to resolve extension: '$({})'.\n\
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

    /// Invalid XML content
    ///
    /// The content violates XML specification rules (e.g., forbidden sequences in comments,
    /// CDATA sections, or processing instructions).
    #[error("Invalid XML: {0}")]
    InvalidXml(String),
}

// Implement From trait for EvalError to avoid duplicated error mapping
pub use crate::eval::EvalError;
impl From<crate::eval::EvalError> for XacroError {
    fn from(e: crate::eval::EvalError) -> Self {
        XacroError::EvalError {
            expr: match &e {
                crate::eval::EvalError::PyishEval { expr, .. } => expr.clone(),
                crate::eval::EvalError::InvalidBoolean { condition, .. } => condition.clone(),
            },
            source: e,
        }
    }
}

// Feature lists for consistent error messages
// Re-exported from directives module (single source of truth)
pub use crate::directives::{IMPLEMENTED_FEATURES, UNIMPLEMENTED_FEATURES};

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

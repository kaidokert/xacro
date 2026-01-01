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

    #[error("Macro recursion limit exceeded: depth {depth} > {limit} (possible infinite loop)")]
    MacroRecursionLimit { depth: usize, limit: usize },

    #[error("Block parameter '{param}' cannot have a default value")]
    BlockParameterWithDefault { param: String },

    #[error("Missing block parameter '{param}' in macro '{macro_name}'")]
    MissingBlockParameter { macro_name: String, param: String },

    #[error("Unused block in macro '{macro_name}' (provided {extra_count} extra child elements)")]
    UnusedBlock {
        macro_name: String,
        extra_count: usize,
    },

    #[error("Undefined block '{name}'")]
    UndefinedBlock { name: String },
}

//! Expansion state and context management

use crate::{
    error::XacroError, eval::EvalContext, parse::macro_def::MacroDefinition, processor::CompatMode,
};
use core::cell::RefCell;
use std::{collections::HashMap, path::PathBuf, rc::Rc};
use xmltree::XMLNode;

pub struct XacroContext {
    /// Property processor with scope support
    pub properties: EvalContext,

    /// Macro definitions wrapped in Rc - uses RefCell for interior mutability
    pub macros: RefCell<HashMap<String, Rc<MacroDefinition>>>,

    /// CLI arguments (shared with EvalContext for $(arg) resolution)
    /// Wrapped in Rc<RefCell<...>> for shared mutable access
    pub args: Rc<RefCell<HashMap<String, String>>>,

    /// Include stack for circular include detection (uses RefCell for interior mutability)
    pub include_stack: RefCell<Vec<PathBuf>>,

    /// Namespace stack: (file_path, xacro_namespace_prefix) (uses RefCell for interior mutability)
    pub namespace_stack: RefCell<Vec<(PathBuf, String)>>,

    /// Block stack for insert_block arguments (uses RefCell for interior mutability)
    /// Stores pre-expanded XMLNode content for each block parameter
    pub block_stack: RefCell<Vec<HashMap<String, Vec<XMLNode>>>>,

    /// Current base path for resolving relative includes (uses RefCell for interior mutability)
    pub base_path: RefCell<PathBuf>,

    /// Current overall recursion depth (uses RefCell for interior mutability to enable RAII guards)
    pub recursion_depth: RefCell<usize>,

    /// Maximum recursion depth before triggering error
    /// Set conservatively to prevent stack overflow before the check triggers
    pub max_recursion_depth: usize,

    /// Python xacro compatibility modes
    pub compat_mode: CompatMode,
}

impl XacroContext {
    /// Default maximum recursion depth
    /// Set conservatively to prevent stack overflow before the check triggers
    pub const DEFAULT_MAX_DEPTH: usize = 50;

    /// Create a new context with the given base path
    pub fn new(
        base_path: PathBuf,
        xacro_ns: String,
    ) -> Self {
        Self::new_with_compat(base_path, xacro_ns, HashMap::new(), CompatMode::none())
    }

    /// Create a new context with CLI arguments
    ///
    /// The args parameter is shared with the EvalContext for $(arg) resolution.
    pub fn new_with_args(
        base_path: PathBuf,
        xacro_ns: String,
        args: HashMap<String, String>,
    ) -> Self {
        Self::new_with_compat(base_path, xacro_ns, args, CompatMode::none())
    }

    /// Create a new context with compatibility mode
    ///
    /// The compat_mode parameter enables Python xacro compatibility features.
    /// The args parameter is shared with the EvalContext for $(arg) resolution.
    pub fn new_with_compat(
        base_path: PathBuf,
        xacro_ns: String,
        args: HashMap<String, String>,
        compat_mode: CompatMode,
    ) -> Self {
        // Wrap args in Rc<RefCell<...>> for shared mutable access
        let args = Rc::new(RefCell::new(args));

        XacroContext {
            properties: EvalContext::new_with_args(args.clone()),
            macros: RefCell::new(HashMap::new()),
            args,
            include_stack: RefCell::new(Vec::new()),
            namespace_stack: RefCell::new(vec![(base_path.clone(), xacro_ns)]),
            block_stack: RefCell::new(Vec::new()),
            base_path: RefCell::new(base_path),
            recursion_depth: RefCell::new(0),
            max_recursion_depth: Self::DEFAULT_MAX_DEPTH,
            compat_mode,
        }
    }

    /// Get the current xacro namespace prefix
    ///
    /// Returns the namespace prefix from the top of the namespace stack.
    /// This is the prefix used for xacro directives in the current file.
    pub fn current_xacro_ns(&self) -> String {
        self.namespace_stack
            .borrow()
            .last()
            .map(|(_, ns)| ns.clone())
            .unwrap_or_else(|| "xacro".to_string())
    }

    /// Get a block parameter value from the block stack
    ///
    /// Searches the block stack from top to bottom for a block with the given name.
    /// Returns a clone of the block's content (Vec<XMLNode>) if found.
    ///
    /// # Errors
    ///
    /// Returns `XacroError::UndefinedBlock` if no block with the given name is found.
    pub fn get_block(
        &self,
        name: &str,
    ) -> Result<Vec<XMLNode>, XacroError> {
        self.block_stack
            .borrow()
            .iter()
            .rev()
            .find_map(|blocks| blocks.get(name))
            .cloned()
            .ok_or_else(|| XacroError::UndefinedBlock {
                name: name.to_string(),
            })
    }

    /// Set the maximum recursion depth
    pub fn set_max_recursion_depth(
        &mut self,
        depth: usize,
    ) {
        self.max_recursion_depth = depth;
    }
}

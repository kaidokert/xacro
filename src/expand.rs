//! Expansion state and context management

use crate::{
    error::XacroError, eval::EvalContext, extensions::ExtensionHandler,
    parse::macro_def::MacroDefinition, processor::CompatMode,
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

    /// All included files (for --deps output)
    pub all_includes: RefCell<Vec<PathBuf>>,

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

    /// Create a new context with the given base path (for testing).
    ///
    /// This is a minimal constructor used by unit tests. Production code should use
    /// `new_with_extensions()` which properly integrates with the extension system.
    pub fn new(
        base_path: PathBuf,
        xacro_ns: String,
    ) -> Self {
        let args = Rc::new(RefCell::new(HashMap::new()));
        let extensions = Rc::new(Vec::new()); // Empty extensions for tests

        XacroContext {
            properties: EvalContext::new_with_extensions(args.clone(), extensions),
            macros: RefCell::new(HashMap::new()),
            args,
            include_stack: RefCell::new(Vec::new()),
            all_includes: RefCell::new(Vec::new()),
            namespace_stack: RefCell::new(vec![(base_path.clone(), xacro_ns)]),
            block_stack: RefCell::new(Vec::new()),
            base_path: RefCell::new(base_path),
            recursion_depth: RefCell::new(0),
            max_recursion_depth: Self::DEFAULT_MAX_DEPTH,
            compat_mode: CompatMode::none(),
        }
    }

    /// Create a new context with custom extensions
    ///
    /// This constructor allows providing custom extension handlers.
    ///
    /// # Arguments
    /// * `base_path` - Base path for resolving relative includes
    /// * `xacro_ns` - Xacro namespace prefix
    /// * `args` - Shared reference to CLI arguments (wrapped in Rc<RefCell<...>>)
    /// * `compat_mode` - Compatibility mode
    /// * `extensions` - Custom extension handlers (wrapped in Rc for sharing)
    pub fn new_with_extensions(
        base_path: PathBuf,
        xacro_ns: String,
        args: Rc<RefCell<HashMap<String, String>>>,
        compat_mode: CompatMode,
        extensions: Rc<Vec<Box<dyn ExtensionHandler>>>,
    ) -> Self {
        XacroContext {
            properties: EvalContext::new_with_extensions(args.clone(), extensions),
            macros: RefCell::new(HashMap::new()),
            args,
            include_stack: RefCell::new(Vec::new()),
            all_includes: RefCell::new(Vec::new()),
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
            .expect("namespace_stack should never be empty - initialized in XacroContext::new()")
    }

    /// Look up a named block from the current macro scope
    /// Returns pre-expanded XMLNodes
    pub fn lookup_block(
        &self,
        name: &str,
    ) -> Result<Vec<XMLNode>, XacroError> {
        // FIX Bug #2: Search entire block stack (most recent to oldest)
        // This allows nested macros to access block parameters from parent macros
        let stack = self.block_stack.borrow();
        for blocks in stack.iter().rev() {
            if let Some(nodes) = blocks.get(name) {
                return Ok(nodes.clone());
            }
        }

        // Not found in block stack
        Err(XacroError::UndefinedBlock {
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

    /// Get all included files (for --deps output)
    ///
    /// Returns a sorted, deduplicated list of all files included during processing.
    /// Sorting ensures deterministic output (Python xacro's set() has non-deterministic
    /// hash-based ordering). This improves on Python's behavior for reproducibility.
    pub fn get_all_includes(&self) -> Vec<PathBuf> {
        let mut includes = self.all_includes.borrow().clone();
        includes.sort();
        includes
    }
}

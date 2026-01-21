//! RAII Guards for Panic-Safe Stack Management
//!
//! These guards ensure that pushed state (scopes, stacks, depth counters) is
//! always popped/restored when the guard goes out of scope, even if a panic
//! occurs during expansion. This prevents the XacroContext from being left in
//! a corrupted state.

use crate::eval::EvalContext;
use core::cell::RefCell;
use std::collections::HashMap;
use std::path::PathBuf;
use xmltree::XMLNode;

/// RAII guard for recursion depth tracking
///
/// Automatically decrements recursion_depth when dropped, ensuring correct
/// depth tracking even if expansion panics.
pub(crate) struct DepthGuard<'a> {
    pub(crate) depth: &'a RefCell<usize>,
}

impl<'a> DepthGuard<'a> {
    /// Creates a new DepthGuard and increments the depth counter
    pub(crate) fn new(depth: &'a RefCell<usize>) -> Self {
        *depth.borrow_mut() += 1;
        Self { depth }
    }
}

impl Drop for DepthGuard<'_> {
    fn drop(&mut self) {
        let mut depth = self.depth.borrow_mut();
        *depth = depth.saturating_sub(1);
    }
}

/// RAII guard for include stack, base path, and namespace stack
///
/// Automatically restores base_path and pops include_stack and namespace_stack when dropped,
/// ensuring correct file context and per-file namespace isolation even if include expansion panics.
pub(crate) struct IncludeGuard<'a> {
    pub(crate) base_path: &'a RefCell<PathBuf>,
    pub(crate) include_stack: &'a RefCell<Vec<PathBuf>>,
    pub(crate) namespace_stack: &'a RefCell<Vec<(PathBuf, String)>>,
    pub(crate) old_base_path: PathBuf,
    pub(crate) include_stack_len: usize,
    pub(crate) namespace_stack_len: usize,
}

impl Drop for IncludeGuard<'_> {
    fn drop(&mut self) {
        *self.base_path.borrow_mut() = self.old_base_path.clone();

        // Only pop if stacks grew (handles partial state from panics during push)
        let mut include = self.include_stack.borrow_mut();
        if include.len() > self.include_stack_len {
            include.pop();
        }

        let mut namespace = self.namespace_stack.borrow_mut();
        if namespace.len() > self.namespace_stack_len {
            namespace.pop();
        }
    }
}

/// RAII guard for property scopes
///
/// Automatically pops property scope when dropped, ensuring correct variable
/// shadowing even if macro expansion panics.
pub(crate) struct ScopeGuard<'a> {
    pub(crate) properties: &'a EvalContext,
}

impl Drop for ScopeGuard<'_> {
    fn drop(&mut self) {
        self.properties.pop_scope();
    }
}

/// RAII guard for block stacks
///
/// Automatically pops block stack when dropped, ensuring correct block
/// resolution even if macro expansion panics.
pub(crate) struct BlockGuard<'a> {
    pub(crate) blocks: &'a RefCell<Vec<HashMap<String, Vec<XMLNode>>>>,
}

impl Drop for BlockGuard<'_> {
    fn drop(&mut self) {
        self.blocks.borrow_mut().pop();
    }
}

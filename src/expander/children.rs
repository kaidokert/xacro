//! Children expansion helpers
//!
//! This module provides utilities for expanding child nodes of elements.

use crate::error::XacroError;
use xmltree::XMLNode;

use super::{expand_node, XacroContext};

/// Expand a list of child nodes
///
/// Iterates through nodes, expanding each one and flattening results.
/// This handles the common pattern where one input node can expand to
/// multiple output nodes (e.g., macro calls, conditionals).
///
/// # Arguments
/// * `children` - List of nodes to expand
/// * `ctx` - XacroContext with properties, macros, and stacks
///
/// # Returns
/// Flattened list of expanded nodes
pub(crate) fn expand_children_list(
    children: Vec<XMLNode>,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    children.into_iter().try_fold(Vec::new(), |mut acc, child| {
        acc.extend(expand_node(child, ctx)?);
        Ok::<Vec<XMLNode>, XacroError>(acc)
    })
}

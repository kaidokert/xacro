//! Include directive handling and file resolution
//!
//! This module provides functionality for processing `xacro:include` directives,
//! including glob pattern resolution, optional includes, and circular include detection.

use crate::{error::XacroError, parse::xml::extract_xacro_namespace};
use std::path::PathBuf;
use xmltree::{Element, XMLNode};

use super::*;

/// Check if a filename contains glob pattern characters
///
/// Python xacro regex: `re.search('[*[?]+', filename_spec)`
/// Detects wildcard patterns: *, [, or ?
pub(super) fn is_glob_pattern(filename: &str) -> bool {
    filename.contains('*') || filename.contains('[') || filename.contains('?')
}

/// Process a single include file
///
/// Handles:
/// - Circular include detection
/// - File reading and XML parsing
/// - Namespace extraction
/// - Include stack management with RAII guard
///
/// # Arguments
/// * `file_path` - Absolute path to the file to include
/// * `ctx` - XacroContext with include stack and namespace stack
///
/// # Returns
/// Expanded nodes from the included file
fn process_single_include(
    file_path: PathBuf,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Check for circular includes
    if ctx.include_stack.borrow().contains(&file_path) {
        return Err(XacroError::Include(format!(
            "Circular include detected: {}",
            file_path.display()
        )));
    }

    // Read and parse included file
    let content = std::fs::read_to_string(&file_path).map_err(|e| {
        XacroError::Include(format!(
            "Failed to read file '{}': {}",
            file_path.display(),
            e
        ))
    })?;

    let included_root = Element::parse(content.as_bytes()).map_err(|e| {
        XacroError::Include(format!(
            "Failed to parse XML in file '{}': {}",
            file_path.display(),
            e
        ))
    })?;

    // Extract xacro namespace from included file
    // Use the same namespace validation mode as the parent document
    let included_ns = extract_xacro_namespace(&included_root, ctx.compat_mode.namespace)?;

    // Push to include stack with RAII guard for automatic cleanup
    let old_base_path = ctx.base_path.borrow().clone();
    let mut new_base_path = file_path.clone();
    new_base_path.pop();

    // Create RAII guard BEFORE state mutations to ensure cleanup on panic
    // Capture current stack lengths to detect partial pushes
    let _include_guard = IncludeGuard {
        base_path: &ctx.base_path,
        include_stack: &ctx.include_stack,
        namespace_stack: &ctx.namespace_stack,
        old_base_path,
        include_stack_len: ctx.include_stack.borrow().len(),
        namespace_stack_len: ctx.namespace_stack.borrow().len(),
    };

    // Now perform state updates (guard will restore state on panic)
    *ctx.base_path.borrow_mut() = new_base_path;
    ctx.include_stack.borrow_mut().push(file_path.clone());

    // Track included file. Deduplication is handled by get_all_includes().
    ctx.all_includes.borrow_mut().push(file_path.clone());

    ctx.namespace_stack
        .borrow_mut()
        .push((file_path.clone(), included_ns));

    // Expand children and return
    expand_children_list(included_root.children, ctx)
}

/// Handle `xacro:include` directive
///
/// Supports:
/// - Glob patterns (*, [, ?)
/// - Optional includes (optional="true")
/// - Circular include detection
///
/// Python xacro has 3 different behaviors:
/// 1. Glob patterns with no matches → warn, continue
/// 2. optional="true" attribute → silent skip if not found
/// 3. Regular missing file → error
///
/// # Arguments
/// * `elem` - The `<xacro:include>` element
/// * `ctx` - XacroContext with properties and include stack
///
/// # Returns
/// Expanded nodes from included file(s), or empty vec if no match/optional
pub(crate) fn handle_include_directive(
    elem: Element,
    ctx: &XacroContext,
) -> Result<Vec<XMLNode>, XacroError> {
    // Extract filename and substitute expressions
    let filename = ctx
        .properties
        .substitute_text(elem.get_attribute("filename").ok_or_else(|| {
            XacroError::MissingAttribute {
                element: "xacro:include".to_string(),
                attribute: "filename".to_string(),
            }
        })?)?;

    // Check for optional attribute (default: false)
    let optional = elem
        .get_attribute("optional")
        .map(|v| v == "true" || v == "1")
        .unwrap_or(false);

    // Case 1: Glob pattern (detected by *, [, or ? characters)
    if is_glob_pattern(&filename) {
        // Resolve glob pattern relative to base_path
        let glob_pattern = {
            let base = ctx.base_path.borrow();
            if std::path::Path::new(&filename).is_absolute() {
                filename.clone()
            } else {
                base.join(&filename)
                    .to_str()
                    .ok_or_else(|| {
                        XacroError::Include(format!("Invalid UTF-8 in glob pattern: {}", filename))
                    })?
                    .to_string()
            }
        };

        // Find matches
        let mut matches: Vec<PathBuf> = glob::glob(&glob_pattern)
            .map_err(|e| {
                XacroError::Include(format!("Invalid glob pattern '{}': {}", filename, e))
            })?
            .filter_map(|result| match result {
                Ok(path) => Some(path),
                Err(e) => {
                    log::warn!("Error reading glob match: {}", e);
                    None
                }
            })
            .collect();

        // No matches - warn (unless optional) and continue (Python behavior)
        if matches.is_empty() {
            if !optional {
                log::warn!(
                    "Include tag's filename spec \"{}\" matched no files.",
                    filename
                );
            }
            return Ok(vec![]);
        }

        // Process all matches in sorted order (Python sorts matches)
        matches.sort();
        let mut result = Vec::new();
        for file_path in matches {
            let expanded = process_single_include(file_path, ctx)?;
            result.extend(expanded);
        }

        return Ok(result);
    }

    // Case 2 & 3: Regular file (not a glob pattern)
    // Resolve path relative to base_path
    let file_path = ctx.base_path.borrow().join(&filename);

    // Case 2: optional="true" - check if file exists before processing
    if optional && !file_path.exists() {
        return Ok(vec![]);
    }

    // Case 3: Process the include (will error if file not found)
    process_single_include(file_path, ctx)
}

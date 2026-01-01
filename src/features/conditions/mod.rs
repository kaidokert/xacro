use crate::error::XacroError;
use crate::utils::eval::eval_boolean;
use crate::utils::xml::is_xacro_element;
use std::collections::HashMap;
use xmltree::{Element, XMLNode};

pub struct ConditionProcessor;

impl ConditionProcessor {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self
    }

    /// Process conditionals (xacro:if and xacro:unless) in XML tree
    ///
    /// Takes the properties HashMap from PropertyProcessor to evaluate conditional expressions.
    /// Returns the processed Element with conditionals flattened (replaced by their children or removed).
    ///
    /// CRITICAL: This must be called AFTER PropertyProcessor so that:
    /// 1. Properties are available for expression evaluation
    /// 2. Conditional 'value' attributes are still raw expressions (not substituted)
    pub fn process(
        &self,
        mut xml: Element,
        properties: &HashMap<String, String>,
    ) -> Result<Element, XacroError> {
        // Process the tree, flattening conditionals
        process_element(&mut xml, properties)?;
        Ok(xml)
    }
}

/// Recursively process element, flattening xacro:if/unless
///
/// Conditionals are replaced by their children (if condition is true) or removed entirely.
/// This is "flattening" - the xacro:if element disappears, only its children remain.
///
/// Uses drain() to take ownership of children, avoiding clones for better performance.
fn process_element(
    element: &mut Element,
    properties: &HashMap<String, String>,
) -> Result<(), XacroError> {
    let mut new_children = Vec::new();

    // Use drain() to take ownership and avoid cloning every element
    for child in element.children.drain(..) {
        match child {
            XMLNode::Element(mut elem) => {
                let is_if = is_xacro_element(&elem, "if");
                let is_unless = is_xacro_element(&elem, "unless");

                if is_if || is_unless {
                    // This is a conditional element (xacro:if or xacro:unless)
                    let tag_name = if is_if { "xacro:if" } else { "xacro:unless" };

                    // Get the condition expression
                    let value = elem.attributes.get("value").ok_or_else(|| {
                        XacroError::MissingAttribute {
                            element: tag_name.to_string(),
                            attribute: "value".to_string(),
                        }
                    })?;

                    // Evaluate condition using eval_boolean (with type preservation!)
                    let condition =
                        eval_boolean(value, properties).map_err(|e| XacroError::EvalError {
                            expr: value.clone(),
                            source: e,
                        })?;

                    // Determine whether to include children (if: condition, unless: !condition)
                    let should_include = if is_if { condition } else { !condition };

                    if should_include {
                        // Condition is met: process children first, then include them (flattened)
                        // CRITICAL: Must process children to handle nested conditionals
                        process_element(&mut elem, properties)?;
                        new_children.extend(elem.children);
                    }
                    // Condition not met: skip entirely (add nothing)
                } else {
                    // Regular element: keep and recurse into it
                    process_element(&mut elem, properties)?;
                    new_children.push(XMLNode::Element(elem));
                }
            }
            node => {
                // Text nodes, comments, etc. - keep as-is
                new_children.push(node);
            }
        }
    }

    // Replace children with flattened version
    element.children = new_children;
    Ok(())
}

#[cfg(test)]
mod tests;

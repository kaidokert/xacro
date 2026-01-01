use crate::error::XacroError;
use crate::utils::eval::eval_boolean;
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
fn process_element(
    element: &mut Element,
    properties: &HashMap<String, String>,
) -> Result<(), XacroError> {
    let mut new_children = Vec::new();

    for child in &element.children {
        match child {
            XMLNode::Element(elem) => {
                // Check if this is xacro:if
                if is_xacro_element(elem, "if") {
                    // Get the condition expression
                    let value = elem.attributes.get("value").ok_or_else(|| {
                        XacroError::MissingAttribute {
                            element: "xacro:if".to_string(),
                            attribute: "value".to_string(),
                        }
                    })?;

                    // Evaluate condition using eval_boolean (with type preservation!)
                    if eval_boolean(value, properties).map_err(|e| XacroError::EvalError {
                        expr: value.clone(),
                        source: e,
                    })? {
                        // Condition is TRUE: process children first, then include them (flattened)
                        // CRITICAL: Must process children to handle nested conditionals
                        let mut elem_clone = elem.clone();
                        process_element(&mut elem_clone, properties)?;
                        new_children.extend(elem_clone.children);
                    }
                    // Condition is FALSE: skip entirely (add nothing)
                } else if is_xacro_element(elem, "unless") {
                    // xacro:unless is the inverse of xacro:if
                    let value = elem.attributes.get("value").ok_or_else(|| {
                        XacroError::MissingAttribute {
                            element: "xacro:unless".to_string(),
                            attribute: "value".to_string(),
                        }
                    })?;

                    // Evaluate condition (note the ! for inverse)
                    if !eval_boolean(value, properties).map_err(|e| XacroError::EvalError {
                        expr: value.clone(),
                        source: e,
                    })? {
                        // Condition is FALSE: process children first, then include them (flattened)
                        let mut elem_clone = elem.clone();
                        process_element(&mut elem_clone, properties)?;
                        new_children.extend(elem_clone.children);
                    }
                    // Condition is TRUE: skip entirely (add nothing)
                } else {
                    // Regular element: keep and recurse into it
                    let mut elem_clone = elem.clone();
                    process_element(&mut elem_clone, properties)?;
                    new_children.push(XMLNode::Element(elem_clone));
                }
            }
            _ => {
                // Text nodes, comments, etc. - keep as-is
                new_children.push(child.clone());
            }
        }
    }

    // Replace children with flattened version
    element.children = new_children;
    Ok(())
}

/// Check if element is a xacro element with given name
///
/// Handles namespace prefixes correctly (not string matching on "xacro:if").
/// Checks both:
/// - element.prefix == "xacro"
/// - element.namespace == "http://www.ros.org/wiki/xacro" (when no prefix, default namespace)
fn is_xacro_element(
    element: &Element,
    tag_name: &str,
) -> bool {
    element.name == tag_name
        && (element.prefix.as_deref() == Some("xacro")
            || element.namespace.as_deref() == Some("http://www.ros.org/wiki/xacro"))
}

#[cfg(test)]
mod tests;

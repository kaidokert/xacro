use crate::error::XacroError;
use crate::utils::xml::is_xacro_element;
use log::warn;
use pyisheval::Interpreter;
use std::collections::HashMap;
use xmltree::{
    Element,
    XMLNode::{Element as NodeElement, Text as TextElement},
};

/// Built-in math constants (name, value) that are pre-initialized
/// Users can override these, but will receive a warning
const BUILTIN_CONSTANTS: &[(&str, f64)] = &[
    ("pi", core::f64::consts::PI),
    ("e", core::f64::consts::E),
    ("tau", core::f64::consts::TAU),
    ("M_PI", core::f64::consts::PI), // Legacy alias
    ("inf", f64::INFINITY),
    ("nan", f64::NAN),
];

pub struct PropertyProcessor {
    interpreter: Interpreter,
}

/// Helper: Check if we should skip processing this element's body
/// Macro definition bodies should NOT be processed during property collection/substitution
/// because macro parameters (like ${name}) don't exist until expansion time.
fn should_skip_macro_body(
    element: &Element,
    xacro_ns: &str,
) -> bool {
    is_xacro_element(element, "macro", xacro_ns)
}

impl PropertyProcessor {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            interpreter: Interpreter::new(),
        }
    }

    /// Initialize math constants for property evaluation
    ///
    /// Python xacro exposes all math module symbols for backwards compatibility.
    /// This includes constants like pi, e, tau, and functions like sin, cos, etc.
    /// For now, we add the most commonly used constants. Functions would require
    /// extending pyisheval or the expression evaluator.
    fn init_math_constants(properties: &mut HashMap<String, String>) {
        properties.extend(
            BUILTIN_CONSTANTS
                .iter()
                .map(|(name, value)| (name.to_string(), value.to_string())),
        );
    }

    /// Process properties in XML tree, returning both the processed tree and the properties map
    ///
    /// CRITICAL: Returns (Element, HashMap) so that properties can be passed to subsequent
    /// processors (like ConditionProcessor) that need them for expression evaluation.
    pub fn process(
        &self,
        mut xml: Element,
        xacro_ns: &str,
    ) -> Result<(Element, HashMap<String, String>), XacroError> {
        let mut properties = HashMap::new();

        // Initialize built-in math constants
        Self::init_math_constants(&mut properties);

        self.collect_properties(&xml, &mut properties, xacro_ns)?;
        self.substitute_properties(&mut xml, &properties, xacro_ns)?;
        Self::remove_property_elements(&mut xml, xacro_ns);
        Ok((xml, properties)) // Return both Element and properties!
    }

    pub(crate) fn collect_properties(
        &self,
        element: &Element,
        properties: &mut HashMap<String, String>,
        xacro_ns: &str,
    ) -> Result<(), XacroError> {
        // CRITICAL: Skip macro definition bodies
        // Macro parameters don't exist during definition, only during expansion
        if should_skip_macro_body(element, xacro_ns) {
            return Ok(()); // Don't recurse into macro bodies
        }

        // Check if this is a property element (either <property> or <xacro:property>)
        // Check namespace (not prefix) - robust against xmlns:x="..." aliasing
        let is_property = element.name == "property"
            && (element.namespace.is_none() || element.namespace.as_deref() == Some(xacro_ns));

        if is_property {
            if let (Some(name), Some(value)) = (
                element.attributes.get("name"),
                element.attributes.get("value"),
            ) {
                // Warn if user is overriding a built-in constant
                if BUILTIN_CONSTANTS.iter().any(|(k, _)| *k == name.as_str()) {
                    warn!(
                        "Property '{}' overrides built-in math constant. \
                         This may cause unexpected behavior. \
                         Consider using a different name.",
                        name
                    );
                }

                // Evaluate the value in case it contains expressions referencing other properties
                let evaluated_value = self.substitute_in_text(value, properties)?;
                properties.insert(name.clone(), evaluated_value);
            }
        }

        for child in &element.children {
            if let NodeElement(child_elem) = child {
                self.collect_properties(child_elem, properties, xacro_ns)?;
            }
        }

        Ok(())
    }

    pub(crate) fn substitute_properties(
        &self,
        element: &mut Element,
        properties: &HashMap<String, String>,
        xacro_ns: &str,
    ) -> Result<(), XacroError> {
        // CRITICAL: Skip macro definition bodies
        // Macro parameters (like ${name}, ${size}) don't exist during definition
        // They'll be substituted by MacroProcessor during expansion
        if should_skip_macro_body(element, xacro_ns) {
            return Ok(()); // Don't recurse into macro bodies
        }

        // CRITICAL: Check if this is a conditional element (xacro:if or xacro:unless)
        // We must NOT substitute the 'value' attribute on conditionals because:
        // 1. ConditionProcessor needs the raw expression like "${3*0.1}"
        // 2. If we substitute, it becomes "0.3" string, losing type information
        // 3. This breaks float truthiness in eval_boolean
        let is_conditional = is_xacro_element(element, "if", xacro_ns)
            || is_xacro_element(element, "unless", xacro_ns);

        // Process attributes
        for (key, value) in element.attributes.iter_mut() {
            // Skip 'value' attribute on conditionals - preserve raw expression
            if is_conditional && key == "value" {
                continue; // Keep original expression unchanged
            }
            // Normal substitution for all other attributes
            *value = self.substitute_in_text(value, properties)?;
        }

        // Recurse into children
        for child in &mut element.children {
            if let NodeElement(child_elem) = child {
                self.substitute_properties(child_elem, properties, xacro_ns)?;
            } else if let TextElement(text) = child {
                *text = self.substitute_in_text(text, properties)?;
            }
        }

        Ok(())
    }

    pub(crate) fn substitute_in_text(
        &self,
        text: &str,
        properties: &HashMap<String, String>,
    ) -> Result<String, XacroError> {
        use crate::utils::eval::eval_text_with_interpreter;

        eval_text_with_interpreter(text, properties, &self.interpreter).map_err(|e| {
            XacroError::EvalError {
                // Preserve the specific failing expression from EvalError
                expr: match &e {
                    crate::utils::eval::EvalError::PyishEval { expr, .. } => expr.clone(),
                    crate::utils::eval::EvalError::InvalidBoolean { condition, .. } => {
                        condition.clone()
                    }
                },
                source: e,
            }
        })
    }

    pub(crate) fn remove_property_elements(
        element: &mut Element,
        xacro_ns: &str,
    ) {
        element.children.retain_mut(|child| {
            if let NodeElement(child_elem) = child {
                // Remove property elements (either <property> or <xacro:property>)
                // Check namespace (not prefix) - robust against xmlns:x="..." aliasing
                let is_property = child_elem.name == "property"
                    && (child_elem.namespace.is_none()
                        || child_elem.namespace.as_deref() == Some(xacro_ns));

                if is_property {
                    return false;
                }

                // CRITICAL: Don't recurse into macro bodies
                // Properties inside macros need to stay until macro expansion
                if !should_skip_macro_body(child_elem, xacro_ns) {
                    Self::remove_property_elements(child_elem, xacro_ns);
                }
            }
            true
        });
    }
}

#[cfg(test)]
mod tests;

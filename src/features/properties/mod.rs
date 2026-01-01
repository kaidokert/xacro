use crate::error::XacroError;
use crate::utils::xml::{is_xacro_element, XACRO_NAMESPACE};
use pyisheval::Interpreter;
use std::collections::HashMap;
use xmltree::{
    Element,
    XMLNode::{Element as NodeElement, Text as TextElement},
};

pub struct PropertyProcessor {
    interpreter: Interpreter,
}

impl PropertyProcessor {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            interpreter: Interpreter::new(),
        }
    }

    /// Process properties in XML tree, returning both the processed tree and the properties map
    ///
    /// CRITICAL: Returns (Element, HashMap) so that properties can be passed to subsequent
    /// processors (like ConditionProcessor) that need them for expression evaluation.
    pub fn process(
        &self,
        mut xml: Element,
    ) -> Result<(Element, HashMap<String, String>), XacroError> {
        let mut properties = HashMap::new();
        self.collect_properties(&xml, &mut properties)?;
        self.substitute_properties(&mut xml, &properties)?;
        Self::remove_property_elements(&mut xml);
        Ok((xml, properties)) // Return both Element and properties!
    }

    fn collect_properties(
        &self,
        element: &Element,
        properties: &mut HashMap<String, String>,
    ) -> Result<(), XacroError> {
        // Check if this is a property element (either <property> or <xacro:property>)
        // Check namespace (not prefix) - robust against xmlns:x="..." aliasing
        let is_property = element.name == "property"
            && (element.namespace.is_none()
                || element.namespace.as_deref() == Some(XACRO_NAMESPACE));

        if is_property {
            if let (Some(name), Some(value)) = (
                element.attributes.get("name"),
                element.attributes.get("value"),
            ) {
                // Evaluate the value in case it contains expressions referencing other properties
                let evaluated_value = self.substitute_in_text(value, properties)?;
                properties.insert(name.clone(), evaluated_value);
            }
        }

        for child in &element.children {
            if let NodeElement(child_elem) = child {
                self.collect_properties(child_elem, properties)?;
            }
        }

        Ok(())
    }

    pub(crate) fn substitute_properties(
        &self,
        element: &mut Element,
        properties: &HashMap<String, String>,
    ) -> Result<(), XacroError> {
        // CRITICAL: Check if this is a conditional element (xacro:if or xacro:unless)
        // We must NOT substitute the 'value' attribute on conditionals because:
        // 1. ConditionProcessor needs the raw expression like "${3*0.1}"
        // 2. If we substitute, it becomes "0.3" string, losing type information
        // 3. This breaks float truthiness in eval_boolean
        let is_conditional = is_xacro_element(element, "if") || is_xacro_element(element, "unless");

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
                self.substitute_properties(child_elem, properties)?;
            } else if let TextElement(text) = child {
                *text = self.substitute_in_text(text, properties)?;
            }
        }

        Ok(())
    }

    fn substitute_in_text(
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

    fn remove_property_elements(element: &mut Element) {
        element.children.retain_mut(|child| {
            if let NodeElement(child_elem) = child {
                // Remove property elements (either <property> or <xacro:property>)
                // Check namespace (not prefix) - robust against xmlns:x="..." aliasing
                let is_property = child_elem.name == "property"
                    && (child_elem.namespace.is_none()
                        || child_elem.namespace.as_deref() == Some(XACRO_NAMESPACE));

                if is_property {
                    return false;
                }
                Self::remove_property_elements(child_elem);
            }
            true
        });
    }
}

#[cfg(test)]
mod tests;

use crate::error::XacroError;
use std::collections::HashMap;
use xmltree::{
    Element,
    XMLNode::{Element as NodeElement, Text as TextElement},
};

pub struct PropertyProcessor {}

impl PropertyProcessor {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {}
    }

    pub fn process(
        &self,
        mut xml: Element,
    ) -> Result<Element, XacroError> {
        let mut properties = HashMap::new();
        Self::collect_properties(&xml, &mut properties)?;
        Self::substitute_properties(&mut xml, &properties)?;
        Self::remove_property_elements(&mut xml);
        Ok(xml)
    }

    fn collect_properties(
        element: &Element,
        properties: &mut HashMap<String, String>,
    ) -> Result<(), XacroError> {
        // Check if this is a property element (either <property> or <xacro:property>)
        let is_property = element.name == "property"
            && (element.prefix.is_none() || element.prefix.as_deref() == Some("xacro"));

        if is_property {
            if let (Some(name), Some(value)) = (
                element.attributes.get("name"),
                element.attributes.get("value"),
            ) {
                // Evaluate the value in case it contains expressions referencing other properties
                let evaluated_value = Self::substitute_in_text(value, properties)?;
                properties.insert(name.clone(), evaluated_value);
            }
        }

        for child in &element.children {
            if let NodeElement(child_elem) = child {
                Self::collect_properties(child_elem, properties)?;
            }
        }

        Ok(())
    }

    pub(crate) fn substitute_properties(
        element: &mut Element,
        properties: &HashMap<String, String>,
    ) -> Result<(), XacroError> {
        for value in element.attributes.values_mut() {
            *value = Self::substitute_in_text(value, properties)?;
        }

        for child in &mut element.children {
            if let NodeElement(child_elem) = child {
                Self::substitute_properties(child_elem, properties)?;
            } else if let TextElement(text) = child {
                *text = Self::substitute_in_text(text, properties)?;
            }
        }

        Ok(())
    }

    fn substitute_in_text(
        text: &str,
        properties: &HashMap<String, String>,
    ) -> Result<String, XacroError> {
        use crate::utils::eval::eval_text;

        eval_text(text, properties).map_err(|e| XacroError::EvalError {
            expr: text.to_string(),
            source: e,
        })
    }

    fn remove_property_elements(element: &mut Element) {
        element.children.retain_mut(|child| {
            if let NodeElement(child_elem) = child {
                // Remove property elements (either <property> or <xacro:property>)
                let is_property = child_elem.name == "property"
                    && (child_elem.prefix.is_none()
                        || child_elem.prefix.as_deref() == Some("xacro"));

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

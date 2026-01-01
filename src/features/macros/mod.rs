use crate::{
    error::XacroError,
    features::properties::PropertyProcessor,
    utils::{
        pretty_print_hashmap, pretty_print_xml,
        xml::{is_xacro_element, XACRO_NAMESPACE},
    },
};
use log::debug;
use std::collections::HashMap;
use xmltree::{Element, XMLNode::Element as NodeElement};

#[derive(Debug, Clone)]
struct MacroDefinition {
    params: HashMap<String, Option<String>>,
    content: Element,
}

/// Macro processor with configurable recursion depth limit
///
/// The `MAX_DEPTH` const generic allows users to override the default recursion
/// limit (100) without modifying the crate, useful for deeply nested macro structures.
///
/// # Examples
/// ```
/// // Use default depth limit of 100
/// let processor = MacroProcessor::new();
///
/// // Use custom depth limit of 200
/// let processor = MacroProcessor::<200>::new();
/// ```
pub struct MacroProcessor<const MAX_DEPTH: usize = 100> {}

impl<const MAX_DEPTH: usize> MacroProcessor<MAX_DEPTH> {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {}
    }

    pub fn process(
        &self,
        mut xml: Element,
        global_properties: &HashMap<String, String>,
    ) -> Result<Element, XacroError> {
        let mut macros = HashMap::new();

        debug!("Input XML:\n{}", pretty_print_xml(&xml));

        Self::collect_macros(&xml, &mut macros)?;
        debug!("Collected macros:\n{}", pretty_print_hashmap(&macros));

        Self::expand_macros(&mut xml, &macros, global_properties, 0)?;
        debug!("Expanded XML:\n{}", pretty_print_xml(&xml));

        Self::remove_macro_definitions(&mut xml);
        debug!("Output XML:\n{}", pretty_print_xml(&xml));
        Ok(xml)
    }

    fn collect_macros(
        element: &Element,
        macros: &mut HashMap<String, MacroDefinition>,
    ) -> Result<(), XacroError> {
        if Self::is_macro_definition(element) {
            if let (Some(name), Some(params)) = (
                element.attributes.get("name"),
                element.attributes.get("params"),
            ) {
                let params_map = Self::parse_params(params)?;
                let content = element.clone();
                macros.insert(
                    name.clone(),
                    MacroDefinition {
                        params: params_map,
                        content,
                    },
                );
            }
        }

        for child in &element.children {
            if let NodeElement(child_elem) = child {
                Self::collect_macros(child_elem, macros)?;
            }
        }

        Ok(())
    }

    fn parse_params(params_str: &str) -> Result<HashMap<String, Option<String>>, XacroError> {
        let mut params = HashMap::new();
        for param in params_str.split_whitespace() {
            if param.contains(":=") {
                let parts: Vec<&str> = param.split(":=").collect();
                if parts.len() == 2 {
                    params.insert(parts[0].to_string(), Some(parts[1].to_string()));
                }
            } else {
                params.insert(param.to_string(), None);
            }
        }
        Ok(params)
    }

    fn expand_macros(
        element: &mut Element,
        macros: &HashMap<String, MacroDefinition>,
        global_properties: &HashMap<String, String>,
        depth: usize,
    ) -> Result<(), XacroError> {
        // Check recursion depth to prevent infinite loops with self-referential macros
        if depth > MAX_DEPTH {
            return Err(XacroError::MacroRecursionLimit {
                depth,
                limit: MAX_DEPTH,
            });
        }

        let mut new_children = Vec::new();

        for child in core::mem::take(&mut element.children) {
            match child {
                NodeElement(mut child_elem) => {
                    if Self::is_macro_call(&child_elem) {
                        let macro_name = Self::get_macro_name(&child_elem)?;
                        if let Some(macro_def) = macros.get(&macro_name) {
                            let args = Self::collect_macro_args(&child_elem)?;
                            let mut expanded =
                                Self::expand_macro_content(macro_def, &args, global_properties)?;

                            // CRITICAL FIX: Re-scan the expanded content for nested macros
                            // Increment depth to prevent infinite recursion
                            Self::expand_macros(
                                &mut expanded,
                                macros,
                                global_properties,
                                depth + 1,
                            )?;

                            new_children.extend(expanded.children);
                        } else {
                            return Err(XacroError::UndefinedMacro(macro_name));
                        }
                    } else {
                        Self::expand_macros(&mut child_elem, macros, global_properties, depth)?;
                        new_children.push(NodeElement(child_elem));
                    }
                }
                _ => new_children.push(child),
            }
        }

        element.children = new_children;
        Ok(())
    }

    fn expand_macro_content(
        macro_def: &MacroDefinition,
        args: &HashMap<String, String>,
        global_properties: &HashMap<String, String>,
    ) -> Result<Element, XacroError> {
        let mut content = macro_def.content.clone();

        for param_name in args.keys() {
            if !macro_def.params.contains_key(param_name) {
                return Err(XacroError::MissingParameter {
                    macro_name: content.name.clone(),
                    param: param_name.clone(),
                });
            }
        }

        // CRITICAL: Build merged scope (global properties + macro parameters)
        // Use with_capacity to avoid reallocation during extend
        let mut substitutions =
            HashMap::with_capacity(global_properties.len() + macro_def.params.len());
        substitutions.extend(
            global_properties
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );

        // Add macro parameters, which override globals if there's a conflict
        for (param_name, default_value) in &macro_def.params {
            let value = args
                .get(param_name)
                .cloned()
                .or_else(|| default_value.clone())
                .ok_or_else(|| XacroError::MissingParameter {
                    macro_name: content.name.clone(),
                    param: param_name.clone(),
                })?;
            substitutions.insert(param_name.clone(), value);
        }

        // Create a temporary PropertyProcessor for substitution with merged scope
        let property_processor = PropertyProcessor::new();
        property_processor.substitute_properties(&mut content, &substitutions)?;

        Ok(content)
    }

    fn remove_macro_definitions(element: &mut Element) {
        element.children.retain_mut(|child| {
            if let NodeElement(child_elem) = child {
                if Self::is_macro_definition(child_elem) {
                    debug!("Removing macro definition: {:?}", child_elem);
                    return false;
                }
                Self::remove_macro_definitions(child_elem);
            }
            true
        });
    }

    fn is_macro_call(element: &Element) -> bool {
        // List of xacro tags that are NOT macro calls
        const NON_CALL_TAGS: &[&str] = &["macro", "property", "if", "unless", "insert_block"];

        // Check namespace (not prefix) - robust against xmlns:x="..." aliasing
        element.namespace.as_deref() == Some(XACRO_NAMESPACE)
            && !NON_CALL_TAGS.contains(&element.name.as_str())
    }

    fn is_macro_definition(element: &Element) -> bool {
        is_xacro_element(element, "macro")
    }

    fn get_macro_name(element: &Element) -> Result<String, XacroError> {
        Ok(element.name.clone())
    }

    fn collect_macro_args(element: &Element) -> Result<HashMap<String, String>, XacroError> {
        Ok(element.attributes.clone())
    }
}

#[cfg(test)]
mod tests;

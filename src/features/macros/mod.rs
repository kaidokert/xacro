use crate::{
    error::XacroError,
    features::properties::PropertyProcessor,
    utils::{
        pretty_print_hashmap, pretty_print_xml,
        xml::{is_xacro_element, XACRO_NAMESPACE},
    },
};
use log::debug;
use std::collections::{HashMap, HashSet};
use xmltree::{Element, XMLNode::Element as NodeElement};

// Type aliases to simplify complex return types
type ParamsMap = HashMap<String, Option<String>>;
type ParamOrder = Vec<String>;
type BlockParamsSet = HashSet<String>;
type ParsedParams = (ParamsMap, ParamOrder, BlockParamsSet);

type MacroArgs = HashMap<String, String>;
type MacroBlocks = HashMap<String, Element>;
type CollectedArgs = (MacroArgs, MacroBlocks);

#[derive(Debug, Clone)]
struct MacroDefinition {
    params: ParamsMap,            // Regular params with optional defaults
    param_order: ParamOrder,      // Parameter declaration order (critical for block params!)
    block_params: BlockParamsSet, // Block params (names without * prefix)
    content: Element,
}

/// Macro processor with configurable recursion depth limit
///
/// The `MAX_DEPTH` const generic allows users to override the default recursion
/// limit (100) without modifying the crate, useful for deeply nested macro structures.
///
/// # Examples
/// ```
/// use xacro::features::macros::MacroProcessor;
///
/// // Use default depth limit of 100
/// let processor: MacroProcessor = MacroProcessor::new();
///
/// // Use custom depth limit of 200
/// let processor: MacroProcessor<200> = MacroProcessor::<200>::new();
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
                let (params_map, param_order, block_params_set) = Self::parse_params(params)?;
                let content = element.clone();
                macros.insert(
                    name.clone(),
                    MacroDefinition {
                        params: params_map,
                        param_order,
                        block_params: block_params_set,
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

    fn parse_params(params_str: &str) -> Result<ParsedParams, XacroError> {
        let mut params = HashMap::new();
        let mut param_order = Vec::new();
        let mut block_params = HashSet::new();

        for token in params_str.split_whitespace() {
            if let Some(stripped) = token.strip_prefix('*') {
                // Block parameter (e.g., *origin)
                // Block parameters CANNOT have defaults
                if token.contains(":=") {
                    return Err(XacroError::BlockParameterWithDefault {
                        param: token.to_string(),
                    });
                }

                // Store the stripped parameter name
                let param_name = stripped.to_string();
                params.insert(param_name.clone(), None);
                param_order.push(param_name.clone());
                block_params.insert(param_name);
            } else if token.contains(":=") {
                // Regular parameter with default value
                let parts: Vec<&str> = token.splitn(2, ":=").collect();
                params.insert(parts[0].to_string(), Some(parts[1].to_string()));
                param_order.push(parts[0].to_string());
            } else {
                // Regular parameter without default
                params.insert(token.to_string(), None);
                param_order.push(token.to_string());
            }
        }

        Ok((params, param_order, block_params))
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
                            let (args, blocks) = Self::collect_macro_args(&child_elem, macro_def)?;
                            let mut expanded = Self::expand_macro_content(
                                macro_def,
                                &args,
                                &blocks,
                                global_properties,
                            )?;

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
        blocks: &HashMap<String, Element>,
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
            // Skip block parameters (they're not string substitutions)
            if macro_def.block_params.contains(param_name) {
                continue;
            }

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

        // Process insert_block elements, replacing them with the block content
        Self::process_insert_blocks(&mut content, blocks, &substitutions)?;

        Ok(content)
    }

    fn process_insert_blocks(
        element: &mut Element,
        blocks: &HashMap<String, Element>,
        substitutions: &HashMap<String, String>,
    ) -> Result<(), XacroError> {
        let mut new_children = Vec::new();

        for child in core::mem::take(&mut element.children) {
            match child {
                NodeElement(elem) => {
                    if is_xacro_element(&elem, "insert_block") {
                        // This is an insert_block element - replace with block content
                        let block_name = elem.attributes.get("name").ok_or_else(|| {
                            XacroError::MissingAttribute {
                                element: "xacro:insert_block".to_string(),
                                attribute: "name".to_string(),
                            }
                        })?;

                        // Look up the block
                        let block =
                            blocks
                                .get(block_name)
                                .ok_or_else(|| XacroError::UndefinedBlock {
                                    name: block_name.clone(),
                                })?;

                        // Clone the block (allows multiple inserts of same block)
                        let mut block_copy = block.clone();

                        // Recursively process the block:
                        // 1. Apply property substitutions (block can contain ${...} expressions)
                        let property_processor = PropertyProcessor::new();
                        property_processor.substitute_properties(&mut block_copy, substitutions)?;

                        // 2. Recursively process any nested insert_block calls
                        Self::process_insert_blocks(&mut block_copy, blocks, substitutions)?;

                        // Insert the block element itself (not just its children)
                        new_children.push(NodeElement(block_copy));
                    } else {
                        // Regular element: recurse into it
                        let mut elem_copy = elem;
                        Self::process_insert_blocks(&mut elem_copy, blocks, substitutions)?;
                        new_children.push(NodeElement(elem_copy));
                    }
                }
                other => new_children.push(other),
            }
        }

        element.children = new_children;
        Ok(())
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

    fn collect_macro_args(
        element: &Element,
        macro_def: &MacroDefinition,
    ) -> Result<CollectedArgs, XacroError> {
        let mut param_values = HashMap::new();
        let mut block_values = HashMap::new();

        // Extract regular parameters from attributes
        for (name, value) in &element.attributes {
            if !macro_def.block_params.contains(name) {
                param_values.insert(name.clone(), value.clone());
            }
        }

        // Extract block parameters from child elements IN ORDER
        // Only count Element children, not text/comment nodes
        let children: Vec<Element> = element
            .children
            .iter()
            .filter_map(|c| {
                if let xmltree::XMLNode::Element(e) = c {
                    Some(e.clone())
                } else {
                    None
                }
            })
            .collect();

        let mut child_idx = 0;

        // Iterate through params in order they were declared
        // Block params consume child elements sequentially
        for param_name in &macro_def.param_order {
            if macro_def.block_params.contains(param_name) {
                if child_idx >= children.len() {
                    return Err(XacroError::MissingBlockParameter {
                        macro_name: macro_def.content.name.clone(),
                        param: param_name.clone(),
                    });
                }
                block_values.insert(param_name.clone(), children[child_idx].clone());
                child_idx += 1;
            }
        }

        // Error if extra children provided
        if child_idx < children.len() {
            return Err(XacroError::UnusedBlock {
                macro_name: macro_def.content.name.clone(),
                extra_count: children.len() - child_idx,
            });
        }

        Ok((param_values, block_values))
    }
}

#[cfg(test)]
mod tests;

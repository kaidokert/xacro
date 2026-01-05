use crate::error::XacroError;
use std::collections::{HashMap, HashSet};
pub use xmltree::Element;

// Type aliases to simplify complex return types
pub type ParamsMap = HashMap<String, Option<String>>;
pub type ParamOrder = Vec<String>;
pub type BlockParamsSet = HashSet<String>;
pub type ParsedParams = (ParamsMap, ParamOrder, BlockParamsSet);

pub type MacroArgs = HashMap<String, String>;
pub type MacroBlocks = HashMap<String, Element>;
pub type CollectedArgs = (MacroArgs, MacroBlocks);

#[derive(Debug, Clone)]
pub struct MacroDefinition {
    pub name: String,            // Macro name from 'name' attribute (for error messages)
    pub params: ParamsMap,       // Regular params with optional defaults
    pub param_order: ParamOrder, // Parameter declaration order (critical for block params!)
    pub block_params: BlockParamsSet, // Block params (names without * prefix)
    pub content: Element,
}

/// Utility functions for parsing and validating macro definitions
pub struct MacroProcessor;

impl MacroProcessor {
    /// Split a parameter string on whitespace, respecting quoted sections.
    ///
    /// Examples:
    /// - `"a b c"` → `["a", "b", "c"]`
    /// - `"a:='x y' b:=1"` → `["a:='x y'", "b:=1"]`
    /// - `"pos:='0 0 0' *block"` → `["pos:='0 0 0'", "*block"]`
    fn split_params_respecting_quotes(params_str: &str) -> Vec<String> {
        let mut tokens = Vec::new();
        let mut current_token = String::new();
        let mut in_quotes = false;
        let mut quote_char = ' ';

        for ch in params_str.chars() {
            if in_quotes {
                current_token.push(ch);
                if ch == quote_char {
                    in_quotes = false;
                }
            } else if ch == '\'' || ch == '"' {
                // Start of quoted section
                in_quotes = true;
                quote_char = ch;
                current_token.push(ch);
            } else if ch.is_whitespace() {
                // End of token (if not empty)
                if !current_token.is_empty() {
                    tokens.push(current_token.clone());
                    current_token.clear();
                }
            } else {
                // Regular character
                current_token.push(ch);
            }
        }

        // Don't forget the last token
        if !current_token.is_empty() {
            tokens.push(current_token);
        }

        tokens
    }

    pub fn parse_params(params_str: &str) -> Result<ParsedParams, XacroError> {
        let mut params = HashMap::new();
        let mut param_order = Vec::new();
        let mut block_params = HashSet::new();

        for token in Self::split_params_respecting_quotes(params_str) {
            // Parse token to determine parameter type and components
            let (param_name_str, is_block, default_value_str) =
                if let Some(stripped) = token.strip_prefix('*') {
                    // Block parameter (e.g., *origin)
                    // Block parameters CANNOT have defaults
                    if token.contains(":=") {
                        return Err(XacroError::BlockParameterWithDefault {
                            param: token.clone(),
                        });
                    }
                    (stripped.to_string(), true, None)
                } else if let Some((name, value)) = token.split_once(":=") {
                    // Regular parameter with default value
                    // Strip surrounding quotes from default value if present
                    let unquoted_value = if (value.starts_with('\'') && value.ends_with('\''))
                        || (value.starts_with('"') && value.ends_with('"'))
                    {
                        &value[1..value.len() - 1]
                    } else {
                        value
                    };
                    (name.to_string(), false, Some(unquoted_value.to_string()))
                } else {
                    // Regular parameter without default
                    (token.clone(), false, None)
                };

            // Validate parameter name is not empty
            if param_name_str.is_empty() {
                return Err(XacroError::InvalidParameterName { param: token });
            }

            let param_name = param_name_str;

            // Detect duplicate declarations
            if params.contains_key(&param_name) {
                return Err(XacroError::DuplicateParamDeclaration { param: param_name });
            }

            // Insert into appropriate data structures
            param_order.push(param_name.clone());
            if is_block {
                block_params.insert(param_name.clone());
                params.insert(param_name, None);
            } else {
                params.insert(param_name, default_value_str);
            }
        }

        Ok((params, param_order, block_params))
    }

    pub fn collect_macro_args(
        element: &Element,
        macro_def: &MacroDefinition,
    ) -> Result<CollectedArgs, XacroError> {
        let mut param_values = HashMap::new();
        let mut block_values = HashMap::new();

        // Extract regular parameters from attributes
        for (name, value) in &element.attributes {
            if macro_def.block_params.contains(name) {
                // Block parameters cannot be specified as attributes
                return Err(XacroError::BlockParameterAttributeCollision {
                    param: name.clone(),
                });
            }
            param_values.insert(name.clone(), value.clone());
        }

        // Extract block parameters from child elements IN ORDER
        // Use iterator to avoid double-cloning (Vec allocation + insertion)
        let mut children_iter = element
            .children
            .iter()
            .filter_map(xmltree::XMLNode::as_element);

        // Iterate through params in order they were declared
        // Block params consume child elements sequentially from the iterator
        for param_name in &macro_def.param_order {
            if macro_def.block_params.contains(param_name) {
                let child_element =
                    children_iter
                        .next()
                        .ok_or_else(|| XacroError::MissingBlockParameter {
                            macro_name: macro_def.name.clone(),
                            param: param_name.clone(),
                        })?;
                block_values.insert(param_name.clone(), child_element.clone());
            }
        }

        // Error if extra children provided
        if children_iter.next().is_some() {
            let extra_count = 1 + children_iter.count();
            return Err(XacroError::UnusedBlock {
                macro_name: macro_def.name.clone(),
                extra_count,
            });
        }

        Ok((param_values, block_values))
    }
}

#[cfg(test)]
mod tests;

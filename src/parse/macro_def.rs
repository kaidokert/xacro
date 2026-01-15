use crate::error::XacroError;
use std::collections::{HashMap, HashSet};
pub use xmltree::Element;

// Type aliases to simplify complex return types
pub type ParamsMap = HashMap<String, Option<String>>;
pub type ParamOrder = Vec<String>;
pub type BlockParamsSet = HashSet<String>;
pub type ParsedParams = (ParamsMap, ParamOrder, BlockParamsSet, BlockParamsSet);

pub type MacroArgs = HashMap<String, String>;
pub type MacroBlocks = HashMap<String, Element>;
pub type CollectedArgs = (MacroArgs, MacroBlocks);

#[derive(Debug, Clone)]
pub struct MacroDefinition {
    pub name: String,            // Macro name from 'name' attribute (for error messages)
    pub params: ParamsMap,       // Regular params with optional defaults
    pub param_order: ParamOrder, // Parameter declaration order (critical for block params!)
    pub block_params: BlockParamsSet, // Block params (names without * prefix)
    pub lazy_block_params: BlockParamsSet, // Lazy block params (**param - insert children only)
    pub content: Element,
}

/// Utility functions for parsing and validating macro definitions
pub struct MacroProcessor;

impl MacroProcessor {
    /// Split a parameter string on whitespace, respecting quoted sections.
    ///
    /// Returns an error if quotes are unbalanced (unclosed quote).
    ///
    /// Examples:
    /// - `"a b c"` → `["a", "b", "c"]`
    /// - `"a:='x y' b:=1"` → `["a:='x y'", "b:=1"]`
    /// - `"pos:='0 0 0' *block"` → `["pos:='0 0 0'", "*block"]`
    /// - `"rpy:='0 0 0"` → Error (unclosed quote)
    fn split_params_respecting_quotes(params_str: &str) -> Result<Vec<String>, XacroError> {
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
                    // Use mem::take to avoid cloning
                    tokens.push(core::mem::take(&mut current_token));
                }
            } else {
                // Regular character
                current_token.push(ch);
            }
        }

        // Check for unbalanced quotes before returning
        if in_quotes {
            return Err(XacroError::UnbalancedQuote {
                quote_char,
                params_str: params_str.to_string(),
            });
        }

        // Don't forget the last token
        if !current_token.is_empty() {
            tokens.push(current_token);
        }

        Ok(tokens)
    }

    /// Parse macro parameters (strict mode - default)
    pub fn parse_params(params_str: &str) -> Result<ParsedParams, XacroError> {
        Self::parse_params_impl(params_str, false)
    }

    /// Parse macro parameters (compatibility mode - accept duplicates)
    pub fn parse_params_compat(params_str: &str) -> Result<ParsedParams, XacroError> {
        Self::parse_params_impl(params_str, true)
    }

    /// Internal implementation for parameter parsing
    fn parse_params_impl(
        params_str: &str,
        compat_mode: bool,
    ) -> Result<ParsedParams, XacroError> {
        let mut params = HashMap::new();
        let mut param_order = Vec::new();
        let mut block_params = HashSet::new();
        let mut lazy_block_params = HashSet::new();

        for token in Self::split_params_respecting_quotes(params_str)? {
            // Parse token to determine parameter type and components
            let (param_name_str, is_block, is_lazy, default_value_str) = if token.starts_with('*') {
                // Block parameter (**param or *param)
                // Block parameters CANNOT have defaults
                if token.contains(":=") || token.contains('=') {
                    return Err(XacroError::BlockParameterWithDefault {
                        param: token.clone(),
                    });
                }

                // Check for lazy block (**param) vs regular block (*param)
                let (stripped, is_lazy) = if let Some(s) = token.strip_prefix("**") {
                    // Lazy block parameter (**param - inserts children only)
                    (s, true)
                } else if let Some(s) = token.strip_prefix('*') {
                    // Regular block parameter (*param - inserts element itself)
                    (s, false)
                } else {
                    unreachable!("starts_with('*') check guarantees this branch is unreachable");
                };

                // Validate no extra asterisks (reject ***param, ****param, etc.)
                if stripped.starts_with('*') {
                    return Err(XacroError::InvalidParameterName {
                        param: token.clone(),
                    });
                }

                (stripped.to_string(), true, is_lazy, None)
            } else if let Some((name, value)) =
                token.split_once(":=").or_else(|| token.split_once('='))
            {
                // Regular parameter with default value (supports := or =)
                // Python xacro supports both syntaxes:
                //   params="width:=5"  (preferred)
                //   params="width=5"   (also valid)
                // Strip surrounding quotes from default value if present
                // Using strip_prefix/strip_suffix is safe and handles edge cases like single-char strings
                let unquoted_value = if let Some(s) =
                    value.strip_prefix('\'').and_then(|s| s.strip_suffix('\''))
                {
                    s
                } else if let Some(s) = value.strip_prefix('"').and_then(|s| s.strip_suffix('"')) {
                    s
                } else {
                    value
                };
                (
                    name.to_string(),
                    false,
                    false,
                    Some(unquoted_value.to_string()),
                )
            } else {
                // Regular parameter without default
                (token.clone(), false, false, None)
            };

            // Validate parameter name is not empty
            if param_name_str.is_empty() {
                return Err(XacroError::InvalidParameterName { param: token });
            }

            let param_name = param_name_str;

            // Detect duplicate declarations (strict mode only)
            if params.contains_key(&param_name) && !compat_mode {
                return Err(XacroError::DuplicateParamDeclaration { param: param_name });
            }
            // In compat mode, silently overwrite (last declaration wins)

            // Insert into appropriate data structures
            // Only add to param_order if not already present (handles compat mode duplicates)
            if !params.contains_key(&param_name) {
                param_order.push(param_name.clone());
            }
            if is_block {
                block_params.insert(param_name.clone());
                if is_lazy {
                    lazy_block_params.insert(param_name.clone());
                } else {
                    // Regular block, remove from lazy set if previously there
                    lazy_block_params.remove(&param_name);
                }
                params.insert(param_name, None);
            } else {
                // In compat mode, if changing from block to non-block, remove from block_params and lazy_block_params
                if compat_mode {
                    block_params.remove(&param_name);
                    lazy_block_params.remove(&param_name);
                }
                params.insert(param_name, default_value_str);
            }
        }

        Ok((params, param_order, block_params, lazy_block_params))
    }

    pub fn collect_macro_args(
        element: &Element,
        macro_def: &MacroDefinition,
    ) -> Result<CollectedArgs, XacroError> {
        let mut param_values = HashMap::new();
        let mut block_values = HashMap::new();

        // Extract regular parameters from attributes
        // Reject namespaced attributes - macro parameters must be local names only
        for (name, value) in &element.attributes {
            let local_name = &name.local_name;

            // Reject namespaced attributes on macro calls (Python xacro behavior)
            if let Some(prefix) = &name.prefix {
                return Err(XacroError::InvalidMacroParameter {
                    param: format!("{}:{}", prefix, name.local_name),
                    reason: "Macro parameters cannot have namespace prefixes".to_string(),
                });
            }

            if macro_def.block_params.contains(local_name) {
                // Block parameters cannot be specified as attributes
                return Err(XacroError::BlockParameterAttributeCollision {
                    param: local_name.clone(),
                });
            }
            param_values.insert(local_name.clone(), value.clone());
        }

        // Extract block parameters from child elements IN ORDER
        // Use iterator to avoid double-cloning (Vec allocation + insertion)
        let mut children_iter = element
            .children
            .iter()
            .filter_map(xmltree::XMLNode::as_element);

        log::debug!(
            "collect_macro_args: macro '{}' has {} block params, {} lazy",
            macro_def.name,
            macro_def.block_params.len(),
            macro_def.lazy_block_params.len()
        );
        log::debug!(
            "collect_macro_args: macro call has {} child elements",
            element
                .children
                .iter()
                .filter_map(|n| n.as_element())
                .count()
        );

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
                log::debug!(
                    "collect_macro_args: captured block param '{}' <- element '<{}>...'",
                    param_name,
                    child_element.name
                );
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

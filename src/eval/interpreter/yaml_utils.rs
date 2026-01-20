use super::parsing::{escape_python_string, find_matching_paren};
use pyisheval::{Interpreter, Value};
use regex::Regex;
use std::collections::HashMap;
use std::sync::OnceLock;

#[cfg(feature = "yaml")]
use saphyr::{LoadableYamlNode, Scalar, Yaml};

#[cfg(feature = "yaml")]
/// Convert a serde_yaml::Value to a Python literal string
///
/// Recursively converts YAML structures to Python dict/list literals that can be
/// evaluated by pyisheval. This matches Python xacro's YamlDictWrapper/YamlListWrapper
/// behavior but generates static literals instead of wrapper objects.
///
/// # Examples
/// ```text
/// {key: value} → "{'key': 'value'}"
/// [1, 2, 3] → "[1, 2, 3]"
/// true → "True"
/// null → "None"
/// ```
pub(super) fn yaml_to_python_literal(
    value: Yaml,
    path: &str,
    yaml_tag_handler_registry: Option<&crate::eval::yaml_tag_handler::YamlTagHandlerRegistry>,
) -> Result<String, crate::error::XacroError> {
    match value {
        Yaml::Value(scalar) => match scalar {
            Scalar::Null => Ok("None".to_string()),
            Scalar::Boolean(b) => {
                if b {
                    Ok("True".to_string())
                } else {
                    Ok("False".to_string())
                }
            }
            Scalar::Integer(i) => Ok(i.to_string()),
            Scalar::FloatingPoint(f) => Ok(f.to_string()),
            Scalar::String(s) => {
                let escaped = escape_python_string(&s);
                Ok(format!("'{}'", escaped))
            }
        },
        Yaml::Sequence(seq) => {
            let elements: Result<Vec<String>, _> = seq
                .into_iter()
                .map(|v| yaml_to_python_literal(v, path, yaml_tag_handler_registry))
                .collect();
            Ok(format!("[{}]", elements?.join(", ")))
        }
        Yaml::Mapping(map) => {
            let entries: Result<Vec<String>, crate::error::XacroError> = map
                .into_iter()
                .map(|(k, v)| -> Result<String, crate::error::XacroError> {
                    let key_str = match &k {
                        Yaml::Value(Scalar::String(s)) => {
                            let escaped = escape_python_string(s);
                            format!("'{}'", escaped)
                        }
                        _ => yaml_to_python_literal(
                            k,
                            path,
                            #[cfg(feature = "yaml")]
                            yaml_tag_handler_registry,
                        )?, // Non-string keys
                    };
                    let value_str = yaml_to_python_literal(
                        v,
                        path,
                        #[cfg(feature = "yaml")]
                        yaml_tag_handler_registry,
                    )?;
                    Ok(format!("{}: {}", key_str, value_str))
                })
                .collect();
            Ok(format!("{{{}}}", entries?.join(", ")))
        }
        Yaml::Tagged(tag, inner) => {
            // Python xacro only supports tags on scalar values (not sequences/mappings)
            // Check if inner is a complex type and handle accordingly
            if matches!(&*inner, Yaml::Sequence(_) | Yaml::Mapping(_)) {
                log::warn!(
                    "YAML tag '!{}' applied to non-scalar value (sequence/mapping) at '{}' - Python xacro does not support this. Value will be used without conversion.",
                    tag.suffix, path
                );
                // Convert to literal without applying tag handler
                return yaml_to_python_literal(
                    *inner,
                    path,
                    #[cfg(feature = "yaml")]
                    yaml_tag_handler_registry,
                );
            }

            // Check if inner is a string scalar for fallback handling
            let is_string_scalar = matches!(&*inner, Yaml::Value(Scalar::String(_)));

            // Extract raw string value for handler (only scalars reach here)
            let raw_value = match &*inner {
                Yaml::Value(Scalar::Integer(i)) => i.to_string(),
                Yaml::Value(Scalar::FloatingPoint(f)) => f.to_string(),
                Yaml::Value(Scalar::String(s)) => s.to_string(),
                Yaml::Value(Scalar::Null) => "None".to_string(),
                Yaml::Value(Scalar::Boolean(b)) => {
                    if *b {
                        "True".to_string()
                    } else {
                        "False".to_string()
                    }
                }
                // Fallback for edge cases like Representation, Alias, BadValue
                _ => yaml_to_python_literal(
                    *inner,
                    path,
                    #[cfg(feature = "yaml")]
                    yaml_tag_handler_registry,
                )?,
            };

            // Try registered handlers (only for scalar values)
            #[cfg(feature = "yaml")]
            if let Some(registry) = yaml_tag_handler_registry {
                if let Some(result) = registry.handle_tag(&tag.suffix, &raw_value) {
                    return Ok(result);
                }
            }

            // No handler matched - warn and pass through value as-is
            log::warn!(
                "Unknown YAML tag '!{}' - value will be used without conversion",
                tag.suffix
            );

            // If it's a string scalar, quote it properly for Python evaluation
            if is_string_scalar {
                let escaped = escape_python_string(&raw_value);
                Ok(format!("'{}'", escaped))
            } else {
                // Numeric values pass through unquoted
                Ok(raw_value)
            }
        }
        Yaml::Representation(repr, _style, _tag) => {
            // Handle unresolved representations - parse as scalar value
            yaml_to_python_literal(
                Yaml::value_from_str(&repr),
                path,
                #[cfg(feature = "yaml")]
                yaml_tag_handler_registry,
            )
        }
        Yaml::Alias(_) => Err(crate::error::XacroError::YamlParseError {
            path: path.to_string(),
            message: "YAML aliases are not supported".to_string(),
        }),
        Yaml::BadValue => Err(crate::error::XacroError::YamlParseError {
            path: path.to_string(),
            message: "YAML contains an invalid value".to_string(),
        }),
    }
}

#[cfg(feature = "yaml")]
/// Load a YAML file and convert it to a Python dict literal string
///
/// # Arguments
/// * `path` - Absolute or relative file path
///
/// # Returns
/// Python dict/list literal string that can be evaluated by pyisheval
///
/// # Errors
/// Returns error if file cannot be read or YAML is invalid
pub(super) fn load_yaml_file(
    path: &str,
    yaml_tag_handler_registry: Option<&crate::eval::yaml_tag_handler::YamlTagHandlerRegistry>,
) -> Result<String, crate::error::XacroError> {
    // Read file contents
    let contents = std::fs::read_to_string(path).map_err(|source| {
        crate::error::XacroError::YamlLoadError {
            path: path.to_string(),
            source,
        }
    })?;

    // Parse YAML (returns Vec<Yaml> for multiple documents, take first)
    let docs =
        Yaml::load_from_str(&contents).map_err(|e| crate::error::XacroError::YamlParseError {
            path: path.to_string(),
            message: e.to_string(),
        })?;

    // Get first document (most common case)
    let yaml_value =
        docs.into_iter()
            .next()
            .ok_or_else(|| crate::error::XacroError::YamlParseError {
                path: path.to_string(),
                message: "YAML file contains no documents".to_string(),
            })?;

    // Convert to Python literal
    yaml_to_python_literal(
        yaml_value,
        path,
        #[cfg(feature = "yaml")]
        yaml_tag_handler_registry,
    )
}

#[cfg(feature = "yaml")]
/// Regex pattern for matching load_yaml() calls
static LOAD_YAML_REGEX: OnceLock<Regex> = OnceLock::new();

#[cfg(feature = "yaml")]
/// Get the load_yaml regex, initializing it on first access
///
/// Matches load_yaml function name up to the opening parenthesis.
/// Uses word boundary to avoid matching inside identifiers.
/// The closing parenthesis is found using find_matching_paren() to handle
/// nested expressions like load_yaml(get_path('config.yaml')).
pub(super) fn get_load_yaml_regex() -> &'static Regex {
    LOAD_YAML_REGEX.get_or_init(|| {
        // Pattern: \b(?:xacro\.)?load_yaml\s*\(
        // Matches: xacro.load_yaml( or load_yaml(
        // Uses word boundary \b to avoid matching inside larger identifiers
        Regex::new(r"\b(?:xacro\.)?load_yaml\s*\(").expect("load_yaml regex should be valid")
    })
}

#[cfg(feature = "yaml")]
/// Preprocess an expression to evaluate load_yaml() calls
///
/// Finds load_yaml() calls (with or without xacro. prefix), loads the YAML files,
/// and replaces the calls with Python dict literals.
///
/// # Limitations
/// - Does not distinguish function calls inside string literals
/// - File paths are relative to current working directory (not current file)
///
/// # Arguments
/// * `expr` - Expression that may contain load_yaml() calls
/// * `interp` - Interpreter for evaluating filename variables
/// * `context` - Context for evaluating filename variables
///
/// # Returns
/// Expression with load_yaml() calls replaced by dict literals
pub(super) fn preprocess_load_yaml(
    expr: &str,
    interp: &mut Interpreter,
    context: &HashMap<String, Value>,
    yaml_tag_handler_registry: Option<&crate::eval::yaml_tag_handler::YamlTagHandlerRegistry>,
) -> Result<String, super::EvalError> {
    let regex = get_load_yaml_regex();
    let mut result = expr.to_string();
    let mut iteration = 0;
    const MAX_ITERATIONS: usize = 100;

    loop {
        iteration += 1;
        if iteration > MAX_ITERATIONS {
            return Err(super::EvalError::PyishEval {
                expr: expr.to_string(),
                source: pyisheval::EvalError::ParseError(
                    "Too many nested load_yaml() calls (possible infinite loop)".to_string(),
                ),
            });
        }

        // Find all load_yaml() matches
        let captures: Vec<_> = regex.captures_iter(&result).collect();
        if captures.is_empty() {
            break;
        }

        let mut made_replacement = false;
        // Process from right to left (innermost first)
        for caps in captures.iter().rev() {
            // Get the match (function name up to opening paren)
            let whole_match = match caps.get(0) {
                Some(m) => m,
                None => continue,
            };
            let paren_pos = whole_match.end() - 1; // Position of '(' after optional whitespace

            // Find matching closing parenthesis
            let close_pos = match find_matching_paren(&result, paren_pos) {
                Some(pos) => pos,
                None => continue, // Skip if no matching paren
            };

            // Extract the argument between the parentheses
            let filename_arg = &result[paren_pos + 1..close_pos];

            // Evaluate filename argument (could be a variable or string literal)
            let filename = if filename_arg.starts_with('\'') || filename_arg.starts_with('"') {
                // String literal - strip quotes
                filename_arg
                    .trim_matches(|c| c == '\'' || c == '"')
                    .to_string()
            } else {
                // Variable or expression - evaluate it
                match interp.eval_with_context(filename_arg, context) {
                    Ok(Value::StringLit(s)) => s,
                    Ok(other) => {
                        return Err(super::EvalError::PyishEval {
                            expr: format!("load_yaml({})", filename_arg),
                            source: pyisheval::EvalError::ParseError(format!(
                                "load_yaml() filename must be a string, got: {:?}",
                                other
                            )),
                        });
                    }
                    Err(e) => {
                        return Err(super::EvalError::PyishEval {
                            expr: format!("load_yaml({})", filename_arg),
                            source: e,
                        });
                    }
                }
            };

            // Load YAML and convert to Python literal
            let python_literal =
                load_yaml_file(&filename, yaml_tag_handler_registry).map_err(|e| {
                    super::EvalError::PyishEval {
                        expr: format!("load_yaml('{}')", filename),
                        source: pyisheval::EvalError::ParseError(e.to_string()),
                    }
                })?;

            // Replace load_yaml(...) with dict literal (from start of match to closing paren)
            result.replace_range(whole_match.start()..=close_pos, &python_literal);
            made_replacement = true;
            break; // Restart loop to rescan
        }

        if !made_replacement {
            break;
        }
    }

    Ok(result)
}

#[cfg(not(feature = "yaml"))]
/// Stub function when yaml feature is disabled
///
/// Checks if load_yaml is used and returns a helpful error message.
/// Uses regex with word boundaries to avoid false positives on similar identifiers.
/// Signature matches the enabled version for consistency.
pub(super) fn preprocess_load_yaml(
    expr: &str,
    _interp: &mut Interpreter,
    _context: &HashMap<String, Value>,
) -> Result<String, super::EvalError> {
    // Use regex with word boundaries to robustly detect load_yaml and xacro.load_yaml
    // This avoids false positives on similar-looking identifiers like "my_load_yaml"
    static LOAD_YAML_REGEX: OnceLock<Regex> = OnceLock::new();
    let regex = LOAD_YAML_REGEX.get_or_init(|| {
        Regex::new(r"\b(?:xacro\.)?load_yaml\b").expect("load_yaml regex should be valid")
    });

    if regex.is_match(expr) {
        return Err(super::EvalError::PyishEval {
            expr: expr.to_string(),
            source: pyisheval::EvalError::ParseError(
                crate::error::XacroError::YamlFeatureDisabled.to_string(),
            ),
        });
    }
    Ok(expr.to_string())
}

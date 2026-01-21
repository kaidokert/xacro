//! Property metadata tracking for Python xacro compatibility
//!
//! This module provides metadata tracking and formatting support that matches
//! Python xacro's int/float distinction and boolean literal formatting.
//!
//! Key features:
//! - Track which properties should format with .0 (floats) vs without (ints)
//! - Track pseudo-boolean properties (True/False literals) for 1.0/0.0 formatting
//! - Metadata propagation through property references

use super::EvalContext;
use crate::eval::interpreter::format_value_python_style;
use crate::eval::interpreter::remove_quotes;

/// Metadata tracked for each property to support Python-like formatting
#[cfg(feature = "compat")]
#[derive(Debug, Clone)]
pub(super) struct PropertyMetadata {
    /// Whether this property should be formatted as float (keep .0 for whole numbers)
    /// True if:
    /// - Property value contains decimal point (e.g., "1.5", "100.0")
    /// - Property comes from division expression (e.g., "${255/255}")
    /// - Property references a float property (e.g., "${float_prop * 2}")
    pub(super) is_float: bool,
    /// Whether this property originated from a boolean literal
    /// True if:
    /// - Property value is "true" or "false" (case-insensitive, e.g., TRUE, False, etc.)
    /// - Property references a pseudo-boolean property
    ///
    /// When true, numeric values 1.0/0.0 should be formatted as "True"/"False"
    pub(super) is_pseudo_boolean: bool,
}

impl<const MAX_SUBSTITUTION_DEPTH: usize> EvalContext<MAX_SUBSTITUTION_DEPTH> {
    /// Generic helper to access a metadata field for a variable
    ///
    /// Checks scoped metadata first (depth:name), then falls back to global metadata.
    /// This avoids duplication between is_var_float and is_var_boolean.
    ///
    /// # Arguments
    /// * `var` - The variable name to look up
    /// * `field_accessor` - Closure to extract the desired field from PropertyMetadata
    ///
    /// # Returns
    /// The field value, or T::default() if not found
    #[cfg(feature = "compat")]
    pub(super) fn get_var_metadata_field<F, T>(
        &self,
        var: &str,
        field_accessor: F,
    ) -> T
    where
        F: Fn(&PropertyMetadata) -> T,
        T: Default,
    {
        let metadata = self.property_metadata.borrow();
        let scope_depth = self.scope_stack.borrow().len();

        // Try scoped metadata first (current scope down to global)
        for depth in (1..=scope_depth).rev() {
            let scoped_key = format!("{}:{}", depth, var);
            if let Some(meta) = metadata.get(&scoped_key) {
                return field_accessor(meta);
            }
        }

        // Fall back to checking global (no scope prefix)
        metadata.get(var).map(field_accessor).unwrap_or_default()
    }

    /// Check if a variable has float metadata (compat feature only)
    ///
    /// Looks up a property by name across all scopes (current to global) to determine
    /// if it should be formatted as a float. Checks scoped metadata first (depth:name),
    /// then falls back to global metadata (no scope prefix).
    ///
    /// # Arguments
    /// * `var` - The variable name to look up
    ///
    /// # Returns
    /// `true` if the variable has float metadata, `false` otherwise
    #[cfg(feature = "compat")]
    pub(super) fn is_var_float(
        &self,
        var: &str,
    ) -> bool {
        self.get_var_metadata_field(var, |m| m.is_float)
    }

    /// Check if a variable is marked as pseudo-boolean (originated from True/False literal)
    ///
    /// Used during formatting to determine if 1.0/0.0 should be formatted as "True"/"False".
    /// Checks scoped metadata first (depth:name), then falls back to global metadata.
    ///
    /// # Arguments
    /// * `var` - The variable name to look up
    ///
    /// # Returns
    /// `true` if the variable has boolean metadata, `false` otherwise
    #[cfg(feature = "compat")]
    pub(super) fn is_var_boolean(
        &self,
        var: &str,
    ) -> bool {
        self.get_var_metadata_field(var, |m| m.is_pseudo_boolean)
    }

    /// Compute float metadata for a property value (compat feature only)
    ///
    /// Determines if a property should be formatted as float (with .0 for whole numbers)
    /// based on Python xacro's int/float distinction heuristics.
    ///
    /// Detection rules:
    /// 1. Value contains decimal point → float
    /// 2. Value is inf/nan → float
    /// 3. Value contains division (/) → float
    /// 4. Value references a float property → float (propagation)
    #[cfg(feature = "compat")]
    pub(super) fn compute_float_metadata(
        &self,
        value: &str,
    ) -> bool {
        if !self.use_python_compat {
            return false;
        }

        let has_decimal = value.contains('.');
        let is_special = value.parse::<f64>().is_ok_and(|n| !n.is_finite());
        let has_division = value.contains('/');

        // Check if expression references any float properties
        let refs_float_prop = if value.contains("${") {
            let refs = self.extract_property_references(value);
            refs.iter().any(|r| self.is_var_float(r))
        } else {
            false
        };

        has_decimal || is_special || has_division || refs_float_prop
    }

    #[cfg(not(feature = "compat"))]
    #[allow(dead_code)]
    pub(super) fn compute_float_metadata(
        &self,
        _value: &str,
    ) -> bool {
        false
    }

    /// Compute boolean metadata for a property value (compat feature only)
    ///
    /// Determines if a property originated from a boolean literal and should
    /// format 1.0/0.0 as "True"/"False" in output.
    ///
    /// Detection rules:
    /// 1. Value is "True"/"False" (Python literals) or "true"/"false" (XML/xacro convention)
    /// 2. Value references a pseudo-boolean property (propagation)
    ///
    /// Limitation: Only works for literal definitions, not boolean expressions
    /// like `${x == y}` which return 1.0/0.0 without type information.
    #[cfg(feature = "compat")]
    pub(super) fn compute_boolean_metadata(
        &self,
        value: &str,
    ) -> bool {
        if !self.use_python_compat {
            return false;
        }

        let trimmed = value.trim();

        // Check for boolean literals (case-insensitive matching)
        // Python xacro's _eval_literal() accepts any casing: true, True, TRUE, etc.
        // All are formatted as "True"/"False" in output
        if trimmed.eq_ignore_ascii_case("true") || trimmed.eq_ignore_ascii_case("false") {
            return true;
        }

        // Check if expression references any boolean properties
        if trimmed.contains("${") {
            let refs = self.extract_property_references(trimmed);
            refs.iter().any(|r| self.is_var_boolean(r))
        } else {
            false
        }
    }

    #[cfg(not(feature = "compat"))]
    #[allow(dead_code)]
    pub(super) fn compute_boolean_metadata(
        &self,
        _value: &str,
    ) -> bool {
        false
    }

    /// Compute property metadata for a value (compat feature only)
    ///
    /// Combines float and boolean metadata computation to avoid duplication.
    /// This is called when adding properties to scope or globally.
    ///
    /// # Arguments
    /// * `value` - The property value string to analyze
    ///
    /// # Returns
    /// PropertyMetadata with both is_float and is_pseudo_boolean computed
    #[cfg(feature = "compat")]
    pub(super) fn compute_property_metadata(
        &self,
        value: &str,
    ) -> PropertyMetadata {
        PropertyMetadata {
            is_float: self.compute_float_metadata(value),
            is_pseudo_boolean: self.compute_boolean_metadata(value),
        }
    }

    /// Check if an expression result should be formatted as float (with .0 for whole numbers)
    ///
    /// Returns true if:
    /// - Expression contains division (always produces float), OR
    /// - Result has fractional part (is not a whole number), OR
    /// - Expression references any float properties (metadata propagation)
    ///
    /// # Arguments
    /// * `expr` - The expression that was evaluated
    /// * `result_value` - The pyisheval Value result
    ///
    /// # Returns
    /// Whether to format the result as float (with .0 for whole numbers)
    #[cfg(feature = "compat")]
    pub(super) fn should_format_as_float(
        &self,
        expr: &str,
        result_value: &pyisheval::Value,
    ) -> bool {
        // Check if expression contains division (always produces float)
        if expr.contains('/') {
            return true;
        }

        // Check if result has fractional part (always float)
        if let pyisheval::Value::Number(n) = result_value {
            if n.fract() != 0.0 {
                return true;
            }
        }

        // Special case: if expression is just a simple variable name (no operators),
        // check its metadata directly
        let trimmed = expr.trim();
        if super::util::is_simple_identifier(trimmed) {
            return self.is_var_float(trimmed);
        }

        // Complex expression: extract variables and check if any are float properties
        // Note: extract_property_references expects ${...} syntax, so wrap the expression
        let vars = self.extract_property_references(&format!("${{{}}}", expr));

        for var in vars {
            if self.is_var_float(&var) {
                return true;
            }
        }

        false
    }

    /// Check if an expression result should be formatted as boolean ("True"/"False")
    ///
    /// Returns true if:
    /// - Result value is exactly 1.0 or 0.0 (boolean numeric values), AND
    /// - Expression is a simple variable reference with boolean metadata
    ///
    /// Note: Complex expressions (like `1 if flag else 0`) are NOT formatted as boolean,
    /// even if they reference boolean properties, because the result type is determined
    /// by the expression itself, not by the properties it references.
    ///
    /// # Arguments
    /// * `expr` - The expression that was evaluated
    /// * `result_value` - The pyisheval Value result
    ///
    /// # Returns
    /// `true` if the result should be formatted as "True"/"False", `false` otherwise
    #[cfg(feature = "compat")]
    pub(super) fn should_format_as_boolean(
        &self,
        expr: &str,
        result_value: &pyisheval::Value,
    ) -> bool {
        // Only boolean values (1.0 or 0.0) can be formatted as boolean
        if !matches!(result_value, pyisheval::Value::Number(n) if *n == 1.0 || *n == 0.0) {
            return false;
        }

        // Only format as boolean for simple variable references
        // Complex expressions determine their own output type
        let trimmed = expr.trim();

        if super::util::is_simple_identifier(trimmed) {
            return self.is_var_boolean(trimmed);
        }

        false
    }

    /// Format an evaluation result with Python-compatible number formatting
    ///
    /// When compat feature is enabled, uses metadata to decide whether to format
    /// numbers as float (with .0) or int. When disabled, uses default formatting.
    /// Always strips quotes from string literals.
    ///
    /// # Arguments
    /// * `value` - The evaluated value to format
    /// * `expr` - The original expression (used for metadata lookup)
    ///
    /// # Returns
    /// Formatted string with quotes stripped if it was a string literal
    #[cfg_attr(not(feature = "compat"), allow(unused_variables))]
    pub(super) fn format_evaluation_result(
        &self,
        value: &pyisheval::Value,
        expr: &str,
    ) -> String {
        // Format the value
        let formatted = {
            #[cfg(feature = "compat")]
            {
                if self.use_python_compat {
                    // Check for boolean formatting first (1.0/0.0 with boolean metadata)
                    if self.should_format_as_boolean(expr, value) {
                        // should_format_as_boolean guarantees value is Number(1.0) or Number(0.0)
                        if let pyisheval::Value::Number(n) = value {
                            if *n == 1.0 {
                                "True".to_string()
                            } else {
                                "False".to_string()
                            }
                        } else {
                            unreachable!("should_format_as_boolean validated this is a Number")
                        }
                    } else {
                        // Not a boolean, use float metadata
                        let force_float = self.should_format_as_float(expr, value);
                        format_value_python_style(value, force_float)
                    }
                } else {
                    // Preserve old behavior: always format with .0 for whole numbers
                    format_value_python_style(value, true)
                }
            }
            #[cfg(not(feature = "compat"))]
            {
                // Preserve old behavior: always format with .0 for whole numbers
                format_value_python_style(value, true)
            }
        };

        // Strip quotes from string literals (always done, not compat-specific)
        remove_quotes(&formatted).to_string()
    }
}

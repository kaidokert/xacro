//! Utility functions for expansion
//!
//! This module provides helper functions used during expansion.

/// Normalize whitespace in attribute values per XML specification.
///
/// XML attributes should have whitespace normalized:
/// - All whitespace characters (space, tab, newline, carriage return) → single space
/// - Multiple consecutive spaces → single space
/// - Leading/trailing whitespace is trimmed
///
/// This matches Python xacro behavior and ensures multiline attribute values
/// (from expressions spanning multiple lines) are properly normalized.
///
/// # Arguments
/// * `value` - The attribute value to normalize
///
/// # Returns
/// Normalized string with whitespace collapsed
pub(super) fn normalize_attribute_whitespace(value: &str) -> String {
    value.split_whitespace().collect::<Vec<_>>().join(" ")
}

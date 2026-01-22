//! YAML custom tag handler trait and implementations
//!
//! This module provides a trait for handling custom YAML tags (e.g., `!degrees`, `!millimeters`)
//! during YAML parsing. Tag handlers can be injected at runtime to customize behavior.

/// Trait for handling custom YAML tags during parsing
///
/// Implementors can provide custom behavior for YAML tags like `!degrees`, `!radians`, etc.
/// The handler receives a string representation of the tagged value and returns a Python
/// expression string that will be evaluated later.
///
/// **Important**: Handlers only receive scalar values (integers, floats, strings).
/// Tags applied to sequences or mappings are not passed to handlers, matching Python xacro behavior.
///
/// # Examples
///
/// ```rust,ignore
/// struct DegreesToRadians;
///
/// impl YamlTagHandler for DegreesToRadians {
///     fn handle_tag(&self, tag: &str, raw_value: &str) -> Option<String> {
///         if tag == "degrees" {
///             let trimmed = raw_value.trim();
///             // Try parsing as number
///             if let Ok(num) = trimmed.parse::<f64>() {
///                 let result = num * std::f64::consts::PI / 180.0;
///                 Some(result.to_string())
///             } else {
///                 // Treat as expression: wrap in conversion
///                 Some(format!("({})*{}", trimmed, std::f64::consts::PI / 180.0))
///             }
///         } else {
///             None
///         }
///     }
/// }
/// ```
pub trait YamlTagHandler {
    /// Handle a YAML tag with the given raw value
    ///
    /// # Arguments
    /// * `tag` - The tag suffix (e.g., "degrees" from "!degrees")
    /// * `raw_value` - The string representation of the scalar value (integer, float, or string text)
    ///
    /// # Returns
    /// * `Some(String)` - Python expression string to use as the value
    /// * `None` - Tag not handled by this handler (try next handler or fall through)
    ///
    /// # Note
    /// The returned string should be a valid Python expression that can be evaluated
    /// by pyisheval. For numeric literals, return the converted value as a string.
    /// For expressions (e.g., "45*2"), wrap them with conversion factor if needed.
    /// For string values, the handler is responsible for proper quoting if needed.
    ///
    /// Handlers only receive scalar values. Tags on sequences/mappings are filtered out upstream.
    fn handle_tag(
        &self,
        tag: &str,
        raw_value: &str,
    ) -> Option<String>;
}

/// Type alias for boxed YAML tag handlers
pub type DynYamlTagHandler = Box<dyn YamlTagHandler>;

/// Registry for YAML tag handlers
///
/// Handlers are tried in registration order. The first handler that returns `Some`
/// wins. If no handlers match, the value is used as-is.
#[derive(Default)]
pub(crate) struct YamlTagHandlerRegistry {
    handlers: Vec<DynYamlTagHandler>,
}

impl YamlTagHandlerRegistry {
    /// Create a new empty registry
    pub fn new() -> Self {
        Self {
            handlers: Vec::new(),
        }
    }

    /// Register a YAML tag handler
    ///
    /// Handlers are tried in registration order. Register more specific handlers
    /// before more general ones.
    pub fn register(
        &mut self,
        handler: DynYamlTagHandler,
    ) {
        self.handlers.push(handler);
    }

    /// Handle a YAML tag by trying all registered handlers in order
    ///
    /// # Arguments
    /// * `tag` - The tag suffix (e.g., "degrees" from "!degrees")
    /// * `raw_value` - The raw string representation of the value
    ///
    /// # Returns
    /// * `Some(String)` - Converted value from first matching handler
    /// * `None` - No handlers matched this tag
    pub fn handle_tag(
        &self,
        tag: &str,
        raw_value: &str,
    ) -> Option<String> {
        for handler in &self.handlers {
            if let Some(result) = handler.handle_tag(tag, raw_value) {
                return Some(result);
            }
        }
        None
    }
}

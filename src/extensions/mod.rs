//! Extension system for $(command args...) substitutions.
//!
//! This module provides a trait-based extension system for xacro $(...)
//! substitutions. Extensions can be registered with the processor and
//! will be called in order when a $(...) expression is encountered.

pub(crate) mod core;
pub(crate) mod ros;

#[cfg(feature = "yaml")]
pub(crate) mod ros_yaml_handlers;

// Re-export types for binary and public API use
pub use self::core::{CwdExtension, EnvExtension};
pub use self::ros::{FindExtension, OptEnvExtension};

// Re-export YAML tag handler types for public API
#[cfg(feature = "yaml")]
pub use crate::eval::yaml_tag_handler::{DynYamlTagHandler, YamlTagHandler};

use ::core::cell::RefCell;
use std::collections::HashMap;
use std::error::Error as StdError;
use std::rc::Rc;

/// Trait for resolving $(command args...) extensions.
///
/// Handlers receive the raw argument string and decide how to parse it.
/// This allows each extension to handle tokenization according to its
/// specific requirements (typically using simple whitespace splitting).
///
/// NOTE: This trait does NOT require Send + Sync because xacro processing
/// is single-threaded. Extensions may use Rc<RefCell<>> for shared state.
pub trait ExtensionHandler {
    /// Attempt to resolve an extension.
    ///
    /// # Parameters
    /// - `command`: The extension verb (e.g., "cwd", "env", "find")
    /// - `args_raw`: Everything after "$(cmd " and before ")" (may be empty)
    ///
    /// # Returns
    /// - `Ok(Some(string))`: Successfully resolved
    /// - `Ok(None)`: This handler doesn't recognize the command (try next handler)
    /// - `Err(e)`: Command recognized but resolution failed
    ///
    /// # Examples
    /// ```text
    /// Input: "$(env HOME)"
    /// command = "env"
    /// args_raw = "HOME"
    ///
    /// Input: "$(optenv VAR default value)"
    /// command = "optenv"
    /// args_raw = "VAR default value"
    ///
    /// Input: "$(cwd)"
    /// command = "cwd"
    /// args_raw = ""
    /// ```
    fn resolve(
        &self,
        command: &str,
        args_raw: &str,
    ) -> Result<Option<String>, Box<dyn StdError>>;

    /// Lifecycle hook: Called when the processor enters a new file context.
    ///
    /// Extensions that need to track the current file being processed can
    /// override this method. The default implementation does nothing.
    ///
    /// # Parameters
    /// - `current_file`: Path to the file currently being processed
    fn on_file_change(
        &self,
        _current_file: &std::path::Path,
    ) {
        // Default implementation does nothing
    }
}

/// Shared argument registry with encapsulated interior mutability.
///
/// This registry stores xacro arguments populated by:
/// - CLI flags (--arg name:=value)
/// - XML declarations (<xacro:arg name="..." value="..."/>)
/// - XML defaults (<xacro:arg name="..." default="..."/>)
///
/// IMPORTANT: Single-threaded design (NOT thread-safe):
/// - Uses Rc<RefCell<>>, NOT Arc<Mutex<>>
/// - xacro processing is inherently sequential (XML parsing order matters)
/// - Rc is cheaper than Arc (no atomic operations)
///
/// NOTE: This struct exists for external extension authors who want to implement
/// custom `$(arg ...)` handling. The core xacro processor handles `$(arg ...)`
/// specially in `EvalContext::resolve_extension()` to ensure correct shared state
/// with `xacro:arg` directives. See `EXTENSION_IMPL_ACTUAL.md` for rationale.
#[derive(Clone)]
pub struct ArgRegistry(Rc<RefCell<HashMap<String, String>>>);

impl ArgRegistry {
    /// Create a new empty argument registry.
    pub fn new() -> Self {
        Self(Rc::new(RefCell::new(HashMap::new())))
    }

    /// Create from existing `Rc<RefCell<HashMap>>` (for compatibility with existing code).
    pub fn from_rc(inner: Rc<RefCell<HashMap<String, String>>>) -> Self {
        Self(inner)
    }

    /// Get the inner `Rc<RefCell<HashMap>>` (for compatibility with existing code).
    pub fn inner(&self) -> Rc<RefCell<HashMap<String, String>>> {
        self.0.clone()
    }

    /// Get an argument value by name.
    pub fn get(
        &self,
        key: &str,
    ) -> Option<String> {
        self.0.borrow().get(key).cloned()
    }

    /// Set an argument value.
    pub fn set(
        &mut self,
        key: impl Into<String>,
        value: impl Into<String>,
    ) {
        self.0.borrow_mut().insert(key.into(), value.into());
    }

    /// Check if an argument is defined.
    pub fn contains(
        &self,
        key: &str,
    ) -> bool {
        self.0.borrow().contains_key(key)
    }

    /// Insert multiple arguments from a map.
    pub fn extend(
        &mut self,
        args: HashMap<String, String>,
    ) {
        self.0.borrow_mut().extend(args);
    }
}

impl Default for ArgRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Extension handler for $(arg name).
///
/// Reads from a shared argument registry that is populated by:
/// 1. CLI: --arg name:=value flags
/// 2. XML: <xacro:arg name="..." value="..."/> declarations
/// 3. XML: <xacro:arg name="..." default="..."/> declarations (if not overridden)
pub struct ArgExtension {
    registry: ArgRegistry,
}

impl ArgExtension {
    pub fn new(registry: ArgRegistry) -> Self {
        Self { registry }
    }
}

impl ExtensionHandler for ArgExtension {
    fn resolve(
        &self,
        command: &str,
        args_raw: &str,
    ) -> Result<Option<String>, Box<dyn StdError>> {
        if command != "arg" {
            return Ok(None);
        }

        let args = extension_utils::expect_args(args_raw, "arg", 1)?;
        let arg_name = &args[0];

        self.registry
            .get(arg_name)
            .ok_or_else(|| format!("Undefined argument: '{}'", arg_name).into())
            .map(Some)
    }
}

/// Utilities for extension handlers (internal use only).
pub(crate) mod extension_utils {
    use std::error::Error as StdError;

    /// Tokenize arguments using simple whitespace splitting.
    ///
    /// Matches Python xacro behavior (string.split(' ') semantics).
    /// Does NOT handle shell-style quoting - see design notes for rationale.
    pub(crate) fn tokenize_args(s: &str) -> Vec<String> {
        // Simple whitespace splitting to match ROS behavior
        // Python xacro uses: string.split(' ')
        // We use split_whitespace() for slightly better handling of multiple spaces
        s.split_whitespace().map(|s| s.to_string()).collect()
    }

    /// Extract exactly N arguments, error if count mismatches.
    pub(crate) fn expect_args(
        raw: &str,
        cmd: &str,
        expected: usize,
    ) -> Result<Vec<String>, Box<dyn StdError>> {
        let args = tokenize_args(raw);
        if args.len() != expected {
            Err(format!(
                "$({}) expects {} argument(s), got {}",
                cmd,
                expected,
                args.len()
            )
            .into())
        } else {
            Ok(args)
        }
    }

    /// Extract 1-N arguments (min/max validation).
    pub(crate) fn expect_args_range(
        raw: &str,
        cmd: &str,
        min: usize,
        max: usize,
    ) -> Result<Vec<String>, Box<dyn StdError>> {
        let args = tokenize_args(raw);
        if args.len() < min || args.len() > max {
            Err(format!(
                "$({}) expects {}-{} arguments, got {}",
                cmd,
                min,
                max,
                args.len()
            )
            .into())
        } else {
            Ok(args)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::extension_utils::*;
    use super::ExtensionHandler;
    use std::collections::HashMap;

    #[test]
    fn test_tokenize_args_simple() {
        let tokens = tokenize_args("a b c");
        assert_eq!(tokens, vec!["a", "b", "c"]);
    }

    #[test]
    fn test_tokenize_args_single() {
        let tokens = tokenize_args("single");
        assert_eq!(tokens, vec!["single"]);
    }

    #[test]
    fn test_tokenize_args_empty() {
        let tokens = tokenize_args("");
        assert!(tokens.is_empty());
    }

    #[test]
    fn test_tokenize_args_whitespace_only() {
        let tokens = tokenize_args("   \t  \n  ");
        assert!(tokens.is_empty());
    }

    #[test]
    fn test_tokenize_args_multiple_spaces() {
        let tokens = tokenize_args("a    b     c");
        assert_eq!(tokens, vec!["a", "b", "c"]);
    }

    #[test]
    fn test_tokenize_args_leading_trailing_spaces() {
        let tokens = tokenize_args("  a b c  ");
        assert_eq!(tokens, vec!["a", "b", "c"]);
    }

    #[test]
    fn test_tokenize_args_quotes_not_special() {
        // Quotes are NOT special - they're just regular characters
        // This matches Python xacro behavior (simple split)
        let tokens = tokenize_args("VAR \"default value\"");
        assert_eq!(tokens, vec!["VAR", "\"default", "value\""]);
    }

    #[test]
    fn test_expect_args_valid() {
        let result = expect_args("a b", "cmd", 2);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec!["a", "b"]);
    }

    #[test]
    fn test_expect_args_single() {
        let result = expect_args("single", "cmd", 1);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec!["single"]);
    }

    #[test]
    fn test_expect_args_mismatch_too_few() {
        let result = expect_args("a", "cmd", 2);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 2 argument(s), got 1"));
    }

    #[test]
    fn test_expect_args_mismatch_too_many() {
        let result = expect_args("a b c", "cmd", 2);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 2 argument(s), got 3"));
    }

    #[test]
    fn test_expect_args_zero_args() {
        let result = expect_args("", "cmd", 0);
        assert!(result.is_ok());
        assert!(result.unwrap().is_empty());
    }

    #[test]
    fn test_expect_args_range_valid_min() {
        let result = expect_args_range("a", "cmd", 1, 3);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec!["a"]);
    }

    #[test]
    fn test_expect_args_range_valid_mid() {
        let result = expect_args_range("a b", "cmd", 1, 3);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec!["a", "b"]);
    }

    #[test]
    fn test_expect_args_range_valid_max() {
        let result = expect_args_range("a b c", "cmd", 1, 3);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec!["a", "b", "c"]);
    }

    #[test]
    fn test_expect_args_range_too_few() {
        let result = expect_args_range("", "cmd", 1, 3);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 1-3 arguments, got 0"));
    }

    #[test]
    fn test_expect_args_range_too_many() {
        let result = expect_args_range("a b c d", "cmd", 1, 3);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 1-3 arguments, got 4"));
    }

    #[test]
    fn test_optenv_multi_arg_behavior() {
        // This test verifies the optenv behavior described in design:
        // $(optenv VAR a b c) should split to ["VAR", "a", "b", "c"]
        // The handler will then join args[1..] to get "a b c" as default
        let tokens = tokenize_args("VAR a b c");
        assert_eq!(tokens, vec!["VAR", "a", "b", "c"]);

        // Simulate what optenv handler does
        let var_name = &tokens[0];
        let default_value = tokens[1..].join(" ");
        assert_eq!(var_name, "VAR");
        assert_eq!(default_value, "a b c");
    }

    #[test]
    fn test_arg_single_token_enforcement() {
        // $(arg name) must have exactly 1 token
        let result = expect_args("name", "arg", 1);
        assert!(result.is_ok());

        // $(arg a b) should fail
        let result = expect_args("a b", "arg", 1);
        assert!(result.is_err());
    }

    // ArgRegistry tests
    #[test]
    fn test_arg_registry_new() {
        let registry = super::ArgRegistry::new();
        assert!(!registry.contains("key"));
    }

    #[test]
    fn test_arg_registry_set_get() {
        let mut registry = super::ArgRegistry::new();
        registry.set("key", "value");
        assert_eq!(registry.get("key"), Some("value".to_string()));
    }

    #[test]
    fn test_arg_registry_contains() {
        let mut registry = super::ArgRegistry::new();
        assert!(!registry.contains("key"));
        registry.set("key", "value");
        assert!(registry.contains("key"));
    }

    #[test]
    fn test_arg_registry_extend() {
        let mut registry = super::ArgRegistry::new();
        let mut map = HashMap::new();
        map.insert("key1".to_string(), "value1".to_string());
        map.insert("key2".to_string(), "value2".to_string());

        registry.extend(map);
        assert_eq!(registry.get("key1"), Some("value1".to_string()));
        assert_eq!(registry.get("key2"), Some("value2".to_string()));
    }

    #[test]
    fn test_arg_registry_clone_shares_state() {
        let mut registry1 = super::ArgRegistry::new();
        registry1.set("key", "value1");

        let mut registry2 = registry1.clone();
        assert_eq!(registry2.get("key"), Some("value1".to_string()));

        // Mutations through one clone are visible in the other
        registry2.set("key", "value2");
        assert_eq!(registry1.get("key"), Some("value2".to_string()));
    }

    // ArgExtension tests
    #[test]
    fn test_arg_extension_success() {
        let mut registry = super::ArgRegistry::new();
        registry.set("test_arg", "test_value");

        let ext = super::ArgExtension::new(registry);
        let result = ext.resolve("arg", "test_arg");

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("test_value".to_string()));
    }

    #[test]
    fn test_arg_extension_wrong_command() {
        let registry = super::ArgRegistry::new();
        let ext = super::ArgExtension::new(registry);
        let result = ext.resolve("notarg", "test_arg");

        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_arg_extension_undefined() {
        let registry = super::ArgRegistry::new();
        let ext = super::ArgExtension::new(registry);
        let result = ext.resolve("arg", "undefined_arg");

        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("Undefined") || err_msg.contains("not defined"));
    }

    #[test]
    fn test_arg_extension_no_args() {
        let registry = super::ArgRegistry::new();
        let ext = super::ArgExtension::new(registry);
        let result = ext.resolve("arg", "");

        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 1 argument(s), got 0"));
    }

    #[test]
    fn test_arg_extension_too_many_args() {
        let registry = super::ArgRegistry::new();
        let ext = super::ArgExtension::new(registry);
        let result = ext.resolve("arg", "arg1 arg2");

        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 1 argument(s), got 2"));
    }
}

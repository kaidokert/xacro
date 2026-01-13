//! Core extension handlers that are always available.

use super::{extension_utils, ExtensionHandler};
use std::error::Error as StdError;

/// Handles $(cwd) - returns current working directory.
///
/// # Examples
/// ```
/// // In xacro file:
/// // <path>$(cwd)/models/robot.urdf</path>
/// //
/// // Resolves to something like:
/// // <path>/home/user/project/models/robot.urdf</path>
/// ```
pub struct CwdExtension;

impl ExtensionHandler for CwdExtension {
    fn resolve(
        &self,
        command: &str,
        args_raw: &str,
    ) -> Result<Option<String>, Box<dyn StdError>> {
        if command != "cwd" {
            return Ok(None);
        }

        if !args_raw.trim().is_empty() {
            return Err("$(cwd) does not take arguments".into());
        }

        std::env::current_dir()
            .map(|p| Some(p.display().to_string()))
            .map_err(|e| Box::new(e) as Box<dyn StdError>)
    }
}

/// Handles $(env VAR) - returns environment variable.
///
/// # Examples
/// ```
/// // In xacro file:
/// // <user>$(env USER)</user>
/// //
/// // Resolves to the USER environment variable value
/// ```
pub struct EnvExtension;

impl ExtensionHandler for EnvExtension {
    fn resolve(
        &self,
        command: &str,
        args_raw: &str,
    ) -> Result<Option<String>, Box<dyn StdError>> {
        if command != "env" {
            return Ok(None);
        }

        let args = extension_utils::expect_args(args_raw, "env", 1)?;
        let var_name = &args[0];

        std::env::var(var_name)
            .map(Some)
            .map_err(|e| Box::new(e) as Box<dyn StdError>)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cwd_extension_success() {
        let ext = CwdExtension;
        let result = ext.resolve("cwd", "");

        assert!(result.is_ok());
        let resolved = result.unwrap();
        assert!(resolved.is_some());

        // Should return a non-empty path
        let path = resolved.unwrap();
        assert!(!path.is_empty());
    }

    #[test]
    fn test_cwd_extension_wrong_command() {
        let ext = CwdExtension;
        let result = ext.resolve("notcwd", "");

        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_cwd_extension_with_args() {
        let ext = CwdExtension;
        let result = ext.resolve("cwd", "unexpected");

        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("does not take arguments"));
    }

    #[test]
    fn test_cwd_extension_whitespace_args() {
        let ext = CwdExtension;
        // Pure whitespace should be treated as no args
        let result = ext.resolve("cwd", "   ");
        assert!(result.is_ok());
    }

    #[test]
    fn test_env_extension_success() {
        let ext = EnvExtension;

        // Set a test environment variable
        std::env::set_var("XACRO_TEST_VAR", "test_value");

        let result = ext.resolve("env", "XACRO_TEST_VAR");

        assert!(result.is_ok());
        let resolved = result.unwrap();
        assert!(resolved.is_some());
        assert_eq!(resolved.unwrap(), "test_value");

        // Clean up
        std::env::remove_var("XACRO_TEST_VAR");
    }

    #[test]
    fn test_env_extension_wrong_command() {
        let ext = EnvExtension;
        let result = ext.resolve("notenv", "VAR");

        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_env_extension_no_args() {
        let ext = EnvExtension;
        let result = ext.resolve("env", "");

        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 1 argument(s), got 0"));
    }

    #[test]
    fn test_env_extension_too_many_args() {
        let ext = EnvExtension;
        let result = ext.resolve("env", "VAR1 VAR2");

        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("expects 1 argument(s), got 2"));
    }

    #[test]
    fn test_env_extension_undefined_var() {
        let ext = EnvExtension;

        // Make sure this var doesn't exist
        std::env::remove_var("XACRO_NONEXISTENT_VAR_12345");

        let result = ext.resolve("env", "XACRO_NONEXISTENT_VAR_12345");

        // Should return an error for undefined environment variable
        assert!(result.is_err());
    }
}

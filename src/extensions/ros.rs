//! ROS-specific extensions (find, optenv)

use crate::extensions::{extension_utils, ExtensionHandler};
use core::cell::RefCell;
use std::{
    collections::HashMap,
    env,
    error::Error as StdError,
    fs,
    path::{Path, PathBuf},
};

/// Extension for $(find package_name) - finds ROS package paths
///
/// Resolution strategy:
/// 1. Check cache first (for performance)
/// 2. Search $ROS_PACKAGE_PATH directories
/// 3. Discover workspaces (look for .catkin_workspace, install/, build/ markers)
/// 4. Search workspace src/ directories for package.xml files
/// 5. Cache result for future lookups
///
/// Performance target: <10ms for 100 consecutive calls (via caching)
pub struct FindExtension {
    /// Cache of package_name -> package_path
    cache: RefCell<HashMap<String, PathBuf>>,
    /// Search paths (from ROS_PACKAGE_PATH + workspace discovery)
    search_paths: RefCell<Option<Vec<PathBuf>>>,
}

impl FindExtension {
    /// Create a new FindExtension with empty cache
    ///
    /// Search paths will be discovered from ROS_PACKAGE_PATH and workspace detection.
    pub fn new() -> Self {
        Self {
            cache: RefCell::new(HashMap::new()),
            search_paths: RefCell::new(None),
        }
    }

    /// Create a FindExtension with explicit search paths
    ///
    /// This is useful for testing or when you want to override the default
    /// search path discovery mechanism.
    pub fn with_search_paths(search_paths: Vec<PathBuf>) -> Self {
        Self {
            cache: RefCell::new(HashMap::new()),
            search_paths: RefCell::new(Some(search_paths)),
        }
    }

    /// Get search paths (lazy initialization)
    fn get_search_paths(&self) -> Vec<PathBuf> {
        // Return cached paths if available
        if let Some(ref paths) = *self.search_paths.borrow() {
            return paths.clone();
        }

        let mut paths = Vec::new();

        // 1. Add paths from ROS_PACKAGE_PATH environment variable
        if let Ok(ros_path) = env::var("ROS_PACKAGE_PATH") {
            for dir in ros_path.split(':') {
                let path = PathBuf::from(dir);
                if path.exists() {
                    paths.push(path);
                }
            }
        }

        // 2. Discover workspace roots and add their src/ directories
        if let Ok(cwd) = env::current_dir() {
            if let Some(workspace_root) = Self::find_workspace_root(&cwd) {
                let src_dir = workspace_root.join("src");
                if src_dir.exists() {
                    paths.push(src_dir);
                }
            }
        }

        // 3. Cache the search paths
        *self.search_paths.borrow_mut() = Some(paths.clone());

        paths
    }

    /// Find workspace root by looking for markers (.catkin_workspace, install/, build/)
    ///
    /// Walks upward from the given directory until a workspace marker is found.
    fn find_workspace_root(start_dir: &Path) -> Option<PathBuf> {
        let mut current = start_dir;

        loop {
            // Check for workspace markers
            if current.join(".catkin_workspace").exists()
                || (current.join("install").exists() && current.join("build").exists())
            {
                return Some(current.to_path_buf());
            }

            // Move up one directory
            current = current.parent()?;
        }
    }

    /// Check if a directory is a ROS package
    ///
    /// A ROS package is identified by the presence of either:
    /// - package.xml (ROS 2 / catkin format)
    /// - manifest.xml (ROS 1 / rosbuild format)
    fn is_ros_package(path: &Path) -> bool {
        path.join("package.xml").exists() || path.join("manifest.xml").exists()
    }

    /// Read the package name from package.xml or manifest.xml
    ///
    /// Returns the actual package name as defined in the package metadata,
    /// which may differ from the directory name (e.g., "tams_apriltags"
    /// in a directory named "tams_apriltags-master").
    fn read_package_name(path: &Path) -> Option<String> {
        // Try package.xml first (ROS 2 / catkin)
        let pkg_xml = path.join("package.xml");
        if pkg_xml.exists() {
            if let Ok(contents) = fs::read_to_string(&pkg_xml) {
                // Simple XML parsing - look for <name>...</name>
                if let Some(start) = contents.find("<name>") {
                    if let Some(end) = contents[start..].find("</name>") {
                        let name_start = start + 6; // len("<name>")
                        let name_end = start + end;
                        return Some(contents[name_start..name_end].trim().to_string());
                    }
                }
            }
        }

        // Try manifest.xml (ROS 1 / rosbuild)
        let manifest_xml = path.join("manifest.xml");
        if manifest_xml.exists() {
            if let Ok(contents) = fs::read_to_string(&manifest_xml) {
                // Look for <package> element
                // Format: <package> or <package name="...">
                if let Some(start) = contents.find("<package") {
                    // Check for name attribute: <package name="pkg_name">
                    if let Some(name_attr) = contents[start..].find("name=\"") {
                        let attr_start = start + name_attr + 6; // len("name=\"")
                        if let Some(attr_end) = contents[attr_start..].find('"') {
                            return Some(
                                contents[attr_start..attr_start + attr_end]
                                    .trim()
                                    .to_string(),
                            );
                        }
                    }
                    // If no name attribute, manifest uses directory name
                    // Return None to signal directory name should be used
                }
            }
        }

        None
    }

    /// Search for a package in the given search paths
    ///
    /// A ROS package is normally identified by package.xml or manifest.xml.
    /// When ROS_PACKAGE_PATH contains direct package directories, we check
    /// the package metadata to get the actual package name (since directory
    /// names may differ, e.g., "tams_apriltags-master" for "tams_apriltags").
    fn search_package(
        &self,
        package_name: &str,
        search_paths: &[PathBuf],
    ) -> Option<PathBuf> {
        for search_path in search_paths {
            // Check if search_path itself is the package (self-match)
            // Accept if directory matches by name OR by package metadata
            if search_path.exists() {
                // Quick check: directory name matches package name
                if let Some(dir_name) = search_path.file_name().and_then(|n| n.to_str()) {
                    if dir_name == package_name {
                        return Some(search_path.clone());
                    }
                }

                // Robust check: read package.xml/manifest.xml for actual package name
                // (handles cases like "tams_apriltags-master" directory containing
                // "tams_apriltags" package)
                if Self::is_ros_package(search_path) {
                    if let Some(actual_name) = Self::read_package_name(search_path) {
                        if actual_name == package_name {
                            return Some(search_path.clone());
                        }
                    }
                }
            }

            // Direct match: search_path/package_name/
            let direct_path = search_path.join(package_name);
            if Self::is_ros_package(&direct_path) {
                return Some(direct_path);
            }

            // Search subdirectories (one level deep for performance)
            if let Ok(entries) = fs::read_dir(search_path) {
                for entry in entries.flatten() {
                    if let Ok(file_type) = entry.file_type() {
                        if file_type.is_dir() {
                            let pkg_path = entry.path().join(package_name);
                            if Self::is_ros_package(&pkg_path) {
                                return Some(pkg_path);
                            }
                        }
                    }
                }
            }
        }

        None
    }

    /// Find a package and return its path (with caching)
    fn find_package(
        &self,
        package_name: &str,
    ) -> Result<PathBuf, Box<dyn StdError>> {
        // Check cache first
        if let Some(cached_path) = self.cache.borrow().get(package_name) {
            return Ok(cached_path.clone());
        }

        // Get search paths (lazy init)
        let search_paths = self.get_search_paths();

        // Search for package
        if let Some(pkg_path) = self.search_package(package_name, &search_paths) {
            // Cache the result
            self.cache
                .borrow_mut()
                .insert(package_name.to_string(), pkg_path.clone());
            return Ok(pkg_path);
        }

        Err(format!("Package not found: '{}'", package_name).into())
    }
}

impl Default for FindExtension {
    fn default() -> Self {
        Self::new()
    }
}

impl ExtensionHandler for FindExtension {
    fn resolve(
        &self,
        command: &str,
        args_raw: &str,
    ) -> Result<Option<String>, Box<dyn StdError>> {
        if command != "find" {
            return Ok(None);
        }

        let args = extension_utils::expect_args(args_raw, "find", 1)?;
        let package_name = &args[0];

        let pkg_path = self.find_package(package_name)?;
        Ok(Some(pkg_path.display().to_string()))
    }
}

/// Extension for $(optenv VAR [default value])
///
/// Gets environment variable with optional default value.
/// If the variable is not set, returns the default (which can contain spaces).
///
/// Examples:
/// - $(optenv ROBOT_NAME)           → value or error if not set
/// - $(optenv ROBOT_NAME default)   → value or "default"
/// - $(optenv ROBOT_NAME my robot)  → value or "my robot" (multi-word default)
pub struct OptEnvExtension;

impl OptEnvExtension {
    pub fn new() -> Self {
        Self
    }
}

impl Default for OptEnvExtension {
    fn default() -> Self {
        Self::new()
    }
}

impl ExtensionHandler for OptEnvExtension {
    fn resolve(
        &self,
        command: &str,
        args_raw: &str,
    ) -> Result<Option<String>, Box<dyn StdError>> {
        if command != "optenv" {
            return Ok(None);
        }

        let args = extension_utils::expect_args_range(args_raw, "optenv", 1, usize::MAX)?;

        let var_name = &args[0];

        // Try to get the environment variable
        match env::var(var_name) {
            Ok(value) => Ok(Some(value)),
            Err(_) => {
                // Variable not set - use default if provided
                if args.len() > 1 {
                    // Join remaining args with spaces for multi-word defaults
                    // Matches Python xacro: ' '.join(args[1:])
                    let default = args[1..].join(" ");
                    Ok(Some(default))
                } else {
                    // No default provided - return error
                    Err(format!("Environment variable not set: '{}'", var_name).into())
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_extension_caching() {
        let ext = FindExtension::new();
        // Cache should be initially empty
        assert_eq!(ext.cache.borrow().len(), 0);
    }

    #[test]
    fn test_optenv_with_default() {
        let ext = OptEnvExtension::new();

        // Set test env var
        env::set_var("TEST_OPTENV_VAR", "test_value");

        // Should return the env var value
        let result = ext.resolve("optenv", "TEST_OPTENV_VAR fallback");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("test_value".to_string()));

        // Clean up
        env::remove_var("TEST_OPTENV_VAR");
    }

    #[test]
    fn test_optenv_fallback_single_word() {
        let ext = OptEnvExtension::new();

        // Use a var that doesn't exist
        let result = ext.resolve("optenv", "NONEXISTENT_VAR default");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("default".to_string()));
    }

    #[test]
    fn test_optenv_fallback_multi_word() {
        let ext = OptEnvExtension::new();

        // Multi-word default should be joined with spaces
        let result = ext.resolve("optenv", "NONEXISTENT_VAR my default value");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("my default value".to_string()));
    }

    #[test]
    fn test_optenv_no_default_fails() {
        let ext = OptEnvExtension::new();

        // Should fail if var doesn't exist and no default
        let result = ext.resolve("optenv", "NONEXISTENT_VAR");
        assert!(result.is_err());
    }

    #[test]
    fn test_optenv_wrong_command_returns_none() {
        let ext = OptEnvExtension::new();
        let result = ext.resolve("other", "arg");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), None);
    }
}

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
use xmltree::Element;

// Platform-specific path separator for RUST_XACRO_PACKAGE_MAP
#[cfg(windows)]
const PATH_LIST_SEPARATOR: char = ';';
#[cfg(not(windows))]
const PATH_LIST_SEPARATOR: char = ':';

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
    /// Package map from RUST_XACRO_PACKAGE_MAP environment variable (lazy-loaded)
    package_map: RefCell<Option<HashMap<String, PathBuf>>>,
    /// Current file being processed (for ancestor package detection)
    current_file: RefCell<Option<PathBuf>>,
}

impl FindExtension {
    /// Create a new FindExtension with empty cache
    ///
    /// Search paths will be discovered from ROS_PACKAGE_PATH and workspace detection.
    pub fn new() -> Self {
        Self {
            cache: RefCell::new(HashMap::new()),
            search_paths: RefCell::new(None),
            package_map: RefCell::new(None),
            current_file: RefCell::new(None),
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
            package_map: RefCell::new(None),
            current_file: RefCell::new(None),
        }
    }

    /// Set the current file being processed (for ancestor package detection)
    fn set_current_file(
        &self,
        file: Option<PathBuf>,
    ) {
        *self.current_file.borrow_mut() = file;
    }

    /// Find a package by walking up ancestor directories from current file
    ///
    /// Looks for package.xml in ancestor directories and checks if the package
    /// name matches. Stops at the first package.xml found (doesn't traverse siblings).
    ///
    /// Returns an absolute path to avoid relative path resolution issues when
    /// the result is used in include directives.
    fn find_ancestor_package(
        &self,
        package_name: &str,
    ) -> Option<PathBuf> {
        let current_file = self.current_file.borrow();
        let file_path = current_file.as_ref()?;

        // Walk up from the file looking for package.xml
        for ancestor in file_path.ancestors().skip(1) {
            // skip(1) to skip the file itself
            let pkg_xml = ancestor.join("package.xml");
            if pkg_xml.exists() {
                // Try to read the package name from package.xml
                if let Ok(content) = fs::read_to_string(&pkg_xml) {
                    if let Ok(element) = Element::parse(content.as_bytes()) {
                        if let Some(name_elem) = element.get_child("name") {
                            if let Some(name_text) = name_elem.get_text() {
                                // Trim whitespace to match read_name_from_package_xml behavior
                                if name_text.trim() == package_name {
                                    // Convert to absolute path to avoid relative path resolution issues
                                    // when the result is used in include directives
                                    let abs_path = ancestor
                                        .canonicalize()
                                        .unwrap_or_else(|_| ancestor.to_path_buf());
                                    return Some(abs_path);
                                }
                            }
                        }
                    }
                }
                // Stop at first package.xml - don't search beyond package boundary
                break;
            }
        }

        None
    }

    /// Get search paths (lazy initialization)
    fn get_search_paths(&self) -> Vec<PathBuf> {
        // Return cached paths if available
        if let Some(ref paths) = *self.search_paths.borrow() {
            return paths.clone();
        }

        let mut paths = Vec::new();

        // 1. Add paths from ROS_PACKAGE_PATH environment variable
        // Use split_paths for cross-platform support (handles ':' on Unix, ';' on Windows)
        if let Ok(ros_path) = env::var("ROS_PACKAGE_PATH") {
            for path in env::split_paths(&ros_path) {
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

    /// Get package path from explicit package map (if configured).
    ///
    /// Format: `pkg1=/path1<SEP>pkg2=/path2` where `<SEP>` is:
    /// - `:` on Unix/Linux/macOS
    /// - `;` on Windows (platform-specific PATH separator)
    ///
    /// This provides hermetic package resolution by explicitly mapping
    /// package names to filesystem paths. When a package is found in this map,
    /// it overrides automatic ROS discovery mechanisms.
    ///
    /// Note: Package names cannot contain `=`. Paths can contain `=`, as only
    /// the first `=` in an entry is used as a separator. This is acceptable
    /// because ROS package names follow Python identifier rules (`[a-z0-9_]`).
    fn get_package_from_map(
        &self,
        package_name: &str,
    ) -> Option<PathBuf> {
        // Check cache first
        {
            let map_ref = self.package_map.borrow();
            if let Some(ref map) = *map_ref {
                return map.get(package_name).cloned();
            }
        }

        // Parse environment variable and cache
        let map_str = env::var("RUST_XACRO_PACKAGE_MAP").unwrap_or_default();
        let map = Self::parse_package_map(&map_str);

        let result = map.get(package_name).cloned();
        *self.package_map.borrow_mut() = Some(map);
        result
    }

    /// Parse package map string, handling paths with colons correctly.
    ///
    /// Format: `pkg1=/path1<SEP>pkg2=/path2` where `<SEP>` is `:` on Unix, `;` on Windows.
    ///
    /// ROS package names follow Python identifier rules: `[a-zA-Z][a-zA-Z0-9_]*`
    /// This allows us to identify package boundaries even when paths contain separators.
    ///
    /// Algorithm:
    /// 1. Find all positions where a valid package name followed by `=` appears
    /// 2. These mark entry boundaries (start of each `name=path` pair)
    /// 3. Extract the path between consecutive boundaries
    ///
    /// Examples:
    /// - `foo=/tmp:bar=/bin` → {foo: /tmp, bar: /bin}
    /// - `foo=/path/with:colon:bar=/other` → {foo: /path/with:colon, bar: /other}
    fn parse_package_map(input: &str) -> HashMap<String, PathBuf> {
        let mut map = HashMap::new();

        if input.is_empty() {
            return map;
        }

        // Find all entry start positions: <separator><name>= or ^<name>=
        // These mark the beginning of each "name=path" entry
        //
        // Note: Entries with invalid package names are silently skipped,
        // while entries with valid names are still parsed. This allows
        // partial parsing of mixed valid/invalid input (e.g., "123foo=/bad:bar=/good"
        // will parse only the "bar" entry).
        let mut entry_starts = Vec::new();
        let bytes = input.as_bytes();

        for i in 0..bytes.len() {
            // Check if this could be the start of an entry
            if i == 0 || bytes[i - 1] == PATH_LIST_SEPARATOR as u8 {
                // Find the '=' after this position
                if let Some(eq_pos) = input[i..].find('=') {
                    let eq_pos = i + eq_pos;
                    let name = input[i..eq_pos].trim();

                    if Self::is_valid_package_name(name) {
                        entry_starts.push((i, eq_pos));
                    }
                }
            }
        }

        // If we found no valid entries, fall back to simple split
        // This handles the common case where there's only one entry or simple paths
        //
        // Note: This fallback is used when the input contains NO valid ROS package
        // name patterns (e.g., "123foo=/path" or "=/path"). In this case, paths
        // with colons will NOT be correctly parsed and may be incorrectly split.
        // This is acceptable since the input itself is malformed (invalid package names).
        if entry_starts.is_empty() {
            return input
                .split(PATH_LIST_SEPARATOR)
                .filter_map(|entry| {
                    entry.split_once('=').and_then(|(name, path)| {
                        let name = name.trim();
                        let path = path.trim();
                        if !name.is_empty() && !path.is_empty() {
                            Some((name.to_string(), PathBuf::from(path)))
                        } else {
                            None
                        }
                    })
                })
                .collect();
        }

        // Extract each entry's name and path
        for (idx, &(start, eq_pos)) in entry_starts.iter().enumerate() {
            let name = input[start..eq_pos].trim();

            // Path starts after '=' and ends at the next entry or end of string
            let path_start = eq_pos + 1;
            let path_end = if idx + 1 < entry_starts.len() {
                // Path ends just before the separator before the next entry
                entry_starts[idx + 1].0.saturating_sub(1)
            } else {
                // Last entry - path goes to end of string
                input.len()
            };

            if path_end > path_start {
                let path = &input[path_start..path_end];
                let path = path.trim();

                if !path.is_empty() {
                    map.insert(name.to_string(), PathBuf::from(path));
                }
            }
        }

        map
    }

    /// Check if a string is a valid ROS package name.
    ///
    /// ROS package names follow Python package naming rules:
    /// - Must start with a letter (a-z, A-Z)
    /// - Can contain letters, numbers, and underscores
    /// - Cannot contain hyphens, dots, or other special characters
    fn is_valid_package_name(name: &str) -> bool {
        let name = name.trim();
        let bytes = name.as_bytes();

        // First character must be a letter
        match bytes.first() {
            Some(&first_byte) if first_byte.is_ascii_alphabetic() => (),
            _ => return false, // Handles empty string or invalid first char
        }

        // Remaining characters must be alphanumeric or underscore
        bytes[1..]
            .iter()
            .all(|&b| b.is_ascii_alphanumeric() || b == b'_')
    }

    /// Find workspace root by looking for markers (.catkin_workspace, install/, build/)
    ///
    /// Walks upward from the given directory until a workspace marker is found.
    fn find_workspace_root(start_dir: &Path) -> Option<PathBuf> {
        start_dir
            .ancestors()
            .find(|p| {
                p.join(".catkin_workspace").exists()
                    || (p.join("install").exists() && p.join("build").exists())
            })
            .map(|p| p.to_path_buf())
    }

    /// Check if a directory is a ROS package
    ///
    /// A ROS package is identified by the presence of either:
    /// - package.xml (ROS 2 / catkin format)
    /// - manifest.xml (ROS 1 / rosbuild format)
    fn is_ros_package(path: &Path) -> bool {
        path.join("package.xml").exists() || path.join("manifest.xml").exists()
    }

    /// Read package name from package.xml (ROS 2 / catkin format)
    fn read_name_from_package_xml(path: &Path) -> Option<String> {
        let pkg_xml = path.join("package.xml");
        if !pkg_xml.exists() {
            return None;
        }
        let file = fs::File::open(&pkg_xml).ok()?;
        let root = Element::parse(file).ok()?;
        root.get_child("name")?
            .get_text()
            .map(|t| t.trim().to_string())
    }

    /// Read package name from manifest.xml (ROS 1 / rosbuild format)
    fn read_name_from_manifest_xml(path: &Path) -> Option<String> {
        let manifest_xml = path.join("manifest.xml");
        if !manifest_xml.exists() {
            return None;
        }
        let file = fs::File::open(&manifest_xml).ok()?;
        let root = Element::parse(file).ok()?;
        if root.name == "package" {
            root.attributes
                .get(&xmltree::AttributeName::local("name"))
                .map(|n| n.trim().to_string())
        } else {
            None
        }
    }

    /// Read the package name from package.xml or manifest.xml
    ///
    /// Returns the actual package name as defined in the package metadata,
    /// which may differ from the directory name (e.g., "tams_apriltags"
    /// in a directory named "tams_apriltags-master").
    fn read_package_name(path: &Path) -> Option<String> {
        Self::read_name_from_package_xml(path).or_else(|| Self::read_name_from_manifest_xml(path))
    }

    /// Search for a package in the given search paths
    ///
    /// Resolution order:
    /// 1. Check RUST_XACRO_PACKAGE_MAP for a valid override; fall through if invalid
    /// 2. Search ROS_PACKAGE_PATH directories for package.xml/manifest.xml
    ///
    /// When ROS_PACKAGE_PATH contains direct package directories, we check
    /// the package metadata to get the actual package name (since directory
    /// names may differ, e.g., "tams_apriltags-master" for "tams_apriltags").
    fn search_package(
        &self,
        package_name: &str,
        search_paths: &[PathBuf],
    ) -> Option<PathBuf> {
        // FIRST: Check explicit package map (override semantics for hermetic builds)
        // Trust the explicit mapping without ROS package validation.
        // Accepts any existing directory, including:
        // - Data-only directories (no package.xml)
        // - Custom hermetic build layouts
        // - Test fixtures
        if let Some(path) = self.get_package_from_map(package_name) {
            if path.is_dir() {
                return Some(path);
            }
            // Warn on misconfiguration: mapped path doesn't exist or isn't a directory
            log::warn!(
                "RUST_XACRO_PACKAGE_MAP entry for '{}' points to non-existent or non-directory path: {}",
                package_name,
                path.display()
            );
        }

        // THEN: Standard ROS discovery (ROS_PACKAGE_PATH, etc.)
        for search_path in search_paths {
            // Check if search_path itself is the package (self-match)
            // Verify package name from metadata to ensure correctness
            if search_path.exists() && Self::is_ros_package(search_path) {
                // Always verify package name from package.xml/manifest.xml
                // This handles both exact directory name matches and mismatched names
                // (e.g., a directory named "my_package-master" for a package named "my_package")
                if let Some(actual_name) = Self::read_package_name(search_path) {
                    if actual_name == package_name {
                        return Some(search_path.clone());
                    }
                }
            }

            // Direct match: search_path/package_name/
            // This handles the common case where package name matches directory name
            // and the package is one level below the search path (typical ROS workspace layout)
            let direct_path = search_path.join(package_name);
            if Self::is_ros_package(&direct_path)
                && Self::read_package_name(&direct_path).as_deref() == Some(package_name)
            {
                return Some(direct_path);
            }

            // Search subdirectories (one level deep for performance)
            // This handles cases where packages are nested or have mismatched directory names
            if let Ok(entries) = fs::read_dir(search_path) {
                for entry in entries.flatten() {
                    if let Ok(file_type) = entry.file_type() {
                        if file_type.is_dir() {
                            let subdir_path = entry.path();
                            // Check if this subdirectory itself is the package
                            if Self::is_ros_package(&subdir_path) {
                                if let Some(actual_name) = Self::read_package_name(&subdir_path) {
                                    if actual_name == package_name {
                                        return Some(subdir_path);
                                    }
                                }
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

        // Check explicit package map first (hermetic mode takes precedence)
        // This ensures RUST_XACRO_PACKAGE_MAP overrides all filesystem discovery
        if let Some(pkg_path) = self.get_package_from_map(package_name) {
            if pkg_path.is_dir() {
                self.cache
                    .borrow_mut()
                    .insert(package_name.to_string(), pkg_path.clone());
                return Ok(pkg_path);
            }
        }

        // Check ancestor directories from current file
        // This handles the common case: xacro files referencing their own package
        if let Some(pkg_path) = self.find_ancestor_package(package_name) {
            // Cache the result
            self.cache
                .borrow_mut()
                .insert(package_name.to_string(), pkg_path.clone());
            return Ok(pkg_path);
        }

        // Get search paths (lazy init)
        let search_paths = self.get_search_paths();

        // Search ROS_PACKAGE_PATH and workspace discovery
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

    fn on_file_change(
        &self,
        current_file: &std::path::Path,
    ) {
        // Only set file context if path is an actual file (not directory)
        // This prevents stale context when run_from_string passes base_path
        if current_file.is_file() {
            self.set_current_file(Some(current_file.to_path_buf()));
        } else {
            self.set_current_file(None);
        }
    }
}

/// Extension for $(optenv VAR [default value])
///
/// Gets environment variable with optional default value.
/// If the variable is not set, returns the default (which can contain spaces).
///
/// Examples:
/// - $(optenv ROBOT_NAME)           → value or empty string if not set
/// - $(optenv ROBOT_NAME default)   → value or "default"
/// - $(optenv ROBOT_NAME my robot)  → value or "my robot" (multi-word default)
#[derive(Default)]
pub struct OptEnvExtension;

impl OptEnvExtension {
    pub fn new() -> Self {
        Self
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
                    // No default provided - return empty string
                    Ok(Some("".to_string()))
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::EnvVarGuard;
    use std::sync::atomic::{AtomicUsize, Ordering};

    static TEST_COUNTER: AtomicUsize = AtomicUsize::new(0);

    #[test]
    fn test_optenv_with_value_present() {
        let ext = OptEnvExtension::new();
        let test_id = TEST_COUNTER.fetch_add(1, Ordering::SeqCst);
        let var_name = format!("TEST_OPTENV_VAR_{}", test_id);

        // Set test env var with unique name to avoid parallel test conflicts
        let _guard = EnvVarGuard::new(&var_name, "test_value");

        // Should return the env var value
        let result = ext.resolve("optenv", &format!("{} fallback", var_name));
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("test_value".to_string()));
    }

    #[test]
    fn test_optenv_fallback_single_word() {
        env::remove_var("NONEXISTENT_VAR");
        let ext = OptEnvExtension::new();

        // Use a var that doesn't exist
        let result = ext.resolve("optenv", "NONEXISTENT_VAR default");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("default".to_string()));
    }

    #[test]
    fn test_optenv_fallback_multi_word() {
        env::remove_var("NONEXISTENT_VAR");
        let ext = OptEnvExtension::new();

        // Multi-word default should be joined with spaces
        let result = ext.resolve("optenv", "NONEXISTENT_VAR my default value");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("my default value".to_string()));
    }

    #[test]
    fn test_optenv_no_default_returns_empty() {
        env::remove_var("NONEXISTENT_VAR");
        let ext = OptEnvExtension::new();

        // Should return empty string if var doesn't exist and no default
        let result = ext.resolve("optenv", "NONEXISTENT_VAR");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("".to_string()));
    }

    #[test]
    fn test_optenv_wrong_command_returns_none() {
        let ext = OptEnvExtension::new();
        let result = ext.resolve("other", "arg");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), None);
    }

    #[test]
    fn test_parse_package_map_simple() {
        let input = "foo=/tmp:bar=/bin";
        let map = FindExtension::parse_package_map(input);

        assert_eq!(map.len(), 2);
        assert_eq!(map.get("foo"), Some(&PathBuf::from("/tmp")));
        assert_eq!(map.get("bar"), Some(&PathBuf::from("/bin")));
    }

    #[test]
    fn test_parse_package_map_path_with_colon() {
        // This is the bug case: path contains a colon
        let input = "urdf_demo=/opt/workshop/Workshop 7: Understanding URDFs/urdf_demo";
        let map = FindExtension::parse_package_map(input);

        assert_eq!(map.len(), 1);
        assert_eq!(
            map.get("urdf_demo"),
            Some(&PathBuf::from(
                "/opt/workshop/Workshop 7: Understanding URDFs/urdf_demo"
            ))
        );
    }

    #[test]
    fn test_parse_package_map_multiple_with_colons() {
        let input = "foo=/path/with:colon:bar=/other/path:baz=/tmp";
        let map = FindExtension::parse_package_map(input);

        assert_eq!(map.len(), 3);
        assert_eq!(map.get("foo"), Some(&PathBuf::from("/path/with:colon")));
        assert_eq!(map.get("bar"), Some(&PathBuf::from("/other/path")));
        assert_eq!(map.get("baz"), Some(&PathBuf::from("/tmp")));
    }

    #[test]
    fn test_parse_package_map_empty() {
        let input = "";
        let map = FindExtension::parse_package_map(input);
        assert_eq!(map.len(), 0);
    }

    #[test]
    fn test_parse_package_map_single_entry() {
        let input = "mypackage=/home/user/workspace/src/mypackage";
        let map = FindExtension::parse_package_map(input);

        assert_eq!(map.len(), 1);
        assert_eq!(
            map.get("mypackage"),
            Some(&PathBuf::from("/home/user/workspace/src/mypackage"))
        );
    }

    #[test]
    fn test_parse_package_map_whitespace() {
        let input = "  foo  =  /tmp  :  bar  =  /bin  ";
        let map = FindExtension::parse_package_map(input);

        assert_eq!(map.len(), 2);
        assert_eq!(map.get("foo"), Some(&PathBuf::from("/tmp")));
        assert_eq!(map.get("bar"), Some(&PathBuf::from("/bin")));
    }

    #[test]
    fn test_is_valid_package_name() {
        // Valid names
        assert!(FindExtension::is_valid_package_name("foo"));
        assert!(FindExtension::is_valid_package_name("foo_bar"));
        assert!(FindExtension::is_valid_package_name("foo123"));
        assert!(FindExtension::is_valid_package_name("f"));
        assert!(FindExtension::is_valid_package_name("FOO"));
        assert!(FindExtension::is_valid_package_name("FooBar"));

        // Invalid names
        assert!(!FindExtension::is_valid_package_name(""));
        assert!(!FindExtension::is_valid_package_name("123foo")); // starts with number
        assert!(!FindExtension::is_valid_package_name("foo-bar")); // hyphen not allowed
        assert!(!FindExtension::is_valid_package_name("foo.bar")); // dot not allowed
        assert!(!FindExtension::is_valid_package_name("foo bar")); // space not allowed
        assert!(!FindExtension::is_valid_package_name("_foo")); // starts with underscore
    }
}

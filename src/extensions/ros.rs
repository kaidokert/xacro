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

// Environment variable name for package path overrides
const PACKAGE_MAP_ENV_VAR: &str = "RUST_XACRO_PACKAGE_MAP";

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

    /// Get all packages that were resolved during processing.
    ///
    /// Returns a map of package names to their absolute paths. This includes:
    /// - All packages resolved via $(find package_name)
    /// - All packages declared in RUST_XACRO_PACKAGE_MAP (converted to absolute)
    ///
    /// Resolution priority: RUST_XACRO_PACKAGE_MAP entries override discovered
    /// packages when both exist for the same package name.
    ///
    /// Entries from RUST_XACRO_PACKAGE_MAP that point to non-existent paths
    /// are excluded with a warning logged.
    ///
    /// This is useful for dependency tracking and reporting which ROS packages
    /// were used during xacro processing.
    pub fn get_found_packages(&self) -> HashMap<String, PathBuf> {
        let mut result = self.cache.borrow().clone();

        // Merge in packages from RUST_XACRO_PACKAGE_MAP
        // These override discovered packages (env var takes precedence)
        self.ensure_package_map_loaded();

        if let Some(ref pkg_map) = *self.package_map.borrow() {
            for (pkg, path) in pkg_map.iter() {
                // Validate path exists and convert to absolute
                if let Some(abs_path) = Self::normalize_and_validate_path(pkg, path) {
                    result.insert(pkg.clone(), abs_path);
                }
            }
        }

        result
    }

    /// Convert a path to absolute and validate it exists.
    /// Returns None with a warning if the path can't be resolved or doesn't exist.
    fn normalize_and_validate_path(
        pkg_name: &str,
        path: &Path,
    ) -> Option<PathBuf> {
        // Convert to absolute without canonicalize() to avoid Windows \\?\ prefix issues
        let abs_path = if path.is_absolute() {
            path.to_path_buf()
        } else {
            match env::current_dir() {
                Ok(cwd) => cwd.join(path),
                Err(_) => {
                    log::warn!(
                        "{} entry for '{}' has relative path but current_dir unavailable: {}",
                        PACKAGE_MAP_ENV_VAR,
                        pkg_name,
                        path.display()
                    );
                    return None;
                }
            }
        };

        // Validate the path exists and is a directory
        if abs_path.is_dir() {
            Some(abs_path)
        } else {
            log::warn!(
                "{} entry for '{}' is not a valid directory: {}",
                PACKAGE_MAP_ENV_VAR,
                pkg_name,
                abs_path.display()
            );
            None
        }
    }

    /// Ensures the package map is loaded from environment variable (lazy initialization)
    fn ensure_package_map_loaded(&self) {
        if self.package_map.borrow().is_none() {
            let map_str = env::var(PACKAGE_MAP_ENV_VAR).unwrap_or_default();
            let map = Self::parse_package_map(&map_str);
            *self.package_map.borrow_mut() = Some(map);
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
    /// Looks for package.xml or manifest.xml in ancestor directories and checks
    /// if the package name matches. Stops at the first package boundary found.
    ///
    /// Returns an absolute path to avoid relative path resolution issues when
    /// the result is used in include directives.
    fn find_ancestor_package(
        &self,
        package_name: &str,
    ) -> Option<PathBuf> {
        let current_file = self.current_file.borrow();
        let file_path = current_file.as_ref()?;

        // Walk up from the file looking for package boundaries
        for ancestor in file_path.ancestors().skip(1) {
            // skip(1) to skip the file itself
            if Self::is_ros_package(ancestor) {
                // Use existing helper to read package name from package.xml or manifest.xml
                if Self::read_package_name(ancestor).as_deref() == Some(package_name) {
                    // Convert to absolute path without using canonicalize()
                    // canonicalize() adds \\?\ prefix on Windows which makes file system operations
                    // strict about path separators, causing issues with mixed separators
                    let abs_path = if ancestor.is_absolute() {
                        ancestor.to_path_buf()
                    } else if let Ok(cwd) = std::env::current_dir() {
                        cwd.join(ancestor)
                    } else {
                        // Can't determine absolute path - return None
                        return None;
                    };
                    return Some(abs_path);
                }
                // Stop at first package boundary - don't search beyond
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
        self.ensure_package_map_loaded();
        self.package_map
            .borrow()
            .as_ref()
            .and_then(|map| map.get(package_name).cloned())
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
        // Search ROS_PACKAGE_PATH and workspace directories
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

        // Try resolution strategies in order, storing result once found
        let pkg_path = {
            // 1. Check explicit package map first (hermetic mode takes precedence)
            if let Some(path) = self.get_package_from_map(package_name) {
                if path.is_dir() {
                    Some(path)
                } else {
                    // Warn on misconfiguration but continue to other strategies
                    log::warn!(
                        "{} entry for '{}' points to non-existent or non-directory path: {}",
                        PACKAGE_MAP_ENV_VAR,
                        package_name,
                        path.display()
                    );
                    None
                }
            } else {
                None
            }
        }
        // 2. Check ancestor directories from current file
        .or_else(|| self.find_ancestor_package(package_name))
        // 3. Search ROS_PACKAGE_PATH and workspace discovery
        .or_else(|| {
            let search_paths = self.get_search_paths();
            self.search_package(package_name, &search_paths)
        });

        // Cache and return result, or error if not found
        if let Some(path) = pkg_path {
            self.cache
                .borrow_mut()
                .insert(package_name.to_string(), path.clone());
            Ok(path)
        } else {
            Err(format!("Package not found: '{}'", package_name).into())
        }
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
        current_file: Option<&std::path::Path>,
    ) {
        self.set_current_file(current_file.map(|p| p.to_path_buf()));
    }

    fn as_any(&self) -> &dyn ::core::any::Any {
        self
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

    fn as_any(&self) -> &dyn ::core::any::Any {
        self
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

    #[test]
    fn test_ancestor_package_detection_with_package_xml() {
        use tempfile::TempDir;

        let tmpdir = TempDir::new().expect("Failed to create temp dir");
        let package_root = tmpdir.path().join("my_package");
        let urdf_dir = package_root.join("urdf");

        fs::create_dir_all(&urdf_dir).expect("Failed to create urdf dir");

        // Create package.xml
        let package_xml = package_root.join("package.xml");
        fs::write(
            &package_xml,
            r#"<?xml version="1.0"?>
<package format="2">
  <name>my_package</name>
  <version>1.0.0</version>
  <description>Test package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>BSD</license>
</package>"#,
        )
        .expect("Failed to write package.xml");

        // Create a dummy xacro file
        let xacro_file = urdf_dir.join("robot.xacro");
        fs::write(&xacro_file, "<robot/>").expect("Failed to write xacro");

        // Test ancestor detection
        let ext = FindExtension::new();
        ext.set_current_file(Some(xacro_file));

        let result = ext.find_ancestor_package("my_package");
        assert!(result.is_some(), "Should find ancestor package");
        let found_path = result.unwrap();
        assert!(
            found_path.ends_with("my_package"),
            "Should return package root"
        );
        assert!(found_path.is_absolute(), "Should return absolute path");
    }

    #[test]
    fn test_ancestor_package_detection_with_manifest_xml() {
        use tempfile::TempDir;

        let tmpdir = TempDir::new().expect("Failed to create temp dir");
        let package_root = tmpdir.path().join("rosbuild_package");
        let src_dir = package_root.join("src");

        fs::create_dir_all(&src_dir).expect("Failed to create src dir");

        // Create manifest.xml (ROS1 rosbuild format)
        let manifest_xml = package_root.join("manifest.xml");
        fs::write(
            &manifest_xml,
            r#"<package name="rosbuild_package">
  <description>ROS1 test package</description>
  <author>Test</author>
  <license>BSD</license>
  <depend package="rospy"/>
</package>"#,
        )
        .expect("Failed to write manifest.xml");

        // Create a dummy xacro file
        let xacro_file = src_dir.join("robot.xacro");
        fs::write(&xacro_file, "<robot/>").expect("Failed to write xacro");

        // Test ancestor detection - should work with manifest.xml too
        let ext = FindExtension::new();
        ext.set_current_file(Some(xacro_file));

        let result = ext.find_ancestor_package("rosbuild_package");
        assert!(
            result.is_some(),
            "Should find ancestor package with manifest.xml"
        );
        let found_path = result.unwrap();
        assert!(
            found_path.ends_with("rosbuild_package"),
            "Should return package root"
        );
        assert!(found_path.is_absolute(), "Should return absolute path");
    }

    #[test]
    fn test_ancestor_package_detection_stops_at_first_boundary() {
        use tempfile::TempDir;

        let tmpdir = TempDir::new().expect("Failed to create temp dir");
        let outer_pkg = tmpdir.path().join("outer_package");
        let inner_pkg = outer_pkg.join("nested").join("inner_package");
        let urdf_dir = inner_pkg.join("urdf");

        fs::create_dir_all(&urdf_dir).expect("Failed to create urdf dir");

        // Create outer package.xml
        let outer_pkg_xml = outer_pkg.join("package.xml");
        fs::write(
            &outer_pkg_xml,
            r#"<?xml version="1.0"?>
<package><name>outer_package</name></package>"#,
        )
        .expect("Failed to write outer package.xml");

        // Create inner package.xml
        let inner_pkg_xml = inner_pkg.join("package.xml");
        fs::write(
            &inner_pkg_xml,
            r#"<?xml version="1.0"?>
<package><name>inner_package</name></package>"#,
        )
        .expect("Failed to write inner package.xml");

        // Create a dummy xacro file in inner package
        let xacro_file = urdf_dir.join("robot.xacro");
        fs::write(&xacro_file, "<robot/>").expect("Failed to write xacro");

        // Test: should find inner_package first, not outer_package
        let ext = FindExtension::new();
        ext.set_current_file(Some(xacro_file.clone()));

        let result = ext.find_ancestor_package("inner_package");
        assert!(result.is_some(), "Should find inner package");
        assert!(result.unwrap().ends_with("inner_package"));

        // Should NOT find outer_package (stops at first boundary)
        ext.set_current_file(Some(xacro_file));
        let result = ext.find_ancestor_package("outer_package");
        assert!(
            result.is_none(),
            "Should not find outer package (stops at first boundary)"
        );
    }

    #[test]
    fn test_ancestor_package_detection_whitespace_trimming() {
        use tempfile::TempDir;

        let tmpdir = TempDir::new().expect("Failed to create temp dir");
        let package_root = tmpdir.path().join("whitespace_pkg");
        let urdf_dir = package_root.join("urdf");

        fs::create_dir_all(&urdf_dir).expect("Failed to create urdf dir");

        // Create package.xml with whitespace in name element
        let package_xml = package_root.join("package.xml");
        fs::write(
            &package_xml,
            r#"<?xml version="1.0"?>
<package>
  <name>
    whitespace_pkg
  </name>
</package>"#,
        )
        .expect("Failed to write package.xml");

        let xacro_file = urdf_dir.join("robot.xacro");
        fs::write(&xacro_file, "<robot/>").expect("Failed to write xacro");

        // Test: should match despite whitespace
        let ext = FindExtension::new();
        ext.set_current_file(Some(xacro_file));

        let result = ext.find_ancestor_package("whitespace_pkg");
        assert!(
            result.is_some(),
            "Should find package despite whitespace in name"
        );
    }

    #[test]
    fn test_on_file_change_sets_and_clears_context() {
        use tempfile::TempDir;

        let tmpdir = TempDir::new().expect("Failed to create temp dir");
        let test_file = tmpdir.path().join("test.xacro");
        fs::write(&test_file, "<robot/>").expect("Failed to write test file");

        let ext = FindExtension::new();

        // Set file context with Some
        ext.on_file_change(Some(&test_file));
        assert!(
            ext.current_file.borrow().is_some(),
            "Should set file context"
        );

        // Clear context with None
        ext.on_file_change(None);
        assert!(
            ext.current_file.borrow().is_none(),
            "Should clear context when passed None"
        );
    }
}

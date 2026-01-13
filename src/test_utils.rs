//! Test utilities shared across unit and integration tests

/// RAII guard for environment variables that automatically cleans up on drop.
///
/// This ensures env vars are removed even if tests panic, preventing test pollution.
/// Use this when tests need to temporarily set environment variables.
pub struct EnvVarGuard {
    name: String,
}

impl EnvVarGuard {
    pub fn new(
        name: impl Into<String>,
        value: &str,
    ) -> Self {
        let name = name.into();
        std::env::set_var(&name, value);
        Self { name }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(&self.name);
    }
}

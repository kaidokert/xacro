//! Test utilities shared across unit and integration tests

/// RAII guard for environment variables that automatically restores original value on drop.
///
/// This ensures env vars are restored to their original state even if tests panic,
/// preventing test pollution. Use this when tests need to temporarily set environment variables.
pub struct EnvVarGuard {
    name: String,
    prev_value: Option<String>,
}

impl EnvVarGuard {
    pub fn new(
        name: impl Into<String>,
        value: &str,
    ) -> Self {
        let name = name.into();
        let prev_value = std::env::var(&name).ok();
        std::env::set_var(&name, value);
        Self { name, prev_value }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        match &self.prev_value {
            Some(val) => std::env::set_var(&self.name, val),
            None => std::env::remove_var(&self.name),
        }
    }
}

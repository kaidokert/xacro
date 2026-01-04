use crate::error::XacroError;
use crate::utils::xml::is_xacro_element;
use core::cell::RefCell;
use log::warn;
use pyisheval::Interpreter;
use std::collections::{HashMap, HashSet};
use xmltree::{
    Element,
    XMLNode::{Element as NodeElement, Text as TextElement},
};

/// Built-in math constants (name, value) that are pre-initialized
/// Users can override these, but will receive a warning
const BUILTIN_CONSTANTS: &[(&str, f64)] = &[
    ("pi", core::f64::consts::PI),
    ("e", core::f64::consts::E),
    ("tau", core::f64::consts::TAU),
    ("M_PI", core::f64::consts::PI), // Legacy alias
    ("inf", f64::INFINITY),
    ("nan", f64::NAN),
];

pub struct PropertyProcessor<const MAX_SUBSTITUTION_DEPTH: usize = 100> {
    interpreter: Interpreter,
    // Lazy evaluation infrastructure for Python xacro compatibility
    // Store raw, unevaluated property values: "x" -> "${y * 2}"
    raw_properties: RefCell<HashMap<String, String>>,
    // Cache fully evaluated values: "x" -> "20"
    evaluated_cache: RefCell<HashMap<String, String>>,
    // Stack for circular dependency detection: ["x", "y"]
    resolution_stack: RefCell<Vec<String>>,
}

/// Helper: Check if we should skip processing this element's body
/// Macro definition bodies should NOT be processed during property collection/substitution
/// because macro parameters (like ${name}) don't exist until expansion time.
fn should_skip_macro_body(
    element: &Element,
    xacro_ns: &str,
) -> bool {
    is_xacro_element(element, "macro", xacro_ns)
}

/// Collect properties from an element tree into a provided HashMap
///
/// This is a recursive tree walker that collects `<xacro:property>` elements
/// and stores their RAW (unevaluated) values into the provided HashMap.
/// Used by macro expansion for scoped property collection.
///
/// # Arguments
/// * `element` - The XML element to process
/// * `properties` - The HashMap to populate with property name -> raw value mappings
/// * `xacro_ns` - The xacro namespace prefix
pub(crate) fn collect_properties_into_map(
    element: &Element,
    properties: &mut HashMap<String, String>,
    xacro_ns: &str,
) -> Result<(), XacroError> {
    // CRITICAL: Skip macro definition bodies
    if should_skip_macro_body(element, xacro_ns) {
        return Ok(());
    }

    // Check if this is a property element
    let is_property = element.name == "property"
        && (element.namespace.is_none() || element.namespace.as_deref() == Some(xacro_ns));

    if is_property {
        if let (Some(name), Some(value)) = (
            element.attributes.get("name"),
            element.attributes.get("value"),
        ) {
            // Store RAW value (lazy evaluation)
            properties.insert(name.clone(), value.clone());
        }
    }

    // Recursively process children
    for child in &element.children {
        if let NodeElement(child_elem) = child {
            collect_properties_into_map(child_elem, properties, xacro_ns)?;
        }
    }

    Ok(())
}

/// Remove property elements from XML tree
///
/// Recursively removes `<xacro:property>` elements from the tree after they have been
/// collected and substituted. Properties inside macro definitions are preserved since
/// they need to be available during macro expansion.
pub(crate) fn remove_property_elements(
    element: &mut Element,
    xacro_ns: &str,
) {
    element.children.retain_mut(|child| {
        if let NodeElement(child_elem) = child {
            // Remove property elements (either <property> or <xacro:property>)
            // Check namespace (not prefix) - robust against xmlns:x="..." aliasing
            let is_property = child_elem.name == "property"
                && (child_elem.namespace.is_none()
                    || child_elem.namespace.as_deref() == Some(xacro_ns));

            if is_property {
                return false;
            }

            // CRITICAL: Don't recurse into macro bodies
            // Properties inside macros need to stay until macro expansion
            if !should_skip_macro_body(child_elem, xacro_ns) {
                remove_property_elements(child_elem, xacro_ns);
            }
        }
        true
    });
}

/// Initialize math constants for property evaluation
///
/// Python xacro exposes all math module symbols for backwards compatibility.
/// This includes constants like pi, e, tau, and functions like sin, cos, etc.
/// For now, we add the most commonly used constants. Functions would require
/// extending pyisheval or the expression evaluator.
fn init_math_constants(properties: &mut HashMap<String, String>) {
    properties.extend(
        BUILTIN_CONSTANTS
            .iter()
            .map(|(name, value)| (name.to_string(), value.to_string())),
    );
}

/// Check if a name is a Python keyword or built-in that shouldn't be treated as a property
fn is_python_keyword(name: &str) -> bool {
    matches!(
        name,
        // Python keywords
        "True" | "False" | "None" |
        "and" | "or" | "not" | "is" | "in" |
        "if" | "else" | "elif" |
        "for" | "while" |
        "lambda" |
        "def" | "class" | "return" | "yield" |
        "try" | "except" | "finally" | "raise" |
        "with" | "as" |
        "import" | "from" |
        "pass" | "break" | "continue" |
        "global" | "nonlocal" |
        "assert" | "del" |
        // Common built-in functions that pyisheval supports
        "abs" | "min" | "max" | "sum" | "len" | "range" |
        "int" | "float" | "str" | "bool" | "list" | "tuple" | "dict" |
        "sin" | "cos" | "tan" | "sqrt" | "radians" | "degrees"
    )
}

impl<const MAX_SUBSTITUTION_DEPTH: usize> PropertyProcessor<MAX_SUBSTITUTION_DEPTH> {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            interpreter: Interpreter::new(),
            raw_properties: RefCell::new(HashMap::new()),
            evaluated_cache: RefCell::new(HashMap::new()),
            resolution_stack: RefCell::new(Vec::new()),
        }
    }

    /// Process properties in XML tree, returning both the processed tree and the properties map
    ///
    /// CRITICAL: Returns (Element, HashMap) so that properties can be passed to subsequent
    /// processors (like ConditionProcessor) that need them for expression evaluation.
    ///
    /// Uses lazy evaluation to match Python xacro behavior:
    /// 1. Collect properties - store RAW values without evaluation in struct field
    /// 2. Resolve properties - lazily evaluate all properties with circular dependency detection
    /// 3. Substitute - replace ${...} expressions in XML with resolved values
    /// 4. Remove property elements from XML
    ///
    /// This approach allows:
    /// - Forward property references (A can reference B even if B is defined later)
    /// - Unused properties with undefined variables (only error when accessed)
    /// - Proper circular dependency detection
    pub fn process(
        &self,
        mut xml: Element,
        xacro_ns: &str,
    ) -> Result<(Element, HashMap<String, String>), XacroError> {
        // Clear state from any previous processing
        self.raw_properties.borrow_mut().clear();
        self.evaluated_cache.borrow_mut().clear();
        self.resolution_stack.borrow_mut().clear();

        // Initialize built-in math constants (as raw strings for lazy evaluation)
        init_math_constants(&mut self.raw_properties.borrow_mut());

        // Step 1: Collect properties (stores RAW values in struct field)
        self.collect_properties(&xml, xacro_ns)?;

        // Step 2: Resolve all properties lazily (with circular dependency detection)
        // This is where forward references are resolved and circular dependencies are caught
        let resolved_properties = self.resolve_all_properties()?;

        // Step 3: Remove property elements BEFORE substitution
        // CRITICAL: This prevents substitute_properties from trying to evaluate
        // the value attributes of property definitions themselves!
        // We've already collected them into resolved_properties, so the elements
        // are no longer needed and removing them avoids "double evaluation"
        remove_property_elements(&mut xml, xacro_ns);

        // Step 4: Substitute using resolved properties in the remaining tree
        // Now the tree only contains actual content (links, joints, etc.), not definitions
        self.substitute_properties(&mut xml, &resolved_properties, xacro_ns)?;

        Ok((xml, resolved_properties)) // Return both Element and resolved properties!
    }

    /// Collect properties into struct field (used by main processing)
    pub(crate) fn collect_properties(
        &self,
        element: &Element,
        xacro_ns: &str,
    ) -> Result<(), XacroError> {
        // CRITICAL: Skip macro definition bodies
        // Macro parameters don't exist during definition, only during expansion
        if should_skip_macro_body(element, xacro_ns) {
            return Ok(()); // Don't recurse into macro bodies
        }

        // Check if this is a property element (either <property> or <xacro:property>)
        // Check namespace (not prefix) - robust against xmlns:x="..." aliasing
        let is_property = element.name == "property"
            && (element.namespace.is_none() || element.namespace.as_deref() == Some(xacro_ns));

        if is_property {
            if let (Some(name), Some(value)) = (
                element.attributes.get("name"),
                element.attributes.get("value"),
            ) {
                // Warn if user is overriding a built-in constant
                if BUILTIN_CONSTANTS.iter().any(|(k, _)| *k == name.as_str()) {
                    warn!(
                        "Property '{}' overrides built-in math constant. \
                         This may cause unexpected behavior. \
                         Consider using a different name.",
                        name
                    );
                }

                // LAZY EVALUATION: Store RAW value in struct field, don't evaluate yet
                // Python xacro behavior: properties are evaluated only when accessed
                // This allows forward references and avoids errors for unused properties
                self.raw_properties
                    .borrow_mut()
                    .insert(name.clone(), value.clone());
            }
        }

        for child in &element.children {
            if let NodeElement(child_elem) = child {
                self.collect_properties(child_elem, xacro_ns)?;
            }
        }

        Ok(())
    }

    pub(crate) fn substitute_properties(
        &self,
        element: &mut Element,
        properties: &HashMap<String, String>,
        xacro_ns: &str,
    ) -> Result<(), XacroError> {
        // CRITICAL: Skip macro definition bodies
        // Macro parameters (like ${name}, ${size}) don't exist during definition
        // They'll be substituted by MacroProcessor during expansion
        if should_skip_macro_body(element, xacro_ns) {
            return Ok(()); // Don't recurse into macro bodies
        }

        // CRITICAL: Check if this is a conditional element (xacro:if or xacro:unless)
        // We must NOT substitute the 'value' attribute on conditionals because:
        // 1. ConditionProcessor needs the raw expression like "${3*0.1}"
        // 2. If we substitute, it becomes "0.3" string, losing type information
        // 3. This breaks float truthiness in eval_boolean
        let is_conditional = is_xacro_element(element, "if", xacro_ns)
            || is_xacro_element(element, "unless", xacro_ns);

        // Process attributes
        for (key, value) in element.attributes.iter_mut() {
            // Skip 'value' attribute on conditionals - preserve raw expression
            if is_conditional && key == "value" {
                continue; // Keep original expression unchanged
            }
            // Normal substitution for all other attributes
            *value = self.substitute_in_text(value, properties)?;
        }

        // Recurse into children
        for child in &mut element.children {
            if let NodeElement(child_elem) = child {
                self.substitute_properties(child_elem, properties, xacro_ns)?;
            } else if let TextElement(text) = child {
                *text = self.substitute_in_text(text, properties)?;
            }
        }

        Ok(())
    }

    pub(crate) fn substitute_in_text(
        &self,
        text: &str,
        properties: &HashMap<String, String>,
    ) -> Result<String, XacroError> {
        use crate::utils::eval::eval_text_with_interpreter;

        // Iterative substitution: keep evaluating as long as ${...} expressions remain
        // This handles cases where property values themselves contain expressions
        // Example: full_name = "link_${name}" where name = "base" → "link_base"
        let mut result = text.to_string();
        let mut iteration = 0;

        while result.contains("${") && iteration < MAX_SUBSTITUTION_DEPTH {
            let next = eval_text_with_interpreter(&result, properties, &self.interpreter).map_err(
                |e| XacroError::EvalError {
                    expr: match &e {
                        crate::utils::eval::EvalError::PyishEval { expr, .. } => expr.clone(),
                        crate::utils::eval::EvalError::InvalidBoolean { condition, .. } => {
                            condition.clone()
                        }
                    },
                    source: e,
                },
            )?;

            // If result didn't change, we're done (avoids infinite loop on unresolvable expressions)
            if next == result {
                break;
            }

            result = next;
            iteration += 1;
        }

        // If we hit the limit with remaining placeholders, this indicates a problem
        if iteration >= MAX_SUBSTITUTION_DEPTH && result.contains("${") {
            let snippet = if result.len() > 100 {
                format!("{}...", &result[..100])
            } else {
                result.clone()
            };
            return Err(XacroError::MaxSubstitutionDepth {
                depth: MAX_SUBSTITUTION_DEPTH,
                snippet,
            });
        }

        Ok(result)
    }

    /// Lazy property resolution with circular dependency detection
    ///
    /// Python xacro stores property values as raw strings and evaluates them only when accessed.
    /// This allows forward references (property A can reference B even if B is defined later)
    /// and avoids errors for unused properties with undefined variables.
    ///
    /// This method implements the same lazy evaluation strategy (from PHASE_X_PLAN.md):
    /// 1. Check cache - if already resolved, return cached value
    /// 2. Check circular dependency - if currently resolving, error
    /// 3. Get raw value from raw_properties field
    /// 4. Mark active (push to resolution stack)
    /// 5. Dependency Discovery & Resolution - recursively resolve all vars in expression
    /// 6. Evaluate using context with resolved dependencies
    /// 7. Unmark & Cache result
    fn resolve_property(
        &self,
        name: &str,
    ) -> Result<String, XacroError> {
        // 1. Check cache
        if let Some(cached) = self.evaluated_cache.borrow().get(name) {
            return Ok(cached.clone());
        }

        // 2. Check circular dependency
        if self.resolution_stack.borrow().contains(&name.to_string()) {
            let chain = self.resolution_stack.borrow().join(" -> ");
            return Err(XacroError::CircularPropertyDependency {
                chain: format!("{} -> {}", chain, name),
            });
        }

        // 3. Get raw value from struct field
        let raw_value = self
            .raw_properties
            .borrow()
            .get(name)
            .cloned()
            .ok_or_else(|| XacroError::UndefinedProperty(name.to_string()))?;

        // 4. Mark active
        self.resolution_stack.borrow_mut().push(name.to_string());

        // 5-6. Dependency Discovery, Resolution & Evaluation
        // Use a closure to ensure stack cleanup happens on all paths (success or error)
        let result = (|| {
            // Parse expression to find all variable references, then recursively resolve them
            let eval_context = self.build_eval_context(&raw_value)?;
            // Evaluate using the context (all dependencies are now resolved)
            self.substitute_in_text(&raw_value, &eval_context)
        })();

        // 7. Unmark (always, regardless of success/failure)
        self.resolution_stack.borrow_mut().pop();

        // Propagate error or cache success
        let evaluated = result?;
        self.evaluated_cache
            .borrow_mut()
            .insert(name.to_string(), evaluated.clone());

        Ok(evaluated)
    }

    /// Build evaluation context for a property value
    ///
    /// Extracts all property references from the value, then recursively resolves them.
    /// Returns a HashMap containing only the resolved properties needed to evaluate the value.
    ///
    /// Since resolve_property is cached, we don't need to preload all cached properties.
    /// We only add the properties actually referenced in this value.
    fn build_eval_context(
        &self,
        value: &str,
    ) -> Result<HashMap<String, String>, XacroError> {
        let mut context = HashMap::new();
        let referenced_props = self.extract_property_references(value);

        for prop_name in referenced_props {
            let resolved = self.resolve_property(&prop_name)?;
            context.insert(prop_name, resolved);
        }

        Ok(context)
    }

    /// Extract property names referenced in a value string using lexical scanning
    ///
    /// Strategy:
    /// 1. Use our Lexer to find ${...} blocks in the raw string
    /// 2. Use regex to find identifiers that look like variables (not in strings/numbers)
    /// 3. Return all found variable names
    ///
    /// Note: Originally planned to use pyisheval AST parsing (PHASE_X_PLAN.md), but
    /// pyisheval 0.9.0 doesn't expose parser/AST modules publicly. This regex approach
    /// handles the common cases and is simpler. It may over-capture in some edge cases,
    /// but that's safe (we'll get proper errors during evaluation if truly undefined).
    fn extract_property_references(
        &self,
        value: &str,
    ) -> HashSet<String> {
        use crate::utils::lexer::{Lexer, TokenType};
        use regex::Regex;

        let mut refs = HashSet::new();
        let lexer = Lexer::new(value);

        // Regex to find identifiers in expressions
        // Matches: variable names (letters/underscore followed by letters/digits/underscore)
        // But NOT when preceded by a dot (e.g., obj.method) or inside strings
        let var_regex = Regex::new(r"\b([a-zA-Z_][a-zA-Z0-9_]*)\b").expect("Valid regex pattern");

        for (token_type, token_value) in lexer {
            if token_type == TokenType::Expr {
                // Extract all identifier-like tokens from the expression
                for cap in var_regex.captures_iter(&token_value) {
                    if let Some(name_match) = cap.get(1) {
                        let name = name_match.as_str();
                        let match_end = name_match.end();

                        // Skip if this is a function call (identifier followed by '(')
                        // Use defensive slicing to be robust against future regex changes
                        let is_function_call = token_value
                            .get(match_end..)
                            .map(|s| s.trim_start().starts_with('('))
                            .unwrap_or(false);

                        // Filter out Python keywords, built-in functions, and function calls
                        // (over-capturing is safe, under-capturing breaks things)
                        if !is_python_keyword(name) && !is_function_call {
                            refs.insert(name.to_string());
                        }
                    }
                }
            }
        }

        refs
    }

    /// Resolve all properties lazily
    ///
    /// Iterates through all properties in raw_properties field and resolves them.
    /// Properties are resolved on-demand using the resolution cache.
    ///
    /// IMPORTANT: This tries to resolve all properties, but skips properties that
    /// cannot be resolved (e.g., properties with undefined variables). This matches
    /// Python xacro's behavior where unused properties with undefined variables
    /// don't cause errors.
    ///
    /// Returns a new HashMap with all resolved values.
    fn resolve_all_properties(&self) -> Result<HashMap<String, String>, XacroError> {
        let mut resolved = HashMap::new();

        // Get snapshot of property names to iterate over
        let prop_names: Vec<String> = self.raw_properties.borrow().keys().cloned().collect();

        // Try to resolve each property
        // If resolution fails, skip it (don't add to resolved map)
        // This allows unused properties with undefined variables to exist without error
        // If they're actually used during substitution, the error will occur then
        for name in prop_names {
            match self.resolve_property(&name) {
                Ok(value) => {
                    log::trace!("Property '{}' resolved to: {}", name, value);
                    resolved.insert(name, value);
                }
                Err(e) => {
                    log::debug!("Property '{}' resolution failed: {:?}", name, e);

                    // Check if this is a circular dependency - those should propagate immediately
                    if matches!(e, XacroError::CircularPropertyDependency { .. }) {
                        log::debug!("  -> Circular dependency, propagating error");
                        return Err(e);
                    }

                    // For other errors (undefined vars, eval errors), skip this property
                    // If it's used later during substitution, the error will surface then
                    log::debug!("  -> Skipping unresolvable property");
                }
            }
        }

        Ok(resolved)
    }
}

#[cfg(test)]
mod tests;

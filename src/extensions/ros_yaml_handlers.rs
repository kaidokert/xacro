//! ROS-specific YAML tag handlers for unit conversions
//!
//! Implements handlers for ROS conventions like `!degrees`, `!radians`, `!millimeters`, etc.
//! These match the behavior of Python xacro's ConstructUnits class.

use crate::eval::yaml_tag_handler::YamlTagHandler;

/// ROS unit conversion YAML tag handler
///
/// Handles standard ROS unit tags:
/// - Angle units: `!degrees`, `!radians`
/// - Length units: `!meters`, `!millimeters`, `!foot`, `!inches`
///
/// Values can be:
/// - Numeric literals: `!degrees 90.0` → `"1.5707963267948966"`
/// - Python expressions: `!degrees 45*2` → `"(45*2)*0.017453292519943295"`
///
/// This matches Python xacro's ConstructUnits behavior.
#[derive(Debug, Clone)]
pub struct RosUnitTagHandler;

impl RosUnitTagHandler {
    /// Create a new ROS unit tag handler
    pub fn new() -> Self {
        Self
    }

    /// Get the conversion factor for a given unit tag
    ///
    /// Returns `Some(factor)` for known units, `None` for unknown tags
    fn get_conversion_factor(tag: &str) -> Option<f64> {
        match tag {
            // Angle units
            "radians" => Some(1.0),
            "degrees" => Some(core::f64::consts::PI / 180.0),
            // Length units
            "meters" => Some(1.0),
            "millimeters" => Some(0.001),
            "foot" => Some(0.3048),
            "inches" => Some(0.0254),
            _ => None,
        }
    }
}

impl Default for RosUnitTagHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl YamlTagHandler for RosUnitTagHandler {
    fn handle_tag(
        &self,
        tag: &str,
        raw_value: &str,
    ) -> Option<String> {
        let conversion_factor = Self::get_conversion_factor(tag)?;

        let trimmed = raw_value.trim();

        // Try parsing as numeric literal first
        if let Ok(num) = trimmed.parse::<f64>() {
            let result = num * conversion_factor;
            return Some(result.to_string());
        }

        // Treat as Python expression
        // If conversion factor is 1.0, no need to wrap
        // Direct equality is safe here since conversion_factor comes from literal constants
        if conversion_factor == 1.0 {
            Some(trimmed.to_string())
        } else {
            // Wrap expression with conversion factor
            Some(format!("({})*{}", trimmed, conversion_factor))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_degrees_numeric() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("degrees", "90.0");
        assert!(result.is_some());
        let value: f64 = result.unwrap().parse().expect("Should be numeric");
        assert!((value - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
    }

    #[test]
    fn test_degrees_expression() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("degrees", "45*2");
        assert_eq!(
            result,
            Some(format!("(45*2)*{}", std::f64::consts::PI / 180.0))
        );
    }

    #[test]
    fn test_radians_passthrough() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("radians", "1.57");
        assert_eq!(result, Some("1.57".to_string()));
    }

    #[test]
    fn test_millimeters_numeric() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("millimeters", "1000");
        assert_eq!(result, Some("1".to_string()));
    }

    #[test]
    fn test_millimeters_expression() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("millimeters", "500*2");
        assert_eq!(result, Some("(500*2)*0.001".to_string()));
    }

    #[test]
    fn test_unknown_tag() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("unknown", "42");
        assert_eq!(result, None);
    }

    #[test]
    fn test_foot_to_meters() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("foot", "1");
        assert_eq!(result, Some("0.3048".to_string()));
    }

    #[test]
    fn test_inches_to_meters() {
        let handler = RosUnitTagHandler::new();
        let result = handler.handle_tag("inches", "12");
        // 12 inches * 0.0254 m/inch = 0.3048 m (floating point may have precision issues)
        assert!(result.is_some());
        let value: f64 = result.unwrap().parse().expect("Should be numeric");
        assert!((value - 0.3048).abs() < 1e-10);
    }
}

# Known Issues and Limitations

This document tracks known deviations from the official Python `xacro` implementation.

## Output Formatting
- **XML Semantics:** The XML output matches Python xacro semantically but not verbatim.
  - Non-significant whitespace differences (attribute ordering, newlines).
  - Namespace declarations may be placed on child elements instead of the root.
- **Float Formatting:** Floating point values are generally formatted as `1.0` or `0.5`, but Python xacro sometimes switches to scientific notation (`1e-05`) or preserves integer formatting (`1` vs `1.0`) differently in edge cases.
- **Boolean Formatting:** `xacro-rs` uses strict numeric types for evaluation. Metadata heuristics attempt to preserve "True"/"False" formatting, but this is fragile. It fails for values derived from `$(arg ...)` or property concatenation, resulting in `1`/`0` output.

## Feature Gaps
- **Dynamic XML:** The `<xacro:element>` and `<xacro:attribute>` directives are not supported.
- **Python features:**
  - The `in` operator is not supported (e.g. `${x in list}`, `${x not in list}`).
  - Ternary operator `x if condition else y` is not supported.
  - String formatting operator `%` is not supported (e.g. `'%05d' % id`).
  - Complex Python types like `set()` and `dict()` are not fully supported.

## Technical Limitations
- **Math Precision:** All numbers are treated as `f64` (double precision float). Python's arbitrary precision integers are not supported.
- **Lambda Scope:** Due to evaluator limitations, the `nan` constant cannot be referenced inside lambda functions (e.g. `lambda x: x if x else nan` will fail).

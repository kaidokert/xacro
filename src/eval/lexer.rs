use regex::Regex;
use std::sync::OnceLock;

static DOLLAR_DOLLAR_BRACE_REGEX: OnceLock<Regex> = OnceLock::new();
static EXPR_REGEX: OnceLock<Regex> = OnceLock::new();
static EXTENSION_REGEX: OnceLock<Regex> = OnceLock::new();
static TEXT_REGEX: OnceLock<Regex> = OnceLock::new();

fn get_dollar_dollar_brace_regex() -> &'static Regex {
    DOLLAR_DOLLAR_BRACE_REGEX.get_or_init(|| Regex::new(r"^\$\$+(\{|\()").unwrap())
}

fn get_expr_regex() -> &'static Regex {
    EXPR_REGEX.get_or_init(|| Regex::new(r"^\$\{[^\}]*\}").unwrap())
}

fn get_extension_regex() -> &'static Regex {
    EXTENSION_REGEX.get_or_init(|| Regex::new(r"^\$\([^\)]*\)").unwrap())
}

fn get_text_regex() -> &'static Regex {
    // Must be anchored with ^ to prevent matching at non-zero offsets
    // This prevents silent data loss when other regexes don't match
    //
    // Pattern breakdown:
    // - [^$]+ : One or more non-dollar characters
    // - \$ : A single dollar sign
    //
    // This matches everything that isn't specifically handled by other tokens.
    // Since regexes are tried in priority order (DOLLAR_DOLLAR_BRACE, EXPR, EXTENSION, then TEXT),
    // special sequences like ${, $(, and $${ are handled before TEXT is tried.
    // The \$ alternative matches any remaining dollars (like those in "$$x" after $${ fails to match).
    TEXT_REGEX.get_or_init(|| Regex::new(r"^([^$]+|\$)").unwrap())
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum TokenType {
    DollarDollarBrace,
    Expr,
    Extension,
    Text,
}

pub(crate) struct Lexer<'a> {
    input_str: &'a str,
    regexes: Vec<(TokenType, &'static Regex)>,
    position: usize,
}

impl<'a> Lexer<'a> {
    pub fn new(input_str: &'a str) -> Self {
        Lexer {
            input_str,
            regexes: vec![
                (
                    TokenType::DollarDollarBrace,
                    get_dollar_dollar_brace_regex(),
                ),
                (TokenType::Expr, get_expr_regex()),
                (TokenType::Extension, get_extension_regex()),
                (TokenType::Text, get_text_regex()),
            ],
            position: 0,
        }
    }
}

impl<'a> Iterator for Lexer<'a> {
    type Item = (TokenType, String);

    fn next(&mut self) -> Option<Self::Item> {
        // End of input - normal termination
        if self.position >= self.input_str.len() {
            return None;
        }

        // Special handling for Extension tokens to support nested parentheses
        // Extension regex can't handle $(foo $(bar)) - it stops at first )
        // Use find_matching_paren instead for proper nesting support
        let remaining = &self.input_str[self.position..];
        if remaining.starts_with("$(") {
            // Find matching closing paren using depth counting
            if let Some(close_pos) = crate::eval::interpreter::find_matching_paren(remaining, 1) {
                // Extract content between $( and )
                let content = &remaining[2..close_pos];
                // Advance position past the entire $(...) construct
                self.position += close_pos + 1;
                return Some((TokenType::Extension, content.to_string()));
            }
            // If no matching paren found, fall through to regex handling
            // which will treat it as malformed ($ as Text, then ( as Text)
        }

        for (token_type, regex) in self.regexes.iter() {
            if let Some(captures) = regex.captures(&self.input_str[self.position..]) {
                if let Some(m) = captures.get(0) {
                    // Safety check: All regexes are anchored with ^, so match must start at 0
                    // This should never fail, but validates our regex assumptions
                    debug_assert_eq!(
                        m.start(),
                        0,
                        "Regex matched at offset {} instead of 0 - regex must be anchored with ^",
                        m.start()
                    );

                    // Advance position by match length
                    self.position += m.end();
                    let matched_text = m.as_str();

                    // Extract the token value based on type
                    let token_value = match token_type {
                        TokenType::DollarDollarBrace => {
                            // For $${ or $$(, we want just the final character ({or ()
                            // The regex captures it in group 1
                            if let Some(capture) = captures.get(1) {
                                capture.as_str()
                            } else {
                                // Fallback: extract last character
                                &matched_text[matched_text.len() - 1..]
                            }
                        }
                        TokenType::Expr => {
                            // Strip ${ and } to get just the expression
                            &matched_text[2..matched_text.len() - 1]
                        }
                        TokenType::Extension => {
                            // Strip $( and ) to get just the extension
                            &matched_text[2..matched_text.len() - 1]
                        }
                        TokenType::Text => matched_text,
                    };

                    return Some((*token_type, token_value.to_string()));
                }
            }
        }

        // If we reach here, no regex matched - this should be impossible
        // with properly designed regexes (TEXT_REGEX should match everything else)
        unreachable!(
            "Lexer error: no regex matched at position {} with remaining input: {:?}",
            self.position,
            &self.input_str[self.position..]
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper to collect all tokens from a lexer
    fn lex_all(input: &str) -> Vec<(TokenType, String)> {
        Lexer::new(input).collect()
    }

    /// Helper to reconstruct input from tokens (for testing completeness)
    fn reconstruct(tokens: &[(TokenType, String)]) -> String {
        let mut result = String::new();
        for (token_type, value) in tokens {
            match token_type {
                TokenType::Text => result.push_str(value),
                TokenType::Expr => {
                    result.push_str("${");
                    result.push_str(value);
                    result.push('}');
                }
                TokenType::Extension => {
                    result.push_str("$(");
                    result.push_str(value);
                    result.push(')');
                }
                TokenType::DollarDollarBrace => {
                    result.push('$');
                    result.push_str(value);
                }
            }
        }
        result
    }

    #[test]
    fn test_lexer_basic() {
        let input = "hello ${world}!";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![
                (TokenType::Text, "hello ".to_string()),
                (TokenType::Expr, "world".to_string()),
                (TokenType::Text, "!".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_multiple_expressions() {
        let input = "${a} and ${b}";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![
                (TokenType::Expr, "a".to_string()),
                (TokenType::Text, " and ".to_string()),
                (TokenType::Expr, "b".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_extension() {
        let input = "$(extension)";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![(TokenType::Extension, "extension".to_string()),]
        );
    }

    #[test]
    fn test_lexer_escape_double_dollar_brace() {
        let input = "$${expr}";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![
                (TokenType::DollarDollarBrace, "{".to_string()),
                (TokenType::Text, "expr}".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_escape_double_dollar_paren() {
        let input = "$$(extension)";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![
                (TokenType::DollarDollarBrace, "(".to_string()),
                (TokenType::Text, "extension)".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_plain_text() {
        let input = "just plain text";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![(TokenType::Text, "just plain text".to_string()),]
        );
    }

    #[test]
    fn test_lexer_empty_string() {
        let input = "";
        let tokens = lex_all(input);
        assert_eq!(tokens, Vec::<(TokenType, String)>::new());
    }

    // ====== EDGE CASES - MALFORMED INPUT ======

    #[test]
    fn test_lexer_unclosed_expression() {
        let input = "${unclosed";
        let tokens = lex_all(input);
        // Unclosed expr: $ captured separately, then {unclosed as text
        assert_eq!(
            tokens,
            vec![
                (TokenType::Text, "$".to_string()),
                (TokenType::Text, "{unclosed".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_unclosed_extension() {
        let input = "$(unclosed";
        let tokens = lex_all(input);
        // Unclosed extension: $ captured separately, then (unclosed as text
        assert_eq!(
            tokens,
            vec![
                (TokenType::Text, "$".to_string()),
                (TokenType::Text, "(unclosed".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_single_dollar() {
        let input = "$";
        let tokens = lex_all(input);
        // Single dollar at end should match TEXT_REGEX (\$$)
        assert_eq!(tokens, vec![(TokenType::Text, "$".to_string()),]);
    }

    #[test]
    fn test_lexer_dollar_in_text() {
        let input = "text $ more text";
        let tokens = lex_all(input);
        // TEXT_REGEX matches non-$ chars, then single $, then more non-$ chars
        assert_eq!(
            tokens,
            vec![
                (TokenType::Text, "text ".to_string()),
                (TokenType::Text, "$".to_string()),
                (TokenType::Text, " more text".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_dollar_unknown() {
        let input = "$unknown";
        let tokens = lex_all(input);
        // Single $ followed by text - captured as separate tokens
        assert_eq!(
            tokens,
            vec![
                (TokenType::Text, "$".to_string()),
                (TokenType::Text, "unknown".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_double_dollar_edge() {
        let input = "$$x";
        let tokens = lex_all(input);

        // With TEXT_REGEX matching single $, we get: $, $, x as separate tokens
        assert_eq!(
            tokens,
            vec![
                (TokenType::Text, "$".to_string()),
                (TokenType::Text, "$".to_string()),
                (TokenType::Text, "x".to_string()),
            ]
        );

        // Verify no truncation via reconstruction
        let reconstructed = reconstruct(&tokens);
        assert_eq!(
            reconstructed, input,
            "Lexer truncated input: '{}' became '{}'",
            input, reconstructed
        );
    }

    #[test]
    fn test_lexer_triple_dollar() {
        let input = "$$$${expr}";
        let tokens = lex_all(input);

        // DOLLAR_DOLLAR_BRACE regex ^\$\$+(\{|\() matches ALL leading dollars + brace
        // So "$$$$" matches, capture group gets "{", leaving "expr}" as text
        // This is a known limitation - multiple $$ are treated as single escape
        assert_eq!(
            tokens,
            vec![
                (TokenType::DollarDollarBrace, "{".to_string()),
                (TokenType::Text, "expr}".to_string()),
            ]
        );

        // NOTE: Reconstruction will NOT match input due to $$ handling
        // This is acceptable as "$$$$" is not standard xacro syntax
        // Standard usage is "$${" for escaping, not "$$$${}"
    }

    #[test]
    fn test_lexer_dollar_at_end() {
        let input = "text$";
        let tokens = lex_all(input);
        // TEXT_REGEX matches text, then single $ separately
        assert_eq!(
            tokens,
            vec![
                (TokenType::Text, "text".to_string()),
                (TokenType::Text, "$".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_empty_expression() {
        let input = "${}";
        let tokens = lex_all(input);
        // Empty expression should still match EXPR_REGEX
        assert_eq!(tokens, vec![(TokenType::Expr, "".to_string()),]);
    }

    #[test]
    fn test_lexer_empty_extension() {
        let input = "$()";
        let tokens = lex_all(input);
        // Empty extension should still match EXTENSION_REGEX
        assert_eq!(tokens, vec![(TokenType::Extension, "".to_string()),]);
    }

    #[test]
    fn test_lexer_extension_with_spaces() {
        // Spaces inside extension content are preserved (not trimmed by lexer)
        let input = "$( arg  foo )";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![(TokenType::Extension, " arg  foo ".to_string()),]
        );
    }

    #[test]
    fn test_lexer_mixed_expression_and_extension() {
        let input = "${x} and $(arg y)";
        let tokens = lex_all(input);
        assert_eq!(
            tokens,
            vec![
                (TokenType::Expr, "x".to_string()),
                (TokenType::Text, " and ".to_string()),
                (TokenType::Extension, "arg y".to_string()),
            ]
        );
    }

    #[test]
    fn test_lexer_nested_extension_in_expression() {
        // Extension nested inside expression - both preserved
        let input = "${$(arg x) * 2}";
        let tokens = lex_all(input);
        assert_eq!(tokens, vec![(TokenType::Expr, "$(arg x) * 2".to_string()),]);
        // The extension is preserved as part of the expression content
        // It will be resolved when the expression is evaluated
    }

    #[test]
    fn test_lexer_complex_expression() {
        let input = "${1 + 2 * 3}";
        let tokens = lex_all(input);
        assert_eq!(tokens, vec![(TokenType::Expr, "1 + 2 * 3".to_string()),]);
    }

    #[test]
    fn test_lexer_nested_braces_in_expression() {
        // EXPR_REGEX uses [^\}]* which stops at first }
        // This is correct for non-nested expressions
        let input = "${dict[key]}";
        let tokens = lex_all(input);
        // Matches ${dict[key]} - the ] is kept because only } terminates
        assert_eq!(tokens, vec![(TokenType::Expr, "dict[key]".to_string()),]);
        // Note: This correctly captures the full expression including ]
    }

    // ====== COMPLETENESS TESTS ======
    // These tests verify that lexer doesn't silently drop input

    #[test]
    fn test_lexer_completeness_all_cases() {
        let test_cases = vec![
            "hello ${world}!",
            "${a} and ${b}",
            "$(extension)",
            // NOTE: "$${expr}" is excluded - it's an escape sequence ($${ → ${)
            // NOTE: "$$$${expr}" is excluded - known limitation with multiple $$
            "plain text",
            "$",
            "text $ more",
            "$unknown",
            "$$x",
            "text$",
            "${}",
            "$()",
        ];

        for input in test_cases {
            let tokens = lex_all(input);
            let reconstructed = reconstruct(&tokens);
            assert_eq!(
                reconstructed, input,
                "Lexer completeness failed for input: '{}' -> tokens: {:?}",
                input, tokens
            );
        }
    }

    #[test]
    fn test_lexer_extension_with_quoted_paren() {
        // This tests the LEXER's ability to handle parentheses inside quoted strings
        // within extension syntax: $(command "arg_with_)_paren")
        // The lexer should NOT treat the ) inside the quotes as the extension terminator

        let input = r#"$(command "arg_with_)_paren")"#;
        let tokens: Vec<(TokenType, String)> = Lexer::new(input).collect();

        // Should get ONE Extension token with content: command "arg_with_)_paren"
        assert_eq!(tokens.len(), 1, "Should parse as single extension token");
        assert!(matches!(tokens[0].0, TokenType::Extension));
        assert_eq!(
            tokens[0].1, r#"command "arg_with_)_paren""#,
            "Extension content should include the full quoted string with paren"
        );
    }

    #[test]
    fn test_lexer_extension_with_single_quoted_paren() {
        // Test with single quotes
        let input = r#"$(command 'arg_with_)_paren')"#;
        let tokens: Vec<(TokenType, String)> = Lexer::new(input).collect();

        assert_eq!(tokens.len(), 1, "Should parse as single extension token");
        assert!(matches!(tokens[0].0, TokenType::Extension));
        assert_eq!(tokens[0].1, r#"command 'arg_with_)_paren'"#);
    }
}

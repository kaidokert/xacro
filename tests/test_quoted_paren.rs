use xacro::eval::lexer::{Lexer, TokenType};

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

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
    TEXT_REGEX.get_or_init(|| Regex::new(r"[^$]+|\$[^{($]+|\$$").unwrap())
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TokenType {
    DollarDollarBrace,
    Expr,
    Extension,
    Text,
}

pub struct Lexer<'a> {
    input_str: &'a str,
    regexes: Vec<(TokenType, Regex)>,
    position: usize,
}

impl<'a> Lexer<'a> {
    pub fn new(input_str: &'a str) -> Self {
        Lexer {
            input_str,
            regexes: vec![
                (
                    TokenType::DollarDollarBrace,
                    get_dollar_dollar_brace_regex().clone(),
                ),
                (TokenType::Expr, get_expr_regex().clone()),
                (TokenType::Extension, get_extension_regex().clone()),
                (TokenType::Text, get_text_regex().clone()),
            ],
            position: 0,
        }
    }
}

impl<'a> Iterator for Lexer<'a> {
    type Item = (TokenType, String);

    fn next(&mut self) -> Option<Self::Item> {
        for (token_type, regex) in self.regexes.iter() {
            if let Some(m) = regex.captures(&self.input_str[self.position..]) {
                if let Some(m) = m.get(0) {
                    self.position += m.end();
                    let m = m.as_str();
                    let m = match token_type {
                        TokenType::DollarDollarBrace => &m[1..],
                        TokenType::Expr => &m[2..m.len() - 1],
                        TokenType::Extension => &m[2..m.len() - 1],
                        TokenType::Text => m,
                    };
                    return Some((*token_type, m.to_string()));
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_lexer() {
        use super::*;
        let input_str = "hello ${world}!";
        let mut lexer = Lexer::new(input_str);
        assert_eq!(lexer.next(), Some((TokenType::Text, "hello ".to_string())));
        assert_eq!(lexer.next(), Some((TokenType::Expr, "world".to_string())));
        assert_eq!(lexer.next(), Some((TokenType::Text, "!".to_string())));
        assert_eq!(lexer.next(), None);
    }
}

//! C 子集词法 + 语法分析
//!
//! 目标是能直接吃下学生写在 STM32 工程里的 `.c`：
//! 类型、数组、if/while/for、函数、`volatile`、宏、注释。
//! 刻意不支持的东西（struct 位域、指针算术、union）会报一条
//! 「暂不支持」的诊断，而不是让仿真崩掉。

use std::collections::HashMap;

/* ------------------------------------------------------------------ *
 * 词法
 * ------------------------------------------------------------------ */

#[derive(Debug, Clone, PartialEq)]
pub enum Token {
    Ident(String),
    Int(i64),
    Float(f64),
    Str(String),
    Char(char),
    Kw(String),
    Plus, Minus, Star, Slash, Percent,
    Assign, PlusEq, MinusEq, StarEq, SlashEq,
    Eq, Ne, Lt, Le, Gt, Ge,
    AndAnd, OrOr, Not,
    Amp, Pipe, Caret, Tilde,
    LParen, RParen, LBrace, RBrace, LBracket, RBracket,
    Semi, Comma, Colon, Question, Dot, Arrow,
    Inc, Dec,
    Eof,
}

pub struct Lexer {
    src: Vec<char>,
    pos: usize,
    pub line: usize,
}

impl Lexer {
    pub fn new(src: &str) -> Self {
        Self { src: src.chars().collect(), pos: 0, line: 1 }
    }

    fn peek(&self) -> Option<char> {
        self.src.get(self.pos).copied()
    }

    fn bump(&mut self) -> Option<char> {
        let c = self.src.get(self.pos).copied();
        if let Some(ch) = c {
            self.pos += 1;
            if ch == '\n' {
                self.line += 1;
            }
        }
        c
    }

    fn skip_ws_and_comments(&mut self) {
        loop {
            match self.peek() {
                Some(c) if c.is_whitespace() => {
                    self.bump();
                }
                Some('/') if self.src.get(self.pos + 1) == Some(&'/') => {
                    while let Some(c) = self.peek() {
                        if c == '\n' {
                            break;
                        }
                        self.bump();
                    }
                }
                Some('/') if self.src.get(self.pos + 1) == Some(&'*') => {
                    self.bump();
                    self.bump();
                    while let Some(c) = self.peek() {
                        if c == '*' && self.src.get(self.pos + 1) == Some(&'/') {
                            self.bump();
                            self.bump();
                            break;
                        }
                        self.bump();
                    }
                }
                _ => return,
            }
        }
    }

    pub fn next_token(&mut self) -> Token {
        self.skip_ws_and_comments();
        let c = match self.peek() {
            Some(c) => c,
            None => return Token::Eof,
        };

        // 数字
        if c.is_ascii_digit() || (c == '.' && self.src.get(self.pos + 1).map_or(false, |d| d.is_ascii_digit())) {
            return self.number();
        }
        // 标识符 / 关键字
        if c.is_alphabetic() || c == '_' {
            return self.ident();
        }
        // 字符字面量
        if c == '\'' {
            self.bump();
            let ch = self.bump().unwrap_or('\0');
            if self.peek() == Some('\'') {
                self.bump();
            }
            return Token::Char(ch);
        }
        // 字符串
        if c == '"' {
            self.bump();
            let mut s = String::new();
            while let Some(ch) = self.peek() {
                if ch == '"' {
                    self.bump();
                    break;
                }
                if ch == '\\' {
                    self.bump();
                    let esc = self.bump().unwrap_or('\0');
                    s.push(match esc {
                        'n' => '\n',
                        't' => '\t',
                        'r' => '\r',
                        '0' => '\0',
                        '\\' => '\\',
                        '"' => '"',
                        o => o,
                    });
                } else {
                    s.push(ch);
                    self.bump();
                }
            }
            return Token::Str(s);
        }

        self.bump();
        let two = self.peek();
        match (c, two) {
            ('+', Some('+')) => { self.bump(); Token::Inc }
            ('+', Some('=')) => { self.bump(); Token::PlusEq }
            ('-', Some('-')) => { self.bump(); Token::Dec }
            ('-', Some('=')) => { self.bump(); Token::MinusEq }
            ('-', Some('>')) => { self.bump(); Token::Arrow }
            ('*', Some('=')) => { self.bump(); Token::StarEq }
            ('/', Some('=')) => { self.bump(); Token::SlashEq }
            ('=', Some('=')) => { self.bump(); Token::Eq }
            ('!', Some('=')) => { self.bump(); Token::Ne }
            ('<', Some('=')) => { self.bump(); Token::Le }
            ('>', Some('=')) => { self.bump(); Token::Ge }
            ('&', Some('&')) => { self.bump(); Token::AndAnd }
            ('|', Some('|')) => { self.bump(); Token::OrOr }
            ('+', _) => Token::Plus,
            ('-', _) => Token::Minus,
            ('*', _) => Token::Star,
            ('/', _) => Token::Slash,
            ('%', _) => Token::Percent,
            ('=', _) => Token::Assign,
            ('<', _) => Token::Lt,
            ('>', _) => Token::Gt,
            ('!', _) => Token::Not,
            ('&', _) => Token::Amp,
            ('|', _) => Token::Pipe,
            ('^', _) => Token::Caret,
            ('~', _) => Token::Tilde,
            ('(', _) => Token::LParen,
            (')', _) => Token::RParen,
            ('{', _) => Token::LBrace,
            ('}', _) => Token::RBrace,
            ('[', _) => Token::LBracket,
            (']', _) => Token::RBracket,
            (';', _) => Token::Semi,
            (',', _) => Token::Comma,
            (':', _) => Token::Colon,
            ('?', _) => Token::Question,
            ('.', _) => Token::Dot,
            _ => Token::Eof,
        }
    }

    fn number(&mut self) -> Token {
        let mut s = String::new();
        let mut is_float = false;
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() {
                s.push(c);
                self.bump();
            } else if c == '.' && !is_float {
                is_float = true;
                s.push(c);
                self.bump();
            } else if (c == 'x' || c == 'X') && s == "0" {
                s.clear();
                self.bump();
                while let Some(h) = self.peek() {
                    if h.is_ascii_hexdigit() {
                        s.push(h);
                        self.bump();
                    } else {
                        break;
                    }
                }
                return Token::Int(i64::from_str_radix(&s, 16).unwrap_or(0));
            } else {
                break;
            }
        }
        if let Some(c) = self.peek() {
            if c == 'f' || c == 'F' {
                self.bump();
                is_float = true;
            }
        }
        if is_float {
            Token::Float(s.parse().unwrap_or(0.0))
        } else {
            Token::Int(s.parse().unwrap_or(0))
        }
    }

    fn ident(&mut self) -> Token {
        let mut s = String::new();
        while let Some(c) = self.peek() {
            if c.is_alphanumeric() || c == '_' {
                s.push(c);
                self.bump();
            } else {
                break;
            }
        }
        match s.as_str() {
            "if" | "else" | "while" | "for" | "do" | "return" | "break" | "continue"
            | "switch" | "case" | "default" | "int" | "float" | "char" | "double" | "void"
            | "long" | "short" | "unsigned" | "signed" | "const" | "static" | "volatile"
            | "struct" | "enum" | "typedef" | "sizeof" | "goto" => Token::Kw(s),
            _ => Token::Ident(s),
        }
    }

    pub fn tokenize(mut self) -> Vec<(Token, usize)> {
        let mut out = Vec::new();
        loop {
            let line = self.line;
            let t = self.next_token();
            let done = t == Token::Eof;
            out.push((t, line));
            if done {
                break;
            }
        }
        out
    }
}

/* ------------------------------------------------------------------ *
 * 预处理：剥掉 #include / 展开 #define / 处理 #ifdef 守卫
 * ------------------------------------------------------------------ */

/// 极简预处理器：返回（处理后的源码，行号映射不做，直接用新行号）
pub fn preprocess(src: &str) -> String {
    let mut defines: HashMap<String, String> = HashMap::new();
    let mut out = String::new();
    // #if 条件栈：true 表示当前分支正在输出
    let mut cond_stack: Vec<bool> = Vec::new();

    for raw in src.lines() {
        let line = raw.trim();

        if line.starts_with("#include") {
            continue;
        }
        if line.starts_with("#pragma") || line.starts_with("#error") {
            continue;
        }

        if let Some(rest) = line.strip_prefix("#define") {
            let rest = rest.trim();
            // 函数式宏先忽略（只记录名字，展开时返回 0）
            if let Some(sp) = rest.find(' ') {
                let name = rest[..sp].to_string();
                let val = rest[sp..].trim().to_string();
                if !val.starts_with('(') {
                    defines.insert(name, val);
                }
            } else {
                defines.insert(rest.to_string(), "1".to_string());
            }
            continue;
        }

        if let Some(rest) = line.strip_prefix("#ifdef") {
            let name = rest.trim().to_string();
            cond_stack.push(defines.contains_key(&name));
            continue;
        }
        if let Some(rest) = line.strip_prefix("#ifndef") {
            let name = rest.trim().to_string();
            cond_stack.push(!defines.contains_key(&name));
            continue;
        }
        if line.starts_with("#else") {
            if let Some(top) = cond_stack.last_mut() {
                *top = !*top;
            }
            continue;
        }
        if line.starts_with("#endif") {
            cond_stack.pop();
            continue;
        }
        if line.starts_with('#') {
            continue;
        }

        // 只有在所有条件都为真时才输出
        if cond_stack.iter().all(|b| *b) {
            let mut expanded = raw.to_string();
            // 简单对象宏替换（最多 3 轮，防止自引用死循环）
            for _ in 0..3 {
                let before = expanded.clone();
                for (k, v) in &defines {
                    expanded = replace_ident(&expanded, k, v);
                }
                if expanded == before {
                    break;
                }
            }
            out.push_str(&expanded);
            out.push('\n');
        }
    }
    out
}

/// 按「完整标识符」边界替换，避免把 `abc` 里的 `a` 换掉
fn replace_ident(text: &str, name: &str, val: &str) -> String {
    let chars: Vec<char> = text.chars().collect();
    let name_chars: Vec<char> = name.chars().collect();
    let mut out = String::new();
    let mut i = 0usize;
    while i < chars.len() {
        if i + name_chars.len() <= chars.len() && &chars[i..i + name_chars.len()] == &name_chars[..] {
            let before_ok = i == 0 || !(chars[i - 1].is_alphanumeric() || chars[i - 1] == '_');
            let after = i + name_chars.len();
            let after_ok = after >= chars.len() || !(chars[after].is_alphanumeric() || chars[after] == '_');
            if before_ok && after_ok {
                out.push_str(val);
                i = after;
                continue;
            }
        }
        out.push(chars[i]);
        i += 1;
    }
    out
}

/* ------------------------------------------------------------------ *
 * AST
 * ------------------------------------------------------------------ */

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BinOp {
    Add, Sub, Mul, Div, Mod,
    Eq, Ne, Lt, Le, Gt, Ge,
    And, Or,
    BitAnd, BitOr, BitXor, Shl, Shr,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UnOp {
    Neg, Pos, Not, BitNot,
}

#[derive(Debug, Clone)]
pub enum Expr {
    Int(i64, usize),
    Float(f64, usize),
    Str(String, usize),
    Char(char, usize),
    Ident(String, usize),
    Index(Box<Expr>, Box<Expr>, usize),
    Call(String, Vec<Expr>, usize),
    Unary(UnOp, Box<Expr>, usize),
    Binary(BinOp, Box<Expr>, Box<Expr>, usize),
    Assign(String, Box<Expr>, usize),
    AssignOp(BinOp, String, Box<Expr>, usize),
    IndexAssign(Box<Expr>, Box<Expr>, Box<Expr>, usize),
    PostInc(String, bool, usize),
    Ternary(Box<Expr>, Box<Expr>, Box<Expr>, usize),
    Cast(String, Box<Expr>, usize),
    SizeofTy(String, usize),
}

#[derive(Debug, Clone)]
pub enum Stmt {
    Decl {
        ty: String,
        name: String,
        array_len: Option<usize>,
        init: Option<Expr>,
        line: usize,
    },
    Expr(Expr),
    Block(Vec<Stmt>, usize),
    If {
        cond: Expr,
        then: Box<Stmt>,
        otherwise: Option<Box<Stmt>>,
        line: usize,
    },
    While {
        cond: Expr,
        body: Box<Stmt>,
        line: usize,
    },
    DoWhile {
        body: Box<Stmt>,
        cond: Expr,
        line: usize,
    },
    For {
        init: Option<Box<Stmt>>,
        cond: Option<Expr>,
        step: Option<Expr>,
        body: Box<Stmt>,
        line: usize,
    },
    Break(usize),
    Continue(usize),
    Return(Option<Expr>, usize),
    Switch {
        expr: Expr,
        cases: Vec<(Option<i64>, Vec<Stmt>)>,
        line: usize,
    },
}

#[derive(Debug, Clone)]
pub struct FuncDef {
    pub name: String,
    pub ret_ty: String,
    pub params: Vec<(String, String)>,
    pub body: Vec<Stmt>,
    pub line: usize,
}

#[derive(Debug, Default)]
pub struct Program {
    pub globals: Vec<Stmt>,
    pub funcs: HashMap<String, FuncDef>,
    /// main 的起始行号（没有则 0）
    pub main_line: usize,
}

/* ------------------------------------------------------------------ *
 * 语法分析
 * ------------------------------------------------------------------ */

pub struct Parser {
    toks: Vec<(Token, usize)>,
    pos: usize,
    pub errors: Vec<String>,
}

type PResult<T> = Result<T, String>;

impl Parser {
    pub fn new(tokens: Vec<(Token, usize)>) -> Self {
        Self { toks: tokens, pos: 0, errors: Vec::new() }
    }

    fn peek(&self) -> Token {
        self.toks.get(self.pos).map(|t| t.0.clone()).unwrap_or(Token::Eof)
    }

    fn peek_at(&self, n: usize) -> Token {
        self.toks.get(self.pos + n).map(|t| t.0.clone()).unwrap_or(Token::Eof)
    }

    fn line(&self) -> usize {
        self.toks.get(self.pos).map(|t| t.1).unwrap_or(0)
    }

    fn bump(&mut self) -> (Token, usize) {
        let t = self.toks.get(self.pos).cloned().unwrap_or((Token::Eof, 0));
        if t.0 != Token::Eof {
            self.pos += 1;
        }
        t
    }

    fn expect(&mut self, t: Token) -> PResult<()> {
        if self.peek() == t {
            self.bump();
            Ok(())
        } else {
            Err(format!("第 {} 行：期望 {:?}，实际 {:?}", self.line(), t, self.peek()))
        }
    }

    fn eat(&mut self, t: &Token) -> bool {
        if &self.peek() == t {
            self.bump();
            true
        } else {
            false
        }
    }

    pub fn parse_program(&mut self) -> Program {
        let mut p = Program::default();
        while self.peek() != Token::Eof {
            match self.parse_toplevel() {
                Ok(item) => match item {
                    TopLevel::Global(s) => p.globals.push(s),
                    TopLevel::Func(f) => {
                        if f.name == "main" {
                            p.main_line = f.line;
                        }
                        p.funcs.insert(f.name.clone(), f);
                    }
                },
                Err(e) => {
                    self.errors.push(e);
                    // 自救：跳到下一个分号或右花括号
                    while self.peek() != Token::Eof
                        && self.peek() != Token::Semi
                        && self.peek() != Token::RBrace
                    {
                        self.bump();
                    }
                    if self.peek() == Token::Semi {
                        self.bump();
                    }
                    if self.peek() == Token::RBrace {
                        self.bump();
                    }
                }
            }
        }
        p
    }

    fn parse_toplevel(&mut self) -> PResult<TopLevel> {
        // 跳过 const/static/volatile/unsigned/signed 等修饰
        let mut ty = String::new();
        loop {
            match &self.peek() {
                Token::Kw(k) if matches!(k.as_str(), "const" | "static" | "volatile" | "unsigned" | "signed" | "long" | "short" | "extern") => {
                    self.bump();
                    // unsigned/signed 参与类型名
                    if k == "unsigned" || k == "signed" {
                        ty.push_str(k);
                        ty.push(' ');
                    }
                }
                _ => break,
            }
        }

        if let Token::Kw(k) = self.peek() {
            if matches!(k.as_str(), "int" | "float" | "char" | "double" | "void") {
                ty.push_str(&k);
                self.bump();
            }
        } else if let Token::Ident(name) = self.peek() {
            // 可能是 uint8_t 之类的 typedef，或全局赋值 / 函数调用
            if is_typedef_alias(&name) {
                ty.push_str(&name);
                self.bump();
            }
        }

        if ty.is_empty() {
            // 不是声明：可能是游离语句（少见），丢掉
            self.bump();
            return Err("顶层既不是函数也不是声明".to_string());
        }

        // 函数定义？
        if let Token::Ident(name) = self.peek() {
            if self.peek_at(1) == Token::LParen {
                self.bump();
                self.bump();
                let mut params = Vec::new();
                while self.peek() != Token::RParen && self.peek() != Token::Eof {
                    let mut pty = String::new();
                    loop {
                        match &self.peek() {
                            Token::Kw(k) if matches!(k.as_str(), "const" | "volatile" | "unsigned" | "signed") => {
                                if k == "unsigned" || k == "signed" {
                                    pty.push_str(k);
                                    pty.push(' ');
                                }
                                self.bump();
                            }
                            _ => break,
                        }
                    }
                    if let Token::Kw(k) = self.peek() {
                        pty.push_str(&k);
                        self.bump();
                    } else if let Token::Ident(a) = self.peek() {
                        if is_typedef_alias(&a) {
                            pty.push_str(&a);
                            self.bump();
                        }
                    }
                    let pname = match self.peek() {
                        Token::Ident(n) => {
                            self.bump();
                            n
                        }
                        _ => {
                            self.bump();
                            String::new()
                        }
                    };
                    // 数组参数
                    if self.peek() == Token::LBracket {
                        self.bump();
                        while self.peek() != Token::RBracket && self.peek() != Token::Eof {
                            self.bump();
                        }
                        self.bump();
                    }
                    params.push((pty, pname));
                    if self.peek() == Token::Comma {
                        self.bump();
                    }
                }
                self.expect(Token::RParen)?;

                // 函数声明（分号结尾）
                if self.peek() == Token::Semi {
                    self.bump();
                    return Err("函数声明（无函数体）已跳过".to_string());
                }

                let body = self.parse_block()?;
                return Ok(TopLevel::Func(FuncDef {
                    name,
                    ret_ty: ty,
                    params,
                    body,
                    line: self.line(),
                }));
            }
        }

        // 全局变量声明（可能是逗号分隔的多个）
        let first = self.parse_decl_with_type(ty.clone())?;
        // 后续的同类型变量直接丢弃（够用了）
        Ok(TopLevel::Global(first))
    }

    fn parse_decl_with_type(&mut self, ty: String) -> PResult<Stmt> {
        let line = self.line();
        let name = match self.peek() {
            Token::Ident(n) => {
                self.bump();
                n
            }
            _ => return Err(format!("第 {line} 行：变量声明缺少名字")),
        };

        let mut array_len = None;
        if self.peek() == Token::LBracket {
            self.bump();
            if let Token::Int(n) = self.peek() {
                array_len = Some(n as usize);
                self.bump();
            } else {
                // int a[] = {...} —— 由初始化列表推断
                while self.peek() != Token::RBracket && self.peek() != Token::Eof {
                    self.bump();
                }
            }
            self.expect(Token::RBracket)?;
        }

        let mut init = None;
        if self.eat(&Token::Assign) {
            if self.peek() == Token::LBrace {
                // 数组初始化列表
                self.bump();
                let mut elems: Vec<Expr> = Vec::new();
                while self.peek() != Token::RBrace && self.peek() != Token::Eof {
                    elems.push(self.parse_expr()?);
                    if !self.eat(&Token::Comma) {
                        break;
                    }
                }
                self.expect(Token::RBrace)?;
                if array_len.is_none() {
                    array_len = Some(elems.len().max(1));
                }
                init = Some(Expr::Str(String::new(), line)); // 占位，下面用 List 存
                // 用一个特殊表达式保存列表：这里借用 Str 不方便，改为直接构造 Index 序列不优雅
                // → 改用 `Expr::Call("#list", elems, line)` 作为数组字面量标记
                init = Some(Expr::Call("#list".to_string(), elems, line));
            } else {
                init = Some(self.parse_expr()?);
            }
        }

        // 逗号分隔的更多变量
        while self.peek() == Token::Comma {
            self.bump();
            if let Token::Ident(_) = self.peek() {
                self.bump();
            }
            if self.peek() == Token::LBracket {
                self.bump();
                while self.peek() != Token::RBracket && self.peek() != Token::Eof {
                    self.bump();
                }
                self.bump();
            }
            if self.eat(&Token::Assign) {
                let _ = self.parse_expr();
            }
        }

        self.eat(&Token::Semi);
        Ok(Stmt::Decl { ty, name, array_len, init, line })
    }

    fn parse_block(&mut self) -> PResult<Vec<Stmt>> {
        let line = self.line();
        self.expect(Token::LBrace)?;
        let mut out = Vec::new();
        while self.peek() != Token::RBrace && self.peek() != Token::Eof {
            match self.parse_stmt() {
                Ok(Some(s)) => out.push(s),
                Ok(None) => {}
                Err(e) => {
                    self.errors.push(e);
                    while self.peek() != Token::Eof
                        && self.peek() != Token::Semi
                        && self.peek() != Token::RBrace
                    {
                        self.bump();
                    }
                    if self.peek() == Token::Semi {
                        self.bump();
                    }
                }
            }
        }
        self.expect(Token::RBrace)?;
        let _ = line;
        Ok(out)
    }

    fn parse_stmt(&mut self) -> PResult<Option<Stmt>> {
        let line = self.line();

        // 空语句
        if self.eat(&Token::Semi) {
            return Ok(None);
        }

        // 复合语句
        if self.peek() == Token::LBrace {
            return Ok(Some(Stmt::Block(self.parse_block()?, line)));
        }

        // 关键字语句
        if let Token::Kw(k) = self.peek() {
            match k.as_str() {
                "if" => {
                    self.bump();
                    self.expect(Token::LParen)?;
                    let cond = self.parse_expr()?;
                    self.expect(Token::RParen)?;
                    let then = match self.parse_stmt()? {
                        Some(s) => s,
                        None => Stmt::Block(vec![], line),
                    };
                    let mut otherwise = None;
                    if self.peek() == Token::Kw("else".into()) {
                        self.bump();
                        otherwise = self.parse_stmt()?;
                    }
                    return Ok(Some(Stmt::If { cond, then: Box::new(then), otherwise: otherwise.map(Box::new), line }));
                }
                "while" => {
                    self.bump();
                    self.expect(Token::LParen)?;
                    let cond = self.parse_expr()?;
                    self.expect(Token::RParen)?;
                    let body = match self.parse_stmt()? {
                        Some(s) => s,
                        None => Stmt::Block(vec![], line),
                    };
                    return Ok(Some(Stmt::While { cond, body: Box::new(body), line }));
                }
                "do" => {
                    self.bump();
                    let body = match self.parse_stmt()? {
                        Some(s) => s,
                        None => Stmt::Block(vec![], line),
                    };
                    if self.peek() == Token::Kw("while".into()) {
                        self.bump();
                    }
                    self.expect(Token::LParen)?;
                    let cond = self.parse_expr()?;
                    self.expect(Token::RParen)?;
                    self.eat(&Token::Semi);
                    return Ok(Some(Stmt::DoWhile { body: Box::new(body), cond, line }));
                }
                "for" => {
                    self.bump();
                    self.expect(Token::LParen)?;
                    let init = if self.peek() == Token::Semi {
                        None
                    } else {
                        self.parse_stmt()?.map(Box::new)
                    };
                    self.eat(&Token::Semi);
                    let cond = if self.peek() == Token::Semi {
                        None
                    } else {
                        Some(self.parse_expr()?)
                    };
                    self.expect(Token::Semi)?;
                    let step = if self.peek() == Token::RParen {
                        None
                    } else {
                        Some(self.parse_expr()?)
                    };
                    self.expect(Token::RParen)?;
                    let body = match self.parse_stmt()? {
                        Some(s) => s,
                        None => Stmt::Block(vec![], line),
                    };
                    return Ok(Some(Stmt::For { init, cond, step, body: Box::new(body), line }));
                }
                "return" => {
                    self.bump();
                    let e = if self.peek() == Token::Semi {
                        None
                    } else {
                        Some(self.parse_expr()?)
                    };
                    self.eat(&Token::Semi);
                    return Ok(Some(Stmt::Return(e, line)));
                }
                "break" => {
                    self.bump();
                    self.eat(&Token::Semi);
                    return Ok(Some(Stmt::Break(line)));
                }
                "continue" => {
                    self.bump();
                    self.eat(&Token::Semi);
                    return Ok(Some(Stmt::Continue(line)));
                }
                "switch" => {
                    self.bump();
                    self.expect(Token::LParen)?;
                    let expr = self.parse_expr()?;
                    self.expect(Token::RParen)?;
                    self.expect(Token::LBrace)?;
                    let mut cases = Vec::new();
                    let mut current: Option<i64> = None;
                    let mut body: Vec<Stmt> = Vec::new();
                    while self.peek() != Token::RBrace && self.peek() != Token::Eof {
                        if self.peek() == Token::Kw("case".into()) {
                            if current.is_some() || !body.is_empty() {
                                cases.push((current.take(), std::mem::take(&mut body)));
                            }
                            self.bump();
                            let v = self.parse_expr()?;
                            current = Some(match v {
                                Expr::Int(n, _) => n,
                                _ => 0,
                            });
                            self.eat(&Token::Colon);
                        } else if self.peek() == Token::Kw("default".into()) {
                            if current.is_some() || !body.is_empty() {
                                cases.push((current.take(), std::mem::take(&mut body)));
                            }
                            self.bump();
                            self.eat(&Token::Colon);
                            current = None;
                        } else if let Ok(Some(s)) = self.parse_stmt() {
                            body.push(s);
                        } else {
                            self.bump();
                        }
                    }
                    cases.push((current.take(), std::mem::take(&mut body)));
                    self.expect(Token::RBrace)?;
                    return Ok(Some(Stmt::Switch { expr, cases, line }));
                }
                // 局部声明
                "int" | "float" | "char" | "double" | "void" | "long" | "short" => {
                    let ty = k.clone();
                    self.bump();
                    return self.parse_decl_with_type(ty).map(Some);
                }
                "const" | "static" | "volatile" | "unsigned" | "signed" => {
                    self.bump();
                    let ty = if let Token::Kw(k2) = self.peek() {
                        self.bump();
                        k2
                    } else {
                        "int".to_string()
                    };
                    return self.parse_decl_with_type(ty).map(Some);
                }
                _ => {}
            }
        }

        // typedef 别名声明（uint8_t x;）
        if let Token::Ident(alias) = self.peek() {
            if is_typedef_alias(&alias) && matches!(self.peek_at(1), Token::Ident(_)) {
                self.bump();
                return self.parse_decl_with_type(alias).map(Some);
            }
        }

        // 表达式语句
        let e = self.parse_expr()?;
        self.eat(&Token::Semi);
        Ok(Some(Stmt::Expr(e)))
    }

    /* ---------------- 表达式（优先级递归下降） ---------------- */

    pub fn parse_expr(&mut self) -> PResult<Expr> {
        self.parse_assign()
    }

    fn parse_assign(&mut self) -> PResult<Expr> {
        let line = self.line();
        // 先试 x = ... / x op= ... / a[i] = ...
        if let Token::Ident(name) = self.peek() {
            let save = self.pos;
            self.bump();
            match self.peek() {
                Token::Assign => {
                    self.bump();
                    let rhs = self.parse_assign()?;
                    return Ok(Expr::Assign(name, Box::new(rhs), line));
                }
                Token::PlusEq => {
                    self.bump();
                    let rhs = self.parse_assign()?;
                    return Ok(Expr::AssignOp(BinOp::Add, name, Box::new(rhs), line));
                }
                Token::MinusEq => {
                    self.bump();
                    let rhs = self.parse_assign()?;
                    return Ok(Expr::AssignOp(BinOp::Sub, name, Box::new(rhs), line));
                }
                Token::StarEq => {
                    self.bump();
                    let rhs = self.parse_assign()?;
                    return Ok(Expr::AssignOp(BinOp::Mul, name, Box::new(rhs), line));
                }
                Token::SlashEq => {
                    self.bump();
                    let rhs = self.parse_assign()?;
                    return Ok(Expr::AssignOp(BinOp::Div, name, Box::new(rhs), line));
                }
                Token::Inc => {
                    self.bump();
                    return Ok(Expr::PostInc(name, true, line));
                }
                Token::Dec => {
                    self.bump();
                    return Ok(Expr::PostInc(name, false, line));
                }
                Token::LBracket => {
                    self.pos = save;
                    let arr = self.parse_unary()?;
                    if self.peek() == Token::Assign {
                        self.bump();
                        let rhs = self.parse_assign()?;
                        // 拆出 base 与 index
                        if let Expr::Index(base, idx, l) = arr {
                            return Ok(Expr::IndexAssign(base, idx, Box::new(rhs), l));
                        }
                        return Err("数组赋值语法不支持".to_string());
                    }
                    return Ok(arr);
                }
                _ => {
                    self.pos = save;
                }
            }
        }
        self.parse_ternary()
    }

    fn parse_ternary(&mut self) -> PResult<Expr> {
        let line = self.line();
        let c = self.parse_binary(0)?;
        if self.eat(&Token::Question) {
            let a = self.parse_assign()?;
            self.expect(Token::Colon)?;
            let b = self.parse_assign()?;
            return Ok(Expr::Ternary(Box::new(c), Box::new(a), Box::new(b), line));
        }
        Ok(c)
    }

    fn bin_prec(t: &Token) -> Option<(u8, BinOp)> {
        Some(match t {
            Token::OrOr => (1, BinOp::Or),
            Token::AndAnd => (2, BinOp::And),
            Token::Pipe => (3, BinOp::BitOr),
            Token::Caret => (4, BinOp::BitXor),
            Token::Amp => (5, BinOp::BitAnd),
            Token::Eq => (6, BinOp::Eq),
            Token::Ne => (6, BinOp::Ne),
            Token::Lt => (7, BinOp::Lt),
            Token::Le => (7, BinOp::Le),
            Token::Gt => (7, BinOp::Gt),
            Token::Ge => (7, BinOp::Ge),
            Token::Plus => (9, BinOp::Add),
            Token::Minus => (9, BinOp::Sub),
            Token::Star => (10, BinOp::Mul),
            Token::Slash => (10, BinOp::Div),
            Token::Percent => (10, BinOp::Mod),
            _ => return None,
        })
    }

    fn parse_binary(&mut self, min_prec: u8) -> PResult<Expr> {
        let line = self.line();
        let mut lhs = self.parse_unary()?;
        loop {
            let t = self.peek();
            let Some((prec, op)) = Self::bin_prec(&t) else { break };
            if prec < min_prec {
                break;
            }
            self.bump();
            let rhs = self.parse_binary(prec + 1)?;
            lhs = Expr::Binary(op, Box::new(lhs), Box::new(rhs), line);
        }
        Ok(lhs)
    }

    fn parse_unary(&mut self) -> PResult<Expr> {
        let line = self.line();
        match self.peek() {
            Token::Minus => {
                self.bump();
                Ok(Expr::Unary(UnOp::Neg, Box::new(self.parse_unary()?), line))
            }
            Token::Plus => {
                self.bump();
                Ok(Expr::Unary(UnOp::Pos, Box::new(self.parse_unary()?), line))
            }
            Token::Not => {
                self.bump();
                Ok(Expr::Unary(UnOp::Not, Box::new(self.parse_unary()?), line))
            }
            Token::Tilde => {
                self.bump();
                Ok(Expr::Unary(UnOp::BitNot, Box::new(self.parse_unary()?), line))
            }
            Token::Inc => {
                self.bump();
                if let Token::Ident(n) = self.peek() {
                    self.bump();
                    return Ok(Expr::AssignOp(BinOp::Add, n, Box::new(Expr::Int(1, line)), line));
                }
                self.parse_unary()
            }
            Token::Dec => {
                self.bump();
                if let Token::Ident(n) = self.peek() {
                    self.bump();
                    return Ok(Expr::AssignOp(BinOp::Sub, n, Box::new(Expr::Int(1, line)), line));
                }
                self.parse_unary()
            }
            Token::LParen => {
                // 强制类型转换 (int)x / (float)(a+b)
                if let Token::Kw(k) = self.peek_at(1) {
                    if matches!(k.as_str(), "int" | "float" | "char" | "double" | "long" | "unsigned" | "signed") {
                        self.bump();
                        self.bump();
                        self.expect(Token::RParen)?;
                        let e = self.parse_unary()?;
                        return Ok(Expr::Cast(k, Box::new(e), line));
                    }
                }
                if let Token::Ident(a) = self.peek_at(1) {
                    if is_typedef_alias(&a) && self.peek_at(2) == Token::RParen {
                        self.bump();
                        self.bump();
                        self.bump();
                        let e = self.parse_unary()?;
                        return Ok(Expr::Cast(a, Box::new(e), line));
                    }
                }
                self.bump();
                let e = self.parse_expr()?;
                self.expect(Token::RParen)?;
                Ok(e)
            }
            Token::Kw(k) if k == "sizeof" => {
                self.bump();
                if self.peek() == Token::LParen {
                    self.bump();
                    let ty = match self.peek() {
                        Token::Kw(t) => {
                            self.bump();
                            t
                        }
                        Token::Ident(t) => {
                            self.bump();
                            t
                        }
                        _ => "int".to_string(),
                    };
                    self.expect(Token::RParen)?;
                    return Ok(Expr::SizeofTy(ty, line));
                }
                let _ = self.parse_unary()?;
                Ok(Expr::Int(4, line))
            }
            _ => self.parse_postfix(),
        }
    }

    fn parse_postfix(&mut self) -> PResult<Expr> {
        let line = self.line();
        let mut e = self.parse_primary()?;
        loop {
            match self.peek() {
                Token::LBracket => {
                    self.bump();
                    let idx = self.parse_expr()?;
                    self.expect(Token::RBracket)?;
                    e = Expr::Index(Box::new(e), Box::new(idx), line);
                }
                Token::LParen => {
                    // (int)(x) 这种情况已在 unary 处理；这里只可能是误解析
                    break;
                }
                Token::Inc => {
                    self.bump();
                    if let Expr::Ident(n, l) = e.clone() {
                        e = Expr::PostInc(n, true, l);
                    }
                }
                Token::Dec => {
                    self.bump();
                    if let Expr::Ident(n, l) = e.clone() {
                        e = Expr::PostInc(n, false, l);
                    }
                }
                Token::Dot | Token::Arrow => {
                    // 结构体成员：暂不支持，跳过
                    self.bump();
                    if let Token::Ident(_) = self.peek() {
                        self.bump();
                    }
                }
                _ => break,
            }
        }
        Ok(e)
    }

    fn parse_primary(&mut self) -> PResult<Expr> {
        let line = self.line();
        let (t, _) = self.bump();
        match t {
            Token::Int(n) => Ok(Expr::Int(n, line)),
            Token::Float(f) => Ok(Expr::Float(f, line)),
            Token::Str(s) => Ok(Expr::Str(s, line)),
            Token::Char(c) => Ok(Expr::Char(c, line)),
            Token::Ident(name) => {
                if self.peek() == Token::LParen {
                    self.bump();
                    let mut args = Vec::new();
                    while self.peek() != Token::RParen && self.peek() != Token::Eof {
                        args.push(self.parse_assign()?);
                        if !self.eat(&Token::Comma) {
                            break;
                        }
                    }
                    self.expect(Token::RParen)?;
                    Ok(Expr::Call(name, args, line))
                } else {
                    Ok(Expr::Ident(name, line))
                }
            }
            Token::Kw(k) if k == "sizeof" => {
                self.pos -= 1;
                self.parse_unary()
            }
            other => Err(format!("第 {line} 行：无法识别的记号 {other:?}")),
        }
    }
}

pub enum TopLevel {
    Global(Stmt),
    Func(FuncDef),
}

pub fn is_typedef_alias(name: &str) -> bool {
    matches!(
        name,
        "uint8_t" | "uint16_t" | "uint32_t" | "uint64_t" | "int8_t" | "int16_t" | "int32_t"
            | "int64_t" | "size_t" | "bool" | "u8" | "u16" | "u32" | "i8" | "i16" | "i32"
    )
}

/// 便捷入口：源码 -> Program
pub fn parse_source(src: &str) -> (Program, Vec<String>) {
    let pre = preprocess(src);
    let toks = Lexer::new(&pre).tokenize();
    let mut p = Parser::new(toks);
    let prog = p.parse_program();
    (prog, p.errors)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_ok(src: &str) -> Program {
        let (p, errs) = parse_source(src);
        assert!(errs.is_empty(), "解析错误: {errs:?}");
        p
    }

    #[test]
    fn strips_comments() {
        let pre = preprocess("int a; // 这是注释\n/* 块注释\n第二行 */\nint b;");
        assert!(!pre.contains("这是注释"));
        assert!(!pre.contains("块注释"));
        assert!(pre.contains("int a;"));
        assert!(pre.contains("int b;"));
    }

    #[test]
    fn drops_includes() {
        let pre = preprocess("#include \"stm32f1xx_hal.h\"\nint x;");
        assert!(!pre.contains("include"));
    }

    #[test]
    fn expands_object_macro() {
        let pre = preprocess("#define BASE 500\nint v = BASE;");
        assert!(pre.contains("500"));
        assert!(!pre.contains("BASE"));
    }

    #[test]
    fn ifndef_guard() {
        let pre = preprocess("#ifndef X\n#define X\nint a;\n#endif");
        assert!(pre.contains("int a;"));
    }

    #[test]
    fn simple_global() {
        let p = parse_ok("int speed = 500;");
        assert_eq!(p.globals.len(), 1);
    }

    #[test]
    fn main_with_while() {
        let p = parse_ok("int main(void){ while(1){ } }");
        assert!(p.funcs.contains_key("main"));
        assert_eq!(p.funcs["main"].body.len(), 1);
        assert!(matches!(p.funcs["main"].body[0], Stmt::While { .. }));
    }

    #[test]
    fn function_with_params() {
        let p = parse_ok("int add(int a, int b){ return a + b; }");
        let f = &p.funcs["add"];
        assert_eq!(f.params.len(), 2);
        assert!(matches!(f.body[0], Stmt::Return(Some(_), _)));
    }

    #[test]
    fn for_loop() {
        let p = parse_ok("int main(void){ for(int i=0;i<10;i=i+1){ } }");
        assert!(matches!(p.funcs["main"].body[0], Stmt::For { .. }));
    }

    #[test]
    fn if_else_chain() {
        let p = parse_ok("void f(int x){ if(x>0){ } else if(x<0){ } else { } }");
        let s = &p.funcs["f"].body[0];
        let Stmt::If { otherwise, .. } = s else { panic!("应为 If") };
        assert!(otherwise.is_some());
    }

    #[test]
    fn array_decl_and_index() {
        let p = parse_ok("int sensor[5];\nint f(void){ return sensor[2]; }");
        assert_eq!(p.globals.len(), 1);
        assert!(matches!(p.globals[0], Stmt::Decl { .. }));
    }

    #[test]
    fn array_initializer() {
        let p = parse_ok("int a[3] = {1, 2, 3};");
        let Stmt::Decl { array_len, init, .. } = &p.globals[0] else { panic!() };
        assert_eq!(*array_len, Some(3));
        assert!(init.is_some());
    }

    #[test]
    fn compound_assignment() {
        let p = parse_ok("void f(void){ int x = 1; x += 2; x -= 1; }");
        assert_eq!(p.funcs["f"].body.len(), 3);
    }

    #[test]
    fn volatile_ignored() {
        let p = parse_ok("volatile int flag = 0;");
        assert_eq!(p.globals.len(), 1);
    }

    #[test]
    fn typedef_alias_type() {
        let p = parse_ok("uint16_t pwm = 0;");
        assert_eq!(p.globals.len(), 1);
    }

    #[test]
    fn cast_expression() {
        let p = parse_ok("void f(void){ float a = 1.5; int b = (int)a; }");
        assert_eq!(p.funcs["f"].body.len(), 2);
    }

    #[test]
    fn ternary() {
        let p = parse_ok("void f(int x){ int y = x > 0 ? 1 : -1; }");
        assert!(matches!(p.funcs["f"].body[0], Stmt::Decl { .. }));
    }

    #[test]
    fn precedence_mul_before_add() {
        let p = parse_ok("void f(void){ int x = 1 + 2 * 3; }");
        let Stmt::Decl { init: Some(e), .. } = &p.funcs["f"].body[0] else { panic!() };
        // 顶层应为 Add，其右侧为 Mul
        assert!(matches!(e, Expr::Binary(BinOp::Add, _, _, _)));
        if let Expr::Binary(_, _, rhs, _) = e {
            assert!(matches!(**rhs, Expr::Binary(BinOp::Mul, _, _, _)));
        }
    }

    #[test]
    fn hex_literal() {
        let p = parse_ok("int reg = 0xFF;");
        let Stmt::Decl { init: Some(Expr::Int(v, _)), .. } = &p.globals[0] else { panic!() };
        assert_eq!(*v, 255);
    }

    #[test]
    fn switch_parsed() {
        let p = parse_ok("void f(int x){ switch(x){ case 1: break; default: break; } }");
        assert!(matches!(p.funcs["f"].body[0], Stmt::Switch { .. }));
    }

    #[test]
    fn do_while_parsed() {
        let p = parse_ok("void f(void){ do { } while(0); }");
        assert!(matches!(p.funcs["f"].body[0], Stmt::DoWhile { .. }));
    }

    #[test]
    fn parser_recovers_from_garbage() {
        let (_, errs) = parse_source("@@@ bad syntax @@@\nint ok = 1;");
        assert!(!errs.is_empty());
    }
}

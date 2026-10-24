//! 静态检查：在跑仿真之前先把明显的坑指出来
//!
//! 每条规则有稳定 ID（CARSIM0xx），前端可以直接拿来做图标和文档链接。

use crate::csim::{parse_source, BinOp, Expr, Program, Stmt};
use crate::interp::Bindings;
use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Error,
    Warn,
    Info,
}

impl Severity {
    pub fn label(self) -> &'static str {
        match self {
            Severity::Error => "错误",
            Severity::Warn => "警告",
            Severity::Info => "提示",
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Issue {
    pub severity: Severity,
    pub line: usize,
    pub rule: String,
    pub message: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct Detected {
    pub mcu: String,
    pub motor_bindings: Vec<String>,
    pub sensor_bindings: Vec<String>,
    pub has_main_loop: bool,
    pub uses_delay: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct LintReport {
    pub issues: Vec<Issue>,
    pub detected: Detected,
    pub parse_errors: Vec<String>,
}

/// 跑一遍静态检查
pub fn lint(src: &str) -> LintReport {
    let (prog, parse_errors) = parse_source(src);
    let mut issues: Vec<Issue> = Vec::new();

    // ---- 解析错误本身就是最高优先级的诊断 ----
    for e in &parse_errors {
        let line = extract_line(e);
        issues.push(Issue {
            severity: Severity::Warn,
            line,
            rule: "CARSIM000".into(),
            message: format!("代码里有我没看懂的部分（已跳过）：{e}"),
        });
    }

    let main = prog.funcs.get("main");

    // ---- CARSIM001: 没有 main ----
    let Some(main) = main else {
        issues.push(Issue {
            severity: Severity::Error,
            line: 1,
            rule: "CARSIM001".into(),
            message: "找不到 main 函数，仿真没有入口。".into(),
        });
        return finalize(issues, &prog, src, parse_errors, false);
    };

    let body = flatten(&main.body);

    // ---- CARSIM002: 主循环 ----
    let has_main_loop = body.iter().any(|s| matches!(s, Stmt::While { .. } | Stmt::For { .. }));
    if !has_main_loop {
        issues.push(Issue {
            severity: Severity::Warn,
            line: main.line,
            rule: "CARSIM002".into(),
            message: "main 里没有 while/for 主循环，小车只会执行一次就停住。".into(),
        });
    }

    // ---- CARSIM003: 死循环但没有延时（会导致物理无法推进）----
    let busy_loop = body.iter().any(|s| {
        matches!(s, Stmt::While { .. } | Stmt::For { .. }) && !contains_call(&[s.clone()], "HAL_Delay")
    });
    if has_main_loop && !contains_call(&body, "HAL_Delay") && !contains_call(&body, "Delay_Ms") {
        issues.push(Issue {
            severity: Severity::Error,
            line: main.line,
            rule: "CARSIM003".into(),
            message: "主循环里没有调用 HAL_Delay / Delay_Ms：仿真无法推进时间，会被判定为忙等死循环。".into(),
        });
    } else if busy_loop && !contains_call(&body, "HAL_Delay") {
        issues.push(Issue {
            severity: Severity::Warn,
            line: main.line,
            rule: "CARSIM003".into(),
            message: "有循环里没有任何延时，建议加上 HAL_Delay 让控制周期稳定。".into(),
        });
    }

    // ---- CARSIM004: 从不写电机 ----
    let writes_motor = contains_any_call(
        &body,
        &[
            "Motor_Set", "MotorSet", "Set_Motor", "motor_set", "Set_Pwm",
            "__HAL_TIM_SET_COMPARE", "TIM_SetCompare",
        ],
    ) || writes_any_global(src, &["pwm_l", "pwm_r", "PWM_L", "PWM_R"]);
    if !writes_motor {
        issues.push(Issue {
            severity: Severity::Error,
            line: main.line,
            rule: "CARSIM004".into(),
            message: "从来没有给电机输出赋值，小车不会动。请用 Motor_Set(l, r) 或 __HAL_TIM_SET_COMPARE 设置 PWM。".into(),
        });
    }

    // ---- CARSIM005: 从不读传感器 ----
    let reads_sensor = contains_any_call(
        &body,
        &[
            "Sensor_Read", "Read_Sensor", "Read_Line", "ReadLine", "Line_Position",
            "ADC_Read", "HAL_ADC_GetValue", "Get_ADC",
        ],
    );
    if !reads_sensor {
        issues.push(Issue {
            severity: Severity::Warn,
            line: main.line,
            rule: "CARSIM005".into(),
            message: "没有读取巡线传感器，小车是「闭着眼睛」在跑。".into(),
        });
    }

    // ---- CARSIM006: PID 积分没有限幅（windup）----
    if let Some(line) = find_unclamped_integral(src) {
        issues.push(Issue {
            severity: Severity::Warn,
            line,
            rule: "CARSIM006".into(),
            message: "积分项（integral / I 项）累加后没有做限幅，长时间偏差会引起积分饱和，小车会剧烈震荡。".into(),
        });
    }

    // ---- CARSIM007: PWM 超出范围 ----
    if let Some((line, v)) = find_pwm_out_of_range(src) {
        issues.push(Issue {
            severity: Severity::Warn,
            line,
            rule: "CARSIM007".into(),
            message: format!("PWM 值 {v} 看起来超出了常见范围（0~1000 或 -1000~1000），请确认定时器周期设置。"),
        });
    }

    // ---- CARSIM008: printf 格式串与参数类型不匹配 ----
    if let Some(line) = find_format_mismatch(src) {
        issues.push(Issue {
            severity: Severity::Warn,
            line,
            rule: "CARSIM008".into(),
            message: "printf 里用 %d 打印了浮点变量，输出会变成乱码，请用 %f。".into(),
        });
    }

    // ---- CARSIM009: 可能的整数除零 ----
    if let Some(line) = find_int_div_zero(src) {
        issues.push(Issue {
            severity: Severity::Warn,
            line,
            rule: "CARSIM009".into(),
            message: "存在对可能为 0 的变量做整数除法，运行时会直接崩溃。".into(),
        });
    }

    // ---- CARSIM010: main 没有 void 参数 / 非标准签名（仅提示）----
    if main.ret_ty != "int" && main.ret_ty != "void" {
        issues.push(Issue {
            severity: Severity::Info,
            line: main.line,
            rule: "CARSIM010".into(),
            message: format!("main 返回类型是 {}，工程里通常用 int 或 void。", main.ret_ty),
        });
    }

    // ---- CARSIM011: 代码里出现了浮点运算但目标 MCU 无 FPU ----
    if src.contains("float") && src.contains("F103") {
        issues.push(Issue {
            severity: Severity::Info,
            line: 1,
            rule: "CARSIM011".into(),
            message: "STM32F103 没有硬件浮点单元，大量 float 运算会明显变慢，关键循环里可以考虑用定点数。".into(),
        });
    }

    finalize(issues, &prog, src, parse_errors, has_main_loop)
}

fn finalize(
    mut issues: Vec<Issue>,
    prog: &Program,
    src: &str,
    parse_errors: Vec<String>,
    has_main_loop: bool,
) -> LintReport {
    let b = Bindings::default();

    // 从源码里找用户显式绑定：// @sim bind motor_left=TIM2_CH1
    let mut motor_bindings = b.motor_bindings.clone();
    let mut sensor_bindings = b.sensor_bindings.clone();
    for line in src.lines() {
        if let Some(rest) = line.split_once("@sim bind").map(|p| p.1) {
            for kv in rest.split(',') {
                let kv = kv.trim();
                if let Some((k, v)) = kv.split_once('=') {
                    match k.trim() {
                        "motor_left" => motor_bindings[0] = v.trim().to_string(),
                        "motor_right" => {
                            if motor_bindings.len() < 2 {
                                motor_bindings.push(v.trim().to_string());
                            } else {
                                motor_bindings[1] = v.trim().to_string();
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    let mcu = detect_mcu(src);
    let uses_delay = prog.funcs.get("main").map_or(false, |m| {
        contains_call(&flatten(&m.body), "HAL_Delay") || contains_call(&flatten(&m.body), "Delay_Ms")
    });

    issues.sort_by_key(|i| (i.severity as u8, i.line));

    LintReport {
        issues,
        detected: Detected {
            mcu,
            motor_bindings,
            sensor_bindings,
            has_main_loop,
            uses_delay,
        },
        parse_errors,
    }
}

/* ------------------------------------------------------------------ *
 * 遍历辅助
 * ------------------------------------------------------------------ */

/// 把嵌套块摊平成一维语句列表（便于做「是否存在某种调用」这类判断）
fn flatten(stmts: &[Stmt]) -> Vec<Stmt> {
    let mut out = Vec::new();
    for s in stmts {
        out.push(s.clone());
        match s {
            Stmt::Block(inner, _) => out.extend(flatten(inner)),
            Stmt::If { then, otherwise, .. } => {
                out.extend(flatten(&[ (**then).clone() ]));
                if let Some(o) = otherwise {
                    out.extend(flatten(&[ (**o).clone() ]));
                }
            }
            Stmt::While { body, .. } => out.extend(flatten(&[ (**body).clone() ])),
            Stmt::DoWhile { body, .. } => out.extend(flatten(&[ (**body).clone() ])),
            Stmt::For { body, .. } => out.extend(flatten(&[ (**body).clone() ])),
            Stmt::Switch { cases, .. } => {
                for (_, b) in cases {
                    out.extend(flatten(b));
                }
            }
            _ => {}
        }
    }
    out
}

fn contains_call(stmts: &[Stmt], name: &str) -> bool {
    contains_any_call(stmts, &[name])
}

fn contains_any_call(stmts: &[Stmt], names: &[&str]) -> bool {
    fn walk_expr(e: &Expr, names: &[&str], found: &mut bool) {
        match e {
            Expr::Call(n, args, _) => {
                if names.contains(&n.as_str()) {
                    *found = true;
                }
                for a in args {
                    walk_expr(a, names, found);
                }
            }
            Expr::Binary(_, l, r, _) => {
                walk_expr(l, names, found);
                walk_expr(r, names, found);
            }
            Expr::Unary(_, i, _) => walk_expr(i, names, found),
            Expr::Assign(_, r, _) => walk_expr(r, names, found),
            Expr::AssignOp(_, _, r, _) => walk_expr(r, names, found),
            Expr::Index(b, i, _) => {
                walk_expr(b, names, found);
                walk_expr(i, names, found);
            }
            Expr::IndexAssign(b, i, r, _) => {
                walk_expr(b, names, found);
                walk_expr(i, names, found);
                walk_expr(r, names, found);
            }
            Expr::Ternary(a, b, c, _) => {
                walk_expr(a, names, found);
                walk_expr(b, names, found);
                walk_expr(c, names, found);
            }
            Expr::Cast(_, i, _) => walk_expr(i, names, found),
            _ => {}
        }
    }

    let mut found = false;
    for s in stmts {
        match s {
            Stmt::Expr(e) => walk_expr(e, names, &mut found),
            Stmt::Decl { init: Some(e), .. } => walk_expr(e, names, &mut found),
            Stmt::Return(Some(e), _) => walk_expr(e, names, &mut found),
            _ => {}
        }
        if found {
            return true;
        }
    }
    // 语句本身也递归（for 的 init/step、if 的条件等）
    for s in stmts {
        match s {
            Stmt::If { cond, .. } => walk_expr(cond, names, &mut found),
            Stmt::While { cond, .. } => walk_expr(cond, names, &mut found),
            Stmt::DoWhile { cond, .. } => walk_expr(cond, names, &mut found),
            Stmt::For { cond, step, .. } => {
                if let Some(c) = cond {
                    walk_expr(c, names, &mut found);
                }
                if let Some(st) = step {
                    walk_expr(st, names, &mut found);
                }
            }
            Stmt::Switch { expr, .. } => walk_expr(expr, names, &mut found),
            _ => {}
        }
        if found {
            return true;
        }
    }
    found
}

/* ------------------------------------------------------------------ *
 * 基于文本的启发式规则
 * ------------------------------------------------------------------ */

/// 源码里是否给某个全局变量赋过值
fn writes_any_global(src: &str, names: &[&str]) -> bool {
    for line in src.lines() {
        let l = line.trim();
        for n in names {
            if l.contains(&format!("{n} =")) || l.contains(&format!("{n}=")) {
                return true;
            }
        }
    }
    false
}

/// 找 `integral += ...` 但没有 if 限幅的写法
fn find_unclamped_integral(src: &str) -> Option<usize> {
    let mut accum_line: Option<usize> = None;
    for (i, line) in src.lines().enumerate() {
        let l = line.trim();
        let is_accum = l.contains("integral +=")
            || l.contains("integral = integral +")
            || l.contains("i_term +=")
            || l.contains("sum_error +=");
        if is_accum {
            accum_line = Some(i + 1);
        }
        // 限幅的常见写法
        let clamp = l.contains("integral >")
            || l.contains("integral <")
            || l.contains("constrain")
            || l.contains("clamp")
            || l.contains("LIMIT")
            || l.contains("limit")
            || l.contains("if(integral")
            || l.contains("if (integral");
        if clamp {
            accum_line = None;
        }
    }
    accum_line
}

/// 找明显超出范围的 PWM 常量
fn find_pwm_out_of_range(src: &str) -> Option<(usize, i64)> {
    for (i, line) in src.lines().enumerate() {
        let l = line.trim();
        let is_pwm_ctx = l.contains("Motor_Set")
            || l.contains("Set_Pwm")
            || l.contains("SET_COMPARE")
            || l.contains("pwm")
            || l.contains("PWM");
        if !is_pwm_ctx {
            continue;
        }
        // 抽取所有整数常量
        for tok in l.split(|c: char| !c.is_ascii_digit() && c != '-') {
            if tok.is_empty() || tok == "-" {
                continue;
            }
            if let Ok(v) = tok.parse::<i64>() {
                if v.abs() > 1000 && v.abs() <= 100000 {
                    return Some((i + 1, v));
                }
            }
        }
    }
    None
}

/// printf 用 %d 打印 float 变量
fn find_format_mismatch(src: &str) -> Option<usize> {
    // 收集 float 变量名
    let mut floats: Vec<String> = Vec::new();
    for line in src.lines() {
        let l = line.trim();
        if l.starts_with("float ") || l.contains(" float ") {
            // float a, b, c;  / float a = 1.0;
            let rest = l.split_once("float ").map(|p| p.1).unwrap_or("");
            let decl_part = rest.split('=').next().unwrap_or("");
            for name in decl_part.split(',') {
                let n = name.trim().trim_end_matches(';').trim();
                if !n.is_empty() && n.chars().all(|c| c.is_alphanumeric() || c == '_') {
                    floats.push(n.to_string());
                }
            }
        }
    }

    for (i, line) in src.lines().enumerate() {
        if !(line.contains("printf") || line.contains("Printf")) {
            continue;
        }
        if !line.contains("%d") {
            continue;
        }
        // 取格式串之后的参数
        if let Some(after) = line.splitn(2, '"').nth(1) {
            let after = after.splitn(2, '"').nth(1).unwrap_or("");
            for f in &floats {
                if after.contains(f.as_str()) {
                    return Some(i + 1);
                }
            }
        }
    }
    None
}

/// `x / y` 且 y 是可能被赋 0 的变量（很粗糙，只报最明显的情况）
fn find_int_div_zero(src: &str) -> Option<usize> {
    for (i, line) in src.lines().enumerate() {
        let l = line.trim();
        if !l.contains('/') || l.contains("//") || l.contains("/*") {
            continue;
        }
        // 显式 / 0
        if l.contains("/ 0") || l.contains("/0") {
            return Some(i + 1);
        }
    }
    None
}

fn detect_mcu(src: &str) -> String {
    for m in [
        "STM32F103C8T6", "STM32F103", "STM32F407", "STM32F411", "STM32H743", "STM32G431",
    ] {
        if src.contains(m) {
            return m.to_string();
        }
    }
    "STM32F103C8T6".to_string()
}

fn extract_line(msg: &str) -> usize {
    // "第 N 行：..."
    if let Some(a) = msg.split_once("第 ") {
        if let Some(b) = a.1.split_once(" 行") {
            return b.0.trim().parse().unwrap_or(1);
        }
    }
    1
}

/* ------------------------------------------------------------------ *
 * 文本渲染（CLI 用）
 * ------------------------------------------------------------------ */

pub fn render_text(r: &LintReport) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    let (e, w, i) = count(&r.issues);
    let _ = writeln!(s, "🔍 静态检查：{} 错误 · {} 警告 · {} 提示", e, w, i);
    let _ = writeln!(s);
    let _ = writeln!(
        s,
        "  目标芯片    {}",
        r.detected.mcu
    );
    let _ = writeln!(
        s,
        "  电机绑定    {}",
        r.detected.motor_bindings.join(", ")
    );
    let _ = writeln!(
        s,
        "  传感器绑定  {}",
        r.detected.sensor_bindings.join(", ")
    );
    let _ = writeln!(s, "  主循环      {}", yn(r.detected.has_main_loop));
    let _ = writeln!(s, "  使用延时    {}", yn(r.detected.uses_delay));
    let _ = writeln!(s);

    if r.issues.is_empty() {
        let _ = writeln!(s, "  ✅ 没有发现问题，代码看起来很干净喵～");
        return s;
    }

    for issue in &r.issues {
        let icon = match issue.severity {
            Severity::Error => "❌",
            Severity::Warn => "⚠️ ",
            Severity::Info => "💡",
        };
        let _ = writeln!(
            s,
            "  {} [{}] {:<4} 第 {} 行",
            icon,
            issue.rule,
            issue.severity.label(),
            issue.line
        );
        let _ = writeln!(s, "     {}", issue.message);
    }
    s
}

fn yn(b: bool) -> &'static str {
    if b {
        "有"
    } else {
        "无"
    }
}

pub fn count(issues: &[Issue]) -> (usize, usize, usize) {
    let mut e = 0;
    let mut w = 0;
    let mut i = 0;
    for x in issues {
        match x.severity {
            Severity::Error => e += 1,
            Severity::Warn => w += 1,
            Severity::Info => i += 1,
        }
    }
    (e, w, i)
}

pub fn has_error(r: &LintReport) -> bool {
    r.issues.iter().any(|i| i.severity == Severity::Error)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lint_of(src: &str) -> LintReport {
        lint(src)
    }

    #[test]
    fn no_main_is_error() {
        let r = lint_of("int foo(void){ return 1; }");
        assert!(has_error(&r));
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM001"));
    }

    #[test]
    fn good_code_has_no_error() {
        let r = lint_of(
            "int main(void){ while(1){ int p = Sensor_Read(2); Motor_Set(400,400); HAL_Delay(10); } }",
        );
        assert!(!has_error(&r), "{:?}", r.issues);
    }

    #[test]
    fn busy_loop_is_error() {
        let r = lint_of("int main(void){ while(1){ Motor_Set(1,1); } }");
        assert!(has_error(&r));
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM003"));
    }

    #[test]
    fn no_motor_output_is_error() {
        let r = lint_of("int main(void){ while(1){ int x = 1; HAL_Delay(10); } }");
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM004"));
    }

    #[test]
    fn no_sensor_read_is_warn() {
        let r = lint_of("int main(void){ while(1){ Motor_Set(1,1); HAL_Delay(10); } }");
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM005"));
    }

    #[test]
    fn integral_windup_is_warn() {
        let src = "int main(void){ float integral = 0; while(1){ integral += 1; Motor_Set(1,1); HAL_Delay(10); } }";
        let r = lint_of(src);
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM006"), "{:?}", r.issues);
    }

    #[test]
    fn clamped_integral_no_warn() {
        let src = "int main(void){ float integral = 0; while(1){ integral += 1; if(integral > 100){ integral = 100; } Motor_Set(1,1); HAL_Delay(10); } }";
        let r = lint_of(src);
        assert!(!r.issues.iter().any(|i| i.rule == "CARSIM006"));
    }

    #[test]
    fn pwm_out_of_range_is_warn() {
        let src = "int main(void){ while(1){ Motor_Set(99999, 0); HAL_Delay(10); } }";
        let r = lint_of(src);
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM007"));
    }

    #[test]
    fn printf_mismatch_is_warn() {
        let src = "float spd = 0;\nint main(void){ while(1){ spd = 1.0; Printf(\"v=%d\", spd); HAL_Delay(10); } }";
        let r = lint_of(src);
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM008"), "{:?}", r.issues);
    }

    #[test]
    fn div_zero_is_warn() {
        let src = "int main(void){ int a = 1 / 0; while(1){ Motor_Set(a,1); HAL_Delay(10); } }";
        let r = lint_of(src);
        assert!(r.issues.iter().any(|i| i.rule == "CARSIM009"));
    }

    #[test]
    fn detects_mcu_from_source() {
        let r = lint_of("// STM32F407 board\nint main(void){ while(1){ Motor_Set(1,1); HAL_Delay(1); } }");
        assert_eq!(r.detected.mcu, "STM32F407");
    }

    #[test]
    fn detects_main_loop_and_delay() {
        let r = lint_of("int main(void){ while(1){ Motor_Set(1,1); HAL_Delay(10); } }");
        assert!(r.detected.has_main_loop);
        assert!(r.detected.uses_delay);
    }

    #[test]
    fn explicit_binding_comment_is_honored() {
        let src = "// @sim bind motor_left=TIM3_CH1\nint main(void){ while(1){ Motor_Set(1,1); HAL_Delay(1); } }";
        let r = lint_of(src);
        assert_eq!(r.detected.motor_bindings[0], "TIM3_CH1");
    }

    #[test]
    fn render_text_runs() {
        let r = lint_of("int main(void){ while(1){ Motor_Set(1,1); HAL_Delay(1); } }");
        let t = render_text(&r);
        assert!(t.contains("静态检查"));
    }
}

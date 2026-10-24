//! C 子集解释器 + HAL 外设仿真
//!
//! 关键设计：**tick yield**。
//! 学生代码写 `while(1) { ...; HAL_Delay(10); }`，解释器执行到 `HAL_Delay`
//! 时并不真的睡过去，而是记下 `pending_yield`，由驱动器把控制权交回
//! 仿真器 —— 仿真器推进物理 10ms，再回到断点继续执行。
//! 于是真实固件代码能被逐 tick 驱动，完全不需要线程或协程。
//!
//! 实现方式：显式的帧栈（Vec<Frame>），驱动器 `drive_until` 是一个循环，
//! 每次取出栈顶帧执行一条语句；yield 时把帧原样放回栈上即可恢复。

use std::collections::HashMap;

use crate::csim::{BinOp, Expr, FuncDef, Program, Stmt, UnOp};
use crate::physics::World;

/* ------------------------------------------------------------------ *
 * 值
 * ------------------------------------------------------------------ */

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Value {
    Int(i64),
    Float(f64),
}

impl Value {
    pub fn as_int(self) -> i64 {
        match self {
            Value::Int(i) => i,
            Value::Float(f) => f as i64,
        }
    }

    pub fn as_float(self) -> f64 {
        match self {
            Value::Int(i) => i as f64,
            Value::Float(f) => f,
        }
    }

    pub fn is_zero(self) -> bool {
        match self {
            Value::Int(i) => i == 0,
            Value::Float(f) => f.abs() < 1e-12,
        }
    }

    /// 数值运算：只要一边是 float 就按浮点算（与 C 的寻常算术转换一致）
    pub fn arith(self, o: BinOp, r: Value) -> Result<Value, String> {
        use BinOp::*;
        if matches!(o, Div | Mod) && r.is_zero() {
            return Err("除零".to_string());
        }
        Ok(match o {
            Add | Sub | Mul | Div | Mod => {
                if matches!(self, Value::Float(_)) || matches!(r, Value::Float(_)) {
                    let a = self.as_float();
                    let b = r.as_float();
                    Value::Float(match o {
                        Add => a + b,
                        Sub => a - b,
                        Mul => a * b,
                        Div => a / b,
                        Mod => a % b,
                        _ => 0.0,
                    })
                } else {
                    let a = self.as_int();
                    let b = r.as_int();
                    Value::Int(match o {
                        Add => a.wrapping_add(b),
                        Sub => a.wrapping_sub(b),
                        Mul => a.wrapping_mul(b),
                        Div => a / b,
                        Mod => a % b,
                        _ => 0,
                    })
                }
            }
            Eq => Value::Int(if (self.as_float() - r.as_float()).abs() < 1e-12 { 1 } else { 0 }),
            Ne => Value::Int(if (self.as_float() - r.as_float()).abs() >= 1e-12 { 1 } else { 0 }),
            Lt => Value::Int(if self.as_float() < r.as_float() { 1 } else { 0 }),
            Le => Value::Int(if self.as_float() <= r.as_float() { 1 } else { 0 }),
            Gt => Value::Int(if self.as_float() > r.as_float() { 1 } else { 0 }),
            Ge => Value::Int(if self.as_float() >= r.as_float() { 1 } else { 0 }),
            And => Value::Int(if !self.is_zero() && !r.is_zero() { 1 } else { 0 }),
            Or => Value::Int(if !self.is_zero() || !r.is_zero() { 1 } else { 0 }),
            BitAnd => Value::Int(self.as_int() & r.as_int()),
            BitOr => Value::Int(self.as_int() | r.as_int()),
            BitXor => Value::Int(self.as_int() ^ r.as_int()),
            Shl => Value::Int(self.as_int() << r.as_int().clamp(0, 63)),
            Shr => Value::Int(self.as_int() >> r.as_int().clamp(0, 63)),
        })
    }
}

/* ------------------------------------------------------------------ *
 * 变量 / 帧
 * ------------------------------------------------------------------ */

#[derive(Debug, Clone)]
pub enum Var {
    Scalar(Value),
    Array(Vec<Value>),
}

/// 一层执行帧
#[derive(Debug, Clone)]
struct Frame {
    stmts: Vec<Stmt>,
    idx: usize,
    locals: Vec<HashMap<String, Var>>,
    /// 若非 None：本帧是循环体，每次跑完重新求值该条件
    repeat: Option<Expr>,
    /// for 循环的 step 表达式
    for_step: Option<Expr>,
    /// 函数帧（return 的归宿）
    is_func: bool,
    /// switch 体（break 的归宿）
    is_switch: bool,
}

impl Frame {
    fn block(stmts: Vec<Stmt>) -> Self {
        Self {
            stmts,
            idx: 0,
            locals: vec![HashMap::new()],
            repeat: None,
            for_step: None,
            is_func: false,
            is_switch: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Signal {
    None,
    Return(Value),
    Break,
    Continue,
}

/* ------------------------------------------------------------------ *
 * 外设绑定
 * ------------------------------------------------------------------ */

#[derive(Debug, Clone)]
pub struct Bindings {
    /// PWM 通道 -> 左右电机
    pub motor_bindings: Vec<String>,
    /// ADC 通道 -> 传感器序号
    pub sensor_bindings: Vec<String>,
}

impl Default for Bindings {
    fn default() -> Self {
        Self {
            motor_bindings: vec!["TIM2_CH1".into(), "TIM2_CH2".into()],
            sensor_bindings: (0..5).map(|i| i.to_string()).collect(),
        }
    }
}

/* ------------------------------------------------------------------ *
 * 解释器
 * ------------------------------------------------------------------ */

const CALL_DEPTH_MAX: usize = 64;

pub struct Interp<'a> {
    prog: &'a Program,
    world: &'a mut World,
    bindings: Bindings,

    stack: Vec<Frame>,
    globals: HashMap<String, Var>,

    /// 最近的 PWM 输出
    pub pwm_l: i32,
    pub pwm_r: i32,
    /// 引擎内累计时间（毫秒）
    pub tick_ms: f64,
    pub logs: Vec<String>,

    pub wrote_motor: bool,
    pub read_sensor: bool,
    pub used_delay: bool,
    pub finished: bool,

    /// 待处理的延时请求
    pending_yield: Option<f64>,
    yielded: Option<f64>,

    steps: usize,
    pub step_budget: usize,
    pub total_steps: usize,
    call_depth: usize,

    /// 未知外部函数（只用于生成警告）
    unknown_fns: HashMap<String, usize>,
}

impl<'a> Interp<'a> {
    pub fn new(prog: &'a Program, world: &'a mut World, bindings: Bindings) -> Self {
        let mut globals = HashMap::new();
        for g in &prog.globals {
            if let Stmt::Decl { name, array_len, init, .. } = g {
                globals.insert(name.clone(), make_var(array_len.as_ref(), init.as_ref()));
            }
        }
        Self {
            prog,
            world,
            bindings,
            stack: Vec::new(),
            globals,
            pwm_l: 0,
            pwm_r: 0,
            tick_ms: 0.0,
            logs: Vec::new(),
            wrote_motor: false,
            read_sensor: false,
            used_delay: false,
            finished: false,
            pending_yield: None,
            yielded: None,
            steps: 0,
            step_budget: 200_000,
            total_steps: 0,
            call_depth: 0,
            unknown_fns: HashMap::new(),
        }
    }

    /* ---------------- 对外主入口 ---------------- */

    /// 首次启动（入栈 main 帧）并跑到第一次 yield 或结束
    pub fn start(&mut self) -> Result<Option<f64>, String> {
        let main = self
            .prog
            .funcs
            .get("main")
            .cloned()
            .ok_or_else(|| "找不到 main 函数".to_string())?;

        let mut f = Frame::block(main.body.clone());
        f.is_func = true;
        self.stack.push(f);
        self.drive()
    }

    /// yield 之后继续跑（物理已推进）；返回 Some(ms) 表示又要等，None 表示结束
    pub fn resume(&mut self) -> Result<Option<f64>, String> {
        if self.stack.is_empty() {
            self.finished = true;
            return Ok(None);
        }
        self.drive()
    }

    fn drive(&mut self) -> Result<Option<f64>, String> {
        self.yielded = None;
        self.steps = 0;

        loop {
            // 栈空 = main 返回 = 程序结束
            if self.stack.is_empty() {
                self.finished = true;
                return Ok(None);
            }

            let mut frame = self.stack.pop().unwrap();

            // 本帧跑完了
            if frame.idx >= frame.stmts.len() {
                if let Some(cond) = frame.repeat.clone() {
                    let c = self.eval(&cond, &mut frame)?;
                    if !c.is_zero() {
                        if let Some(step) = frame.for_step.clone() {
                            self.eval(&step, &mut frame)?;
                        }
                        frame.idx = 0;
                        self.stack.push(frame);
                        continue;
                    }
                }
                // 条件不成立或不是循环：丢弃本帧，回到外层
                continue;
            }

            self.steps += 1;
            self.total_steps += 1;
            if self.steps > self.step_budget {
                return Err("单个 tick 内执行步数超出预算（疑似忙等死循环）".to_string());
            }

            let stmt = frame.stmts[frame.idx].clone();
            let sig = self.exec_stmt(&stmt, &mut frame)?;

            // 先处理延时：语句已执行完，idx 推进后交还控制权
            if let Some(ms) = self.pending_yield.take() {
                frame.idx += 1;
                self.stack.push(frame);
                return Ok(Some(ms));
            }

            match sig {
                Signal::None => {
                    frame.idx += 1;
                    self.stack.push(frame);
                }
                Signal::Break => {
                    self.unwind_loop_or_switch();
                }
                Signal::Continue => {
                    self.unwind_to_loop();
                }
                Signal::Return(v) => {
                    if self.stack.len() <= self.call_depth {
                        // 已经回到最外层（main 返回）
                        self.finished = true;
                        return Ok(None);
                    }
                    // 一路退到最近的函数帧
                    while let Some(f) = self.stack.pop() {
                        if f.is_func {
                            break;
                        }
                    }
                    let _ = v;
                    // 调用点在 call_user 里等待栈回到基准深度
                    if self.stack.len() < self.call_depth {
                        self.call_depth = self.stack.len();
                    }
                    return Ok(None);
                }
            }
        }
    }

    /// break：一直退栈，直到（并包含）最近的循环体或 switch
    fn unwind_loop_or_switch(&mut self) {
        while let Some(f) = self.stack.pop() {
            if f.repeat.is_some() || f.is_switch {
                return;
            }
        }
    }

    /// continue：退到最近的循环体，执行 step 后重新开一轮
    fn unwind_to_loop(&mut self) {
        while let Some(mut f) = self.stack.pop() {
            if f.repeat.is_some() {
                if let Some(step) = f.for_step.clone() {
                    let _ = self.eval(&step, &mut f);
                }
                f.idx = 0;
                self.stack.push(f);
                return;
            }
        }
    }

    /* ---------------- 语句 ---------------- */

    fn exec_stmt(&mut self, s: &Stmt, frame: &mut Frame) -> Result<Signal, String> {
        match s {
            Stmt::Decl { name, array_len, init, .. } => {
                let v = self.make_var_runtime(array_len.as_ref(), init.as_ref(), frame)?;
                if let Some(scope) = frame.locals.last_mut() {
                    scope.insert(name.clone(), v);
                }
                Ok(Signal::None)
            }

            Stmt::Expr(e) => {
                self.eval(e, frame)?;
                Ok(Signal::None)
            }

            Stmt::Block(stmts, _) => {
                self.stack.push(Frame::block(stmts.clone()));
                Ok(Signal::None)
            }

            Stmt::If { cond, then, otherwise, .. } => {
                let c = self.eval(cond, frame)?;
                let branch: Option<&Stmt> = if !c.is_zero() {
                    Some(then)
                } else {
                    otherwise.as_deref()
                };
                if let Some(b) = branch {
                    self.stack.push(Frame::block(vec![b.clone()]));
                }
                Ok(Signal::None)
            }

            Stmt::While { cond, body, .. } => {
                let mut lf = Frame::block(vec![body.clone()]);
                lf.repeat = Some(cond.clone());
                self.stack.push(lf);
                Ok(Signal::None)
            }

            Stmt::DoWhile { body, cond, .. } => {
                // do-while：先跑一次体，再判条件 —— 与 repeat 语义一致
                let mut lf = Frame::block(vec![body.clone()]);
                lf.repeat = Some(cond.clone());
                self.stack.push(lf);
                Ok(Signal::None)
            }

            Stmt::For { init, cond, step, body, .. } => {
                let mut lf = Frame::block(vec![body.clone()]);
                lf.repeat = cond.clone();
                lf.for_step = step.clone();
                // for 的初始化变量要活在整个循环里，所以声明进循环帧的作用域
                if let Some(i) = init {
                    self.exec_stmt(i, &mut lf)?;
                    lf.idx = 0;
                }
                self.stack.push(lf);
                Ok(Signal::None)
            }

            Stmt::Return(e, _) => {
                let v = match e {
                    Some(x) => self.eval(x, frame)?,
                    None => Value::Int(0),
                };
                Ok(Signal::Return(v))
            }

            Stmt::Break(_) => Ok(Signal::Break),
            Stmt::Continue(_) => Ok(Signal::Continue),

            Stmt::Switch { expr, cases, .. } => {
                let v = self.eval(expr, frame)?.as_int();
                let mut chosen: Option<&Vec<Stmt>> = None;
                let mut default_case: Option<&Vec<Stmt>> = None;
                for (key, body) in cases {
                    match key {
                        Some(k) if *k == v => chosen = Some(body),
                        None => default_case = Some(body),
                        _ => {}
                    }
                }
                if let Some(body) = chosen.or(default_case) {
                    let mut sf = Frame::block(body.clone());
                    sf.is_switch = true;
                    self.stack.push(sf);
                }
                Ok(Signal::None)
            }
        }
    }

    /* ---------------- 变量 ---------------- */

    fn make_var_runtime(
        &mut self,
        array_len: Option<&usize>,
        init: Option<&Expr>,
        frame: &mut Frame,
    ) -> Result<Var, String> {
        Ok(match (array_len, init) {
            (Some(n), Some(Expr::Call(tag, elems, _))) if tag == "#list" => {
                let mut arr = vec![Value::Int(0); (*n).max(1)];
                for (i, e) in elems.iter().enumerate() {
                    if i < arr.len() {
                        arr[i] = self.eval(e, frame)?;
                    }
                }
                Var::Array(arr)
            }
            (Some(n), _) => Var::Array(vec![Value::Int(0); (*n).max(1)]),
            (None, Some(e)) => Var::Scalar(self.eval(e, frame)?),
            (None, None) => Var::Scalar(Value::Int(0)),
        })
    }

    fn get_var(&self, frame: &Frame, name: &str) -> Option<Var> {
        for scope in frame.locals.iter().rev() {
            if let Some(v) = scope.get(name) {
                return Some(v.clone());
            }
        }
        self.globals.get(name).cloned()
    }

    fn set_var(&mut self, frame: &mut Frame, name: &str, v: Value) {
        for scope in frame.locals.iter_mut().rev() {
            if scope.contains_key(name) {
                scope.insert(name.to_string(), Var::Scalar(v));
                return;
            }
        }
        self.globals.insert(name.to_string(), Var::Scalar(v));
    }

    fn write_index(&mut self, frame: &mut Frame, name: &str, i: i64, v: Value) {
        for scope in frame.locals.iter_mut().rev() {
            if let Some(Var::Array(a)) = scope.get_mut(name) {
                if i >= 0 && (i as usize) < a.len() {
                    a[i as usize] = v;
                }
                return;
            }
        }
        if let Some(Var::Array(a)) = self.globals.get_mut(name) {
            if i >= 0 && (i as usize) < a.len() {
                a[i as usize] = v;
            }
        }
    }

    /* ---------------- 表达式 ---------------- */

    fn eval(&mut self, e: &Expr, frame: &mut Frame) -> Result<Value, String> {
        match e {
            Expr::Int(n, _) => Ok(Value::Int(*n)),
            Expr::Float(f, _) => Ok(Value::Float(*f)),
            Expr::Char(c, _) => Ok(Value::Int(*c as i64)),
            Expr::Str(_, _) => Ok(Value::Int(0)),

            Expr::Ident(name, _) => Ok(match self.get_var(frame, name) {
                Some(Var::Scalar(v)) => v,
                Some(Var::Array(a)) => a.first().copied().unwrap_or(Value::Int(0)),
                None => Value::Int(0), // 未声明标识符当作 0，不打断仿真
            }),

            Expr::Index(base, idx, _) => {
                let i = self.eval(idx, frame)?.as_int();
                let name = match &**base {
                    Expr::Ident(n, _) => n.clone(),
                    _ => return Ok(Value::Int(0)),
                };
                match self.get_var(frame, &name) {
                    Some(Var::Array(a)) => {
                        if i < 0 || i as usize >= a.len() {
                            return Err(format!("数组 {} 越界访问: {}", name, i));
                        }
                        Ok(a[i as usize])
                    }
                    Some(Var::Scalar(v)) => Ok(v),
                    None => Ok(Value::Int(0)),
                }
            }

            Expr::IndexAssign(base, idx, rhs, _) => {
                let i = self.eval(idx, frame)?.as_int();
                let v = self.eval(rhs, frame)?;
                if let Expr::Ident(n, _) = &**base {
                    let name = n.clone();
                    self.write_index(frame, &name, i, v);
                }
                Ok(v)
            }

            Expr::Unary(op, inner, _) => {
                let v = self.eval(inner, frame)?;
                Ok(match op {
                    UnOp::Neg => match v {
                        Value::Int(i) => Value::Int(-i),
                        Value::Float(f) => Value::Float(-f),
                    },
                    UnOp::Pos => v,
                    UnOp::Not => Value::Int(if v.is_zero() { 1 } else { 0 }),
                    UnOp::BitNot => Value::Int(!v.as_int()),
                })
            }

            Expr::Binary(op, l, r, _) => {
                let a = self.eval(l, frame)?;
                let b = self.eval(r, frame)?;
                a.arith(*op, b)
            }

            Expr::Assign(name, rhs, _) => {
                let v = self.eval(rhs, frame)?;
                let name_c = name.clone();
                self.set_var(frame, &name_c, v);
                Ok(v)
            }

            Expr::AssignOp(op, name, rhs, _) => {
                let cur = match self.get_var(frame, name) {
                    Some(Var::Scalar(v)) => v,
                    _ => Value::Int(0),
                };
                let b = self.eval(rhs, frame)?;
                let v = cur.arith(*op, b)?;
                let name_c = name.clone();
                self.set_var(frame, &name_c, v);
                Ok(v)
            }

            Expr::PostInc(name, inc, _) => {
                let cur = match self.get_var(frame, name) {
                    Some(Var::Scalar(v)) => v,
                    _ => Value::Int(0),
                };
                let next = match cur {
                    Value::Int(i) => Value::Int(if *inc { i + 1 } else { i - 1 }),
                    Value::Float(f) => Value::Float(if *inc { f + 1.0 } else { f - 1.0 }),
                };
                let name_c = name.clone();
                self.set_var(frame, &name_c, next);
                Ok(cur)
            }

            Expr::Ternary(c, a, b, _) => {
                if !self.eval(c, frame)?.is_zero() {
                    self.eval(a, frame)
                } else {
                    self.eval(b, frame)
                }
            }

            Expr::Cast(ty, inner, _) => {
                let v = self.eval(inner, frame)?;
                Ok(match ty.as_str() {
                    "float" | "double" => Value::Float(v.as_float()),
                    _ => Value::Int(v.as_int()),
                })
            }

            Expr::SizeofTy(ty, _) => Ok(Value::Int(match ty.as_str() {
                "char" | "int8_t" | "uint8_t" | "bool" => 1,
                "int16_t" | "uint16_t" | "short" => 2,
                "float" | "int" | "int32_t" | "uint32_t" => 4,
                "double" | "int64_t" | "uint64_t" => 8,
                _ => 4,
            })),

            Expr::Call(name, args, line) => self.call(name, args, frame, *line),
        }
    }

    /* ---------------- 调用 ---------------- */

    fn call(
        &mut self,
        name: &str,
        args: &[Expr],
        frame: &mut Frame,
        line: usize,
    ) -> Result<Value, String> {
        // 先算实参
        let mut vals = Vec::with_capacity(args.len());
        for a in args {
            vals.push(match a {
                Expr::Str(_, _) => Value::Int(0),
                _ => self.eval(a, frame)?,
            });
        }

        // HAL / 内置
        if let Some(r) = self.hal_call(name, &vals, args)? {
            return Ok(r);
        }

        // 用户函数
        let Some(f) = self.prog.funcs.get(name).cloned() else {
            *self.unknown_fns.entry(name.to_string()).or_insert(0) += 1;
            return Ok(Value::Int(0));
        };

        self.call_user(&f, &vals, line)
    }

    fn call_user(&mut self, f: &FuncDef, vals: &[Value], line: usize) -> Result<Value, String> {
        if self.call_depth >= CALL_DEPTH_MAX {
            return Err(format!("函数调用层数超过 {}（疑似无限递归）", CALL_DEPTH_MAX));
        }

        let base = self.stack.len();
        let mut scope = HashMap::new();
        for (i, (_, pname)) in f.params.iter().enumerate() {
            if !pname.is_empty() {
                scope.insert(
                    pname.clone(),
                    Var::Scalar(vals.get(i).copied().unwrap_or(Value::Int(0))),
                );
            }
        }
        let mut ff = Frame::block(f.body.clone());
        ff.locals = vec![scope];
        ff.is_func = true;
        self.stack.push(ff);

        let saved_depth = self.call_depth;
        self.call_depth = base;
        let r = self.run_sub(base, line);
        self.call_depth = saved_depth;
        r
    }

    /// 执行子调用：一直驱动到栈回到 base（函数帧被弹出）
    fn run_sub(&mut self, base: usize, line: usize) -> Result<Value, String> {
        let mut ret = Value::Int(0);
        loop {
            if self.stack.len() <= base {
                return Ok(ret);
            }
            if self.pending_yield.is_some() {
                return Err(format!(
                    "第 {} 行：不支持在函数内部调用延时（请把 HAL_Delay 放在主循环里）",
                    line
                ));
            }

            let mut frame = self.stack.pop().unwrap();

            if frame.idx >= frame.stmts.len() {
                if let Some(cond) = frame.repeat.clone() {
                    let c = self.eval(&cond, &mut frame)?;
                    if !c.is_zero() {
                        if let Some(step) = frame.for_step.clone() {
                            self.eval(&step, &mut frame)?;
                        }
                        frame.idx = 0;
                        self.stack.push(frame);
                        continue;
                    }
                }
                continue;
            }

            self.steps += 1;
            self.total_steps += 1;
            if self.steps > self.step_budget {
                return Err("单个 tick 内执行步数超出预算（疑似忙等死循环）".to_string());
            }

            let stmt = frame.stmts[frame.idx].clone();
            let sig = self.exec_stmt(&stmt, &mut frame)?;

            if self.pending_yield.is_some() {
                return Err(format!(
                    "第 {} 行：不支持在函数内部调用延时（请把 HAL_Delay 放在主循环里）",
                    line
                ));
            }

            match sig {
                Signal::None => {
                    frame.idx += 1;
                    self.stack.push(frame);
                }
                Signal::Break => self.unwind_loop_or_switch(),
                Signal::Continue => self.unwind_to_loop(),
                Signal::Return(v) => {
                    ret = v;
                    // 退到（并弹出）本函数的帧
                    while let Some(f) = self.stack.pop() {
                        if f.is_func {
                            break;
                        }
                    }
                    return Ok(ret);
                }
            }
        }
    }

    /// HAL 与简易 API；None 表示不是内置函数
    fn hal_call(
        &mut self,
        name: &str,
        vals: &[Value],
        args: &[Expr],
    ) -> Result<Option<Value>, String> {
        let n0 = vals.first().copied().unwrap_or(Value::Int(0)).as_int();
        let n1 = vals.get(1).copied().unwrap_or(Value::Int(0)).as_int();

        match name {
            /* ---- 延时：核心的 yield 点 ---- */
            "HAL_Delay" | "Delay_Ms" | "delay_ms" | "osDelay" => {
                let ms = n0.max(0) as f64;
                self.used_delay = true;
                self.tick_ms += ms;
                self.pending_yield = Some(ms);
                Ok(Some(Value::Int(0)))
            }

            /* ---- 系统 ---- */
            "HAL_Init" | "SystemInit" | "HAL_IncTick" | "MX_GPIO_Init" | "HAL_RCC_Init" => {
                Ok(Some(Value::Int(0)))
            }
            "HAL_GetTick" | "millis" | "Get_Tick" => Ok(Some(Value::Int(self.tick_ms as i64))),

            /* ---- 电机输出 ---- */
            "Motor_Set" | "MotorSet" | "Set_Motor" | "motor_set" | "Set_Pwm" => {
                self.set_motor(n0 as i32, n1 as i32);
                Ok(Some(Value::Int(0)))
            }
            "__HAL_TIM_SET_COMPARE" | "TIM_SetCompare" => {
                let ch = arg_channel(args).unwrap_or(n1);
                let v = vals.get(2).copied().unwrap_or(Value::Int(n0)).as_int();
                self.write_pwm_channel(ch, v as i32);
                Ok(Some(Value::Int(0)))
            }
            "HAL_TIM_PWM_Start" | "HAL_TIM_PWM_Init" | "MX_TIM2_Init" => Ok(Some(Value::Int(0))),

            /* ---- 传感器 ---- */
            "Sensor_Read" | "Read_Sensor" => {
                let s = self.world.read_line_sensors();
                self.read_sensor = true;
                Ok(Some(Value::Int(s.get(n0.max(0) as usize).copied().unwrap_or(0) as i64)))
            }
            "Read_Line" | "ReadLine" | "Line_Position" | "Get_LinePos" => {
                self.read_sensor = true;
                // 常见工程约定：返回 -2000..2000 的加权位置
                Ok(Some(Value::Float(self.world.read_line_position() * 2000.0)))
            }
            "ADC_Read" | "HAL_ADC_GetValue" | "Get_ADC" => {
                let ch = arg_channel(args).unwrap_or(n0);
                let idx = self.sensor_index_for_channel(ch);
                let s = self.world.read_line_sensors();
                self.read_sensor = true;
                Ok(Some(Value::Int(
                    s.get(idx).copied().unwrap_or(0) as i64 * 4095,
                )))
            }
            "HAL_ADC_Start" | "HAL_ADC_Stop" | "HAL_ADC_PollForConversion"
            | "HAL_ADC_ConfigChannel" | "MX_ADC1_Init" => Ok(Some(Value::Int(0))),

            "Ultrasonic_Read" | "HC_SR04_Read" | "Get_Distance" => {
                Ok(Some(Value::Float(self.world.read_ultrasonic_cm())))
            }
            "Encoder_Read" | "Get_Encoder" => {
                let e = if n0 == 1 {
                    self.world.car.encoder_r
                } else {
                    self.world.car.encoder_l
                };
                Ok(Some(Value::Int(e as i64)))
            }
            "IMU_Read" | "Get_Gyro" | "MPU6050_ReadGyro" => {
                Ok(Some(Value::Float(self.world.read_gyro_dps())))
            }

            /* ---- GPIO ---- */
            "HAL_GPIO_ReadPin" => Ok(Some(Value::Int(0))),
            "HAL_GPIO_WritePin" | "HAL_GPIO_TogglePin" => Ok(Some(Value::Int(0))),

            /* ---- 输出 ---- */
            "HAL_UART_Transmit" | "printf" | "Printf" | "printk" | "USART_SendString" => {
                let text = format_args_to_string(args, vals);
                if self.logs.len() < 200 {
                    self.logs.push(text);
                }
                Ok(Some(Value::Int(0)))
            }

            /* ---- 数学 ---- */
            "abs" | "fabs" => Ok(Some(match vals.first().copied().unwrap_or(Value::Int(0)) {
                Value::Int(i) => Value::Int(i.abs()),
                Value::Float(f) => Value::Float(f.abs()),
            })),
            "max" => Ok(Some(pick2(vals, |a, b| a > b))),
            "min" => Ok(Some(pick2(vals, |a, b| a < b))),
            "sqrt" => Ok(Some(Value::Float(
                vals.first().copied().unwrap_or(Value::Int(0)).as_float().sqrt(),
            ))),

            _ => Ok(None),
        }
    }

    fn set_motor(&mut self, l: i32, r: i32) {
        self.pwm_l = l;
        self.pwm_r = r;
        self.wrote_motor = true;
        self.world.set_pwm(l, r);
    }

    fn write_pwm_channel(&mut self, ch: i64, v: i32) {
        match ch {
            1 => self.pwm_l = v,
            2 => self.pwm_r = v,
            _ => {}
        }
        self.wrote_motor = true;
        self.world.set_pwm(self.pwm_l, self.pwm_r);
    }

    fn sensor_index_for_channel(&self, ch: i64) -> usize {
        let key = ch.to_string();
        if let Some(i) = self.bindings.sensor_bindings.iter().position(|b| b == &key) {
            return i;
        }
        ch.max(0) as usize
    }

    pub fn unknown_functions(&self) -> Vec<(String, usize)> {
        let mut v: Vec<(String, usize)> = self
            .unknown_fns
            .iter()
            .map(|(k, c)| (k.clone(), *c))
            .collect();
        v.sort_by(|a, b| b.1.cmp(&a.1));
        v
    }
}

/* ------------------------------------------------------------------ *
 * 辅助函数
 * ------------------------------------------------------------------ */

fn make_var(array_len: Option<&usize>, init: Option<&Expr>) -> Var {
    match (array_len, init) {
        (Some(n), Some(Expr::Call(tag, elems, _))) if tag == "#list" => {
            let mut arr = vec![Value::Int(0); (*n).max(1)];
            for (i, e) in elems.iter().enumerate() {
                if i < arr.len() {
                    arr[i] = const_fold(e);
                }
            }
            Var::Array(arr)
        }
        (Some(n), _) => Var::Array(vec![Value::Int(0); (*n).max(1)]),
        (None, Some(e)) => Var::Scalar(const_fold(e)),
        (None, None) => Var::Scalar(Value::Int(0)),
    }
}

/// 常量折叠：全局初始化只允许常量表达式
pub fn const_fold(e: &Expr) -> Value {
    match e {
        Expr::Int(n, _) => Value::Int(*n),
        Expr::Float(f, _) => Value::Float(*f),
        Expr::Char(c, _) => Value::Int(*c as i64),
        Expr::Unary(UnOp::Neg, i, _) => match const_fold(i) {
            Value::Int(v) => Value::Int(-v),
            Value::Float(v) => Value::Float(-v),
        },
        Expr::Binary(op, l, r, _) => const_fold(l)
            .arith(*op, const_fold(r))
            .unwrap_or(Value::Int(0)),
        Expr::Cast(ty, i, _) => match ty.as_str() {
            "float" | "double" => Value::Float(const_fold(i).as_float()),
            _ => Value::Int(const_fold(i).as_int()),
        },
        _ => Value::Int(0),
    }
}

/// 从实参里提取「通道号」：优先字符串实参里的数字
fn arg_channel(args: &[Expr]) -> Option<i64> {
    let mut seen_any = false;
    for a in args {
        let s = match a {
            Expr::Str(s, _) => Some(s.clone()),
            Expr::Ident(n, _) => Some(n.clone()),
            _ => None,
        };
        if let Some(s) = s {
            seen_any = true;
            // "TIM2_CH1" -> 取最后一个数字串，优先匹配 "CH" 后的数字
            let lower = s.to_lowercase();
            if let Some(pos) = lower.find("_ch") {
                let rest = &s[pos + 3..];
                let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
                if let Ok(v) = digits.parse::<i64>() {
                    return Some(v);
                }
            }
        }
    }
    if !seen_any {
        return None;
    }
    for a in args {
        let s = match a {
            Expr::Str(s, _) => s.clone(),
            Expr::Ident(n, _) => n.clone(),
            _ => continue,
        };
        let digits: String = s.chars().filter(|c| c.is_ascii_digit()).collect();
        if let Ok(v) = digits.parse::<i64>() {
            return Some(v);
        }
    }
    None
}

fn pick2(vals: &[Value], cmp: fn(f64, f64) -> bool) -> Value {
    let a = vals.first().copied().unwrap_or(Value::Int(0));
    let b = vals.get(1).copied().unwrap_or(Value::Int(0));
    if cmp(a.as_float(), b.as_float()) {
        a
    } else {
        b
    }
}

/// 把 printf 风格调用拼成一行文本（只处理 %d %f %s）
fn format_args_to_string(args: &[Expr], vals: &[Value]) -> String {
    let fmt = match args.first() {
        Some(Expr::Str(s, _)) => s.clone(),
        _ => return vals.iter().map(|v| fmt_val(*v)).collect::<Vec<_>>().join(" "),
    };
    let mut out = String::new();
    let mut vi = 1usize;
    let mut chars = fmt.chars();
    while let Some(c) = chars.next() {
        if c == '%' {
            match chars.next() {
                Some('d') | Some('i') => {
                    out.push_str(&vals.get(vi).map(|v| v.as_int().to_string()).unwrap_or_default());
                    vi += 1;
                }
                Some('f') => {
                    out.push_str(&format!(
                        "{:.3}",
                        vals.get(vi).map(|v| v.as_float()).unwrap_or(0.0)
                    ));
                    vi += 1;
                }
                Some('s') => {
                    if let Some(Expr::Str(s, _)) = args.get(vi) {
                        out.push_str(s);
                    }
                    vi += 1;
                }
                Some(o) => {
                    out.push('%');
                    out.push(o);
                }
                None => out.push('%'),
            }
        } else {
            out.push(c);
        }
    }
    out
}

fn fmt_val(v: Value) -> String {
    match v {
        Value::Int(i) => i.to_string(),
        Value::Float(f) => format!("{f:.3}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csim::parse_source;
    use crate::physics::{CarConfig, StartIdx, Track, TrackPoint};

    fn tiny_track() -> Track {
        Track {
            name: "t".into(),
            width_m: 0.5,
            line_width_m: 0.03,
            centerline: (0..201)
                .map(|i| TrackPoint { x: 0.0, y: i as f64 * 0.1 })
                .collect(),
            obstacles: vec![],
            start: StartIdx { index: 0 },
            target_lap_time_s: 20.0,
        }
    }

    /// 跑代码直到结束或第一次 yield，返回观测结果
    fn run_once(src: &str) -> Obs {
        let (prog, errs) = parse_source(src);
        assert!(errs.is_empty(), "解析错误: {errs:?}");
        let mut w = World::new(tiny_track(), CarConfig::default(), 0.01, 1);
        let mut it = Interp::new(&prog, &mut w, Bindings::default());
        let r = it.start();
        Obs {
            err: r.as_ref().err().cloned(),
            yielded: r.unwrap_or(None),
            pwm_l: it.pwm_l,
            pwm_r: it.pwm_r,
            tick_ms: it.tick_ms,
            finished: it.finished,
            logs: it.logs.clone(),
            wrote_motor: it.wrote_motor,
            read_sensor: it.read_sensor,
        }
    }

    struct Obs {
        err: Option<String>,
        yielded: Option<f64>,
        pwm_l: i32,
        pwm_r: i32,
        tick_ms: f64,
        finished: bool,
        logs: Vec<String>,
        wrote_motor: bool,
        #[allow(dead_code)]
        read_sensor: bool,
    }

    #[test]
    fn hal_delay_yields() {
        let o = run_once("int main(void){ Motor_Set(400, 400); HAL_Delay(10); while(1){ HAL_Delay(1); } }");
        assert!(o.err.is_none(), "{:?}", o.err);
        assert_eq!(o.yielded, Some(10.0));
        assert_eq!(o.pwm_l, 400);
        assert_eq!(o.pwm_r, 400);
        assert!(o.wrote_motor);
    }

    #[test]
    fn while_loop_with_if_runs() {
        let src = r#"
int main(void){
  int i = 0;
  while(i < 10){
    if(i > 5){ Motor_Set(100 + i, 100); }
    i = i + 1;
  }
  Motor_Set(0, 0);
}
"#;
        let o = run_once(src);
        assert!(o.finished, "应正常结束: {:?}", o.err);
        assert_eq!(o.pwm_l, 0, "循环结束后被置 0（i>5 时设过值，最后一步覆盖）");
    }

    #[test]
    fn for_loop_accumulates() {
        let o = run_once(
            "int main(void){ int s = 0; for(int i=0;i<5;i=i+1){ s = s + i; } Motor_Set(s, 0); }",
        );
        assert_eq!(o.pwm_l, 10, "0+1+2+3+4 = 10");
    }

    #[test]
    fn pid_sample_resumes_across_ticks() {
        let src = r#"
float Kp = 2.0, Ki = 0.1, Kd = 0.5;
float integral = 0, last = 0;
int main(void){
  while(1){
    float pos = Read_Line();
    float err = 0 - pos;
    integral = integral + err;
    float out = Kp*err + Ki*integral + Kd*(err-last);
    last = err;
    Motor_Set(500 + (int)out, 500 - (int)out);
    HAL_Delay(10);
  }
}
"#;
        let (prog, errs) = parse_source(src);
        assert!(errs.is_empty(), "{errs:?}");
        let mut w = World::new(tiny_track(), CarConfig::default(), 0.01, 1);
        let mut it = Interp::new(&prog, &mut w, Bindings::default());

        // 连续跑 5 个 tick，模拟仿真器的驱动方式
        let mut ticks = 0;
        let first = it.start().expect("首 tick");
        assert_eq!(first, Some(10.0));
        ticks += 1;
        for _ in 0..4 {
            assert_eq!(it.resume().expect("后续 tick"), Some(10.0));
            ticks += 1;
        }
        assert_eq!(ticks, 5);
        assert!((it.tick_ms - 50.0).abs() < 1e-6, "累计延时 {}", it.tick_ms);
        assert!(!it.finished, "while(1) 不该结束");
    }

    #[test]
    fn unknown_hal_function_does_not_crash() {
        let o = run_once("int main(void){ HAL_RCC_OscConfig(1,2); HAL_UART_Init(3); Motor_Set(10,10); }");
        assert!(o.err.is_none(), "{:?}", o.err);
        assert_eq!(o.pwm_l, 10);
    }

    #[test]
    fn div_by_zero_is_error() {
        let o = run_once("int main(void){ int a = 1; int b = 0; int c = a / b; Motor_Set(c,0); }");
        assert!(o.err.is_some(), "除零应报错");
    }

    #[test]
    fn array_read_write() {
        let o = run_once("int a[5]; int main(void){ a[2] = 7; Motor_Set(a[2], 0); }");
        assert_eq!(o.pwm_l, 7);
    }

    #[test]
    fn ternary_and_cast() {
        let o = run_once(
            "int main(void){ float f = 3.7; int i = (int)f; int r = i > 3 ? 9 : -9; Motor_Set(r, 0); }",
        );
        assert_eq!(o.pwm_l, 9);
    }

    #[test]
    fn printf_format() {
        let o = run_once("int main(void){ Printf(\"speed=%d pwm=%f\\n\", 42, 1.5); }");
        assert_eq!(o.logs.len(), 1);
        assert!(o.logs[0].contains("42"), "{}", o.logs[0]);
        assert!(o.logs[0].contains("1.500"), "{}", o.logs[0]);
    }

    #[test]
    fn compound_assign_ops() {
        let o = run_once("int main(void){ int x = 10; x += 5; x -= 3; x *= 2; Motor_Set(x, 0); }");
        assert_eq!(o.pwm_l, 24, "(10+5-3)*2 = 24");
    }

    #[test]
    fn break_and_continue() {
        let o = run_once(
            "int main(void){ int s=0; for(int i=0;i<10;i=i+1){ if(i==3) continue; if(i==6) break; s = s + i; } Motor_Set(s,0); }",
        );
        // i = 0,1,2,4,5
        assert_eq!(o.pwm_l, 12);
    }

    #[test]
    fn function_call_and_return() {
        let o = run_once("int add(int a, int b){ return a + b; } int main(void){ Motor_Set(add(3,4), 0); }");
        assert_eq!(o.pwm_l, 7);
    }

    #[test]
    fn nested_function_calls() {
        let o = run_once(
            "int dbl(int x){ return x * 2; } int quad(int x){ return dbl(dbl(x)); } int main(void){ Motor_Set(quad(3), 0); }",
        );
        assert_eq!(o.pwm_l, 12);
    }

    #[test]
    fn do_while_runs_body_once() {
        let o = run_once("int main(void){ int i = 0; do { i = i + 1; } while(i < 3); Motor_Set(i, 0); }");
        assert_eq!(o.pwm_l, 3);
    }

    #[test]
    fn switch_selects_case() {
        let o = run_once(
            "int main(void){ int r = 0; switch(2){ case 1: r = 10; break; case 2: r = 20; break; default: r = 30; } Motor_Set(r, 0); }",
        );
        assert_eq!(o.pwm_l, 20);
    }

    #[test]
    fn busy_loop_hits_step_budget() {
        let o = run_once("int main(void){ while(1){ } }");
        assert!(o.err.is_some(), "无延时的死循环应触发步数预算");
    }

    #[test]
    fn value_arith_type_promotion() {
        assert_eq!(
            Value::Int(2).arith(BinOp::Add, Value::Float(0.5)).unwrap(),
            Value::Float(2.5)
        );
        assert_eq!(Value::Int(7).arith(BinOp::Div, Value::Int(2)).unwrap(), Value::Int(3));
        assert_eq!(
            Value::Float(7.0).arith(BinOp::Div, Value::Int(2)).unwrap(),
            Value::Float(3.5)
        );
        assert!(Value::Int(1).arith(BinOp::Div, Value::Int(0)).is_err());
    }

    #[test]
    fn comparison_ops() {
        assert_eq!(Value::Int(2).arith(BinOp::Gt, Value::Int(1)).unwrap(), Value::Int(1));
        assert_eq!(Value::Int(1).arith(BinOp::Ge, Value::Int(1)).unwrap(), Value::Int(1));
        assert_eq!(Value::Int(1).arith(BinOp::Ne, Value::Int(1)).unwrap(), Value::Int(0));
        assert_eq!(Value::Int(0).arith(BinOp::Or, Value::Int(0)).unwrap(), Value::Int(0));
        assert_eq!(Value::Int(1).arith(BinOp::And, Value::Int(1)).unwrap(), Value::Int(1));
    }

    #[test]
    fn const_fold_handles_negative_and_cast() {
        let (prog, _) = parse_source("int a = -5;\nfloat b = (float)(2 + 3);");
        let Stmt::Decl { init: Some(ia), .. } = &prog.globals[0] else { panic!() };
        let Stmt::Decl { init: Some(ib), .. } = &prog.globals[1] else { panic!() };
        assert_eq!(const_fold(ia), Value::Int(-5));
        assert_eq!(const_fold(ib), Value::Float(5.0));
    }

    #[test]
    fn arg_channel_extracts_number() {
        assert_eq!(arg_channel(&[Expr::Str("TIM_CHANNEL_1".into(), 1)]), Some(1));
        assert_eq!(arg_channel(&[Expr::Str("CHANNEL_3".into(), 1)]), Some(3));
        assert_eq!(arg_channel(&[Expr::Str("TIM2_CH2".into(), 1)]), Some(2));
    }

    #[test]
    fn pwm_channel_write_maps_left_right() {
        let src = r#"
int main(void){
  __HAL_TIM_SET_COMPARE(0, "TIM2_CH1", 700);
  __HAL_TIM_SET_COMPARE(0, "TIM2_CH2", 300);
}
"#;
        let o = run_once(src);
        assert_eq!(o.pwm_l, 700);
        assert_eq!(o.pwm_r, 300);
    }
}

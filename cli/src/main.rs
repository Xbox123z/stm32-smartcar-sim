//! car-sim —— STM32 智能车仿真命令行工具
//!
//! 依赖只有标准库 + car-sim-core，参数解析自己手写（保持零额外依赖）。

mod args;
mod output;

use std::process::ExitCode;

use car_sim_core::sim::{self, RunOptions};
use car_sim_core::{lint, Track};

const VERSION: &str = env!("CARGO_PKG_VERSION");

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().collect();
    let a = args::Args::parse(&argv);

    match a.command.as_deref() {
        Some("check") => cmd_check(&a),
        Some("run") => cmd_run(&a),
        Some("bench") => cmd_bench(&a),
        Some("list-tracks") => cmd_list_tracks(&a),
        Some("demo") => cmd_demo(&a),
        Some("version") | Some("--version") => {
            println!("car-sim v{VERSION}");
            ExitCode::SUCCESS
        }
        _ => {
            print_help();
            if a.command.is_some() {
                eprintln!("未知命令: {}", a.command.as_deref().unwrap_or(""));
                return ExitCode::from(64);
            }
            ExitCode::SUCCESS
        }
    }
}

/* ------------------------------------------------------------------ *
 * 子命令
 * ------------------------------------------------------------------ */

fn cmd_check(a: &args::Args) -> ExitCode {
    let Some(file) = a.free.first() else {
        eprintln!("用法: car-sim check <file.c>");
        return ExitCode::from(64);
    };
    let src = match std::fs::read_to_string(file) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("读不了 {}：{}", file, e);
            return ExitCode::from(66);
        }
    };

    let report = lint::lint(&src);

    if a.json {
        // 只输出静态部分，保持与 run 报告的 static 段结构一致
        let mut m = serde_json_like::Map::new();
        m.insert("file".into(), serde_json_like::Value::Str(file.clone()));
        let (e, w, i) = lint::count(&report.issues);
        m.insert("errors".into(), serde_json_like::Value::Num(e as f64));
        m.insert("warnings".into(), serde_json_like::Value::Num(w as f64));
        m.insert("infos".into(), serde_json_like::Value::Num(i as f64));
        m.insert("issues".into(), serde_json_like::Value::Arr(
            report.issues.iter().map(serde_json_like::issue).collect(),
        ));
        println!("{}", serde_json_like::stringify(&serde_json_like::Value::Obj(m)));
    } else {
        println!("📄 {}", file);
        println!();
        print!("{}", lint::render_text(&report));
    }

    if lint::has_error(&report) {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}

fn cmd_run(a: &args::Args) -> ExitCode {
    let Some(file) = a.free.first() else {
        eprintln!("用法: car-sim run <file.c> [选项]");
        return ExitCode::from(64);
    };
    let src = match std::fs::read_to_string(file) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("读不了 {}：{}", file, e);
            return ExitCode::from(66);
        }
    };

    let lint_report = lint::lint(&src);

    // 有 error 级问题：默认拒绝仿真（--force 可以继续）
    if lint::has_error(&report_of(&lint_report)) && !a.force {
        if a.json {
            println!("{{\"error\":\"静态检查未通过\",\"blocked\":true}}");
        } else {
            eprintln!("❌ 静态检查有必须修的错误，拒绝仿真（加 --force 强制执行）");
            eprintln!();
            print!("{}", lint::render_text(&lint_report));
        }
        return ExitCode::from(1);
    }

    let track = match load_track(&a.track) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("{}", e);
            return ExitCode::from(66);
        }
    };

    let opts = RunOptions {
        track,
        dt: a.dt,
        time_limit_s: a.time_limit,
        seed: a.seed,
        sample_every: a.sample_every,
        ..Default::default()
    };

    let report = sim::run_with_lint(&src, file, opts, report_of(&lint_report).clone());

    // 写 JSON
    if let Some(out) = &a.out {
        if let Err(e) = std::fs::write(out, report_json(&report)) {
            eprintln!("写 {} 失败：{}", out, e);
        } else if !a.quiet {
            println!("💾 报告已写入 {}", out);
        }
    }
    if let Some(csv) = &a.csv {
        if let Err(e) = std::fs::write(csv, sim::telemetry_csv(&report)) {
            eprintln!("写 {} 失败：{}", csv, e);
        } else if !a.quiet {
            println!("📈 遥测已写入 {}", csv);
        }
    }

    if a.json {
        println!("{}", report_json(&report));
    } else if !a.quiet {
        print!("{}", output::render_report(&report));
    }

    match report.sim.result {
        sim::RunResult::Finished => ExitCode::SUCCESS,
        sim::RunResult::Crashed => ExitCode::from(2),
        sim::RunResult::Timeout => ExitCode::from(3),
        sim::RunResult::Failed => ExitCode::from(4),
    }
}

fn cmd_bench(a: &args::Args) -> ExitCode {
    let Some(file) = a.free.first() else {
        eprintln!("用法: car-sim bench <file.c>");
        return ExitCode::from(64);
    };
    let src = match std::fs::read_to_string(file) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("读不了 {}：{}", file, e);
            return ExitCode::from(66);
        }
    };

    let tracks = bundled_tracks();
    let mut rows: Vec<(String, String, u8, f64, usize)> = Vec::new();

    for (name, t) in tracks {
        let opts = RunOptions {
            track: t,
            dt: a.dt,
            time_limit_s: a.time_limit,
            seed: a.seed,
            ..Default::default()
        };
        let r = sim::run_file(&src, file, opts);
        rows.push((
            name,
            r.sim.result.label().to_string(),
            r.score.total,
            r.sim.progress * 100.0,
            r.sim.off_track_events,
        ));
    }

    if a.json {
        let items: Vec<String> = rows
            .iter()
            .map(|(n, res, sc, p, off)| {
                format!(
                    "{{\"track\":\"{}\",\"result\":\"{}\",\"score\":{},\"progress\":{:.1},\"off_track\":{}}}",
                    n, res, sc, p, off
                )
            })
            .collect();
        println!("[{}]", items.join(","));
    } else {
        println!("🏁 {} 的多赛道对比", file);
        println!();
        println!(
            "  {:<10} {:<8} {:<6} {:<10} {}",
            "赛道", "结果", "评分", "进度", "出界次数"
        );
        println!("  {}", "-".repeat(52));
        for (n, res, sc, p, off) in &rows {
            println!("  {:<10} {:<8} {:<6} {:<10.1}% {}", n, res, sc, p, off);
        }
    }

    ExitCode::SUCCESS
}

fn cmd_list_tracks(_a: &args::Args) -> ExitCode {
    for (name, t) in bundled_tracks() {
        println!(
            "  {:<10} 长度 {:>6.2}m  宽 {:.2}m  障碍 {}  目标 {:.0}s",
            name,
            t.length(),
            t.width_m,
            t.obstacles.len(),
            t.target_lap_time_s
        );
    }
    ExitCode::SUCCESS
}

fn cmd_demo(a: &args::Args) -> ExitCode {
    let src = include_str!("../../simcore/examples/good_line_follower.c");
    let opts = RunOptions {
        dt: a.dt,
        time_limit_s: a.time_limit,
        seed: a.seed,
        ..Default::default()
    };
    let report = sim::run_file(src, "simcore/examples/good_line_follower.c", opts);

    if a.json {
        println!("{}", report_json(&report));
    } else {
        println!("🎬 演示：用内置的循迹示例代码跑一遍 track01");
        println!();
        print!("{}", output::render_report(&report));
    }
    ExitCode::SUCCESS
}

/* ------------------------------------------------------------------ *
 * 赛道加载
 * ------------------------------------------------------------------ */

fn load_track(name: &str) -> Result<Track, String> {
    // 先找内置
    for (n, t) in bundled_tracks() {
        if n == name {
            return Ok(t);
        }
    }
    // 再当文件路径
    let p = std::path::Path::new(name);
    if p.is_file() {
        let s = std::fs::read_to_string(p).map_err(|e| e.to_string())?;
        return parse_track(&s).map_err(|e| format!("赛道文件解析失败：{e}"));
    }
    // 也允许 "track01" 不带 .json
    let alt = std::path::Path::new("simcore/tracks").join(format!("{name}.json"));
    if alt.is_file() {
        let s = std::fs::read_to_string(&alt).map_err(|e| e.to_string())?;
        return parse_track(&s).map_err(|e| format!("赛道文件解析失败：{e}"));
    }
    Err(format!(
        "找不到赛道 {name}（内置: track01/track02/track03，也可以是 json 文件路径）"
    ))
}

fn parse_track(s: &str) -> Result<Track, String> {
    // 极简 JSON 解析：只需要 name/width_m/centerline/obstacles/target_lap_time_s
    let mut name = String::from("custom");
    let mut width = 0.5f64;
    let mut line_width = 0.03f64;
    let mut target = 25.0f64;
    let mut centerline: Vec<car_sim_core::physics::TrackPoint> = Vec::new();
    let mut obstacles: Vec<car_sim_core::physics::Obstacle> = Vec::new();

    // 提取 name
    if let Some(v) = json_string_field(s, "name") {
        name = v;
    }
    if let Some(v) = json_number_field(s, "width_m") {
        width = v;
    }
    if let Some(v) = json_number_field(s, "line_width_m") {
        line_width = v;
    }
    if let Some(v) = json_number_field(s, "target_lap_time_s") {
        target = v;
    }

    // centerline: 找 "centerline" 之后的数组
    if let Some(arr) = json_array_after(s, "centerline") {
        for obj in json_objects_in(&arr) {
            let x = json_number_field(&obj, "x").unwrap_or(0.0);
            let y = json_number_field(&obj, "y").unwrap_or(0.0);
            centerline.push(car_sim_core::physics::TrackPoint { x, y });
        }
    }

    // obstacles
    if let Some(arr) = json_array_after(s, "obstacles") {
        for obj in json_objects_in(&arr) {
            let ty = json_string_field(&obj, "type").unwrap_or_else(|| "circle".to_string());
            let x = json_number_field(&obj, "x").unwrap_or(0.0);
            let y = json_number_field(&obj, "y").unwrap_or(0.0);
            if ty == "box" {
                let w = json_number_field(&obj, "w").unwrap_or(0.3);
                let h = json_number_field(&obj, "h").unwrap_or(0.3);
                obstacles.push(car_sim_core::physics::Obstacle::Box { x, y, w, h });
            } else {
                let r = json_number_field(&obj, "r").unwrap_or(0.15);
                obstacles.push(car_sim_core::physics::Obstacle::Circle { x, y, r });
            }
        }
    }

    if centerline.len() < 2 {
        return Err("赛道中心线至少需要 2 个点".to_string());
    }

    Ok(Track {
        name,
        width_m: width,
        line_width_m: line_width,
        centerline,
        obstacles,
        start: car_sim_core::physics::StartIdx { index: 0 },
        target_lap_time_s: target,
    })
}

/* ---------------- 极简 JSON 读取（避免引入依赖） ---------------- */

fn json_string_field(s: &str, key: &str) -> Option<String> {
    let needle = format!("\"{key}\"");
    let i = s.find(&needle)?;
    let rest = &s[i + needle.len()..];
    let rest = rest.trim_start();
    if !rest.starts_with(':') {
        return None;
    }
    let rest = rest[1..].trim_start();
    if !rest.starts_with('"') {
        return None;
    }
    let rest = &rest[1..];
    let end = rest.find('"')?;
    Some(rest[..end].to_string())
}

fn json_number_field(s: &str, key: &str) -> Option<f64> {
    let needle = format!("\"{key}\"");
    let i = s.find(&needle)?;
    let rest = &s[i + needle.len()..];
    let rest = rest.trim_start();
    if !rest.starts_with(':') {
        return None;
    }
    let rest = rest[1..].trim_start();
    let take: String = rest
        .chars()
        .take_while(|c| c.is_ascii_digit() || *c == '-' || *c == '.' || *c == 'e' || *c == 'E' || *c == '+')
        .collect();
    take.parse().ok()
}

/// 取 key 后面第一个 `[...]`（按括号配对）
fn json_array_after(s: &str, key: &str) -> Option<String> {
    let needle = format!("\"{key}\"");
    let i = s.find(&needle)?;
    let rest = &s[i + needle.len()..];
    let start = rest.find('[')?;
    let bytes = rest.as_bytes();
    let mut depth = 0usize;
    for (j, b) in bytes.iter().enumerate().skip(start) {
        match b {
            b'[' => depth += 1,
            b']' => {
                depth -= 1;
                if depth == 0 {
                    return Some(rest[start + 1..j].to_string());
                }
            }
            _ => {}
        }
    }
    None
}

/// 把数组内容切成若干个 {...} 对象片段
fn json_objects_in(arr: &str) -> Vec<String> {
    let mut out = Vec::new();
    let bytes = arr.as_bytes();
    let mut depth = 0usize;
    let mut start: Option<usize> = None;
    for (i, b) in bytes.iter().enumerate() {
        match b {
            b'{' => {
                if depth == 0 {
                    start = Some(i);
                }
                depth += 1;
            }
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    if let Some(s0) = start {
                        out.push(arr[s0..=i].to_string());
                    }
                    start = None;
                }
            }
            _ => {}
        }
    }
    out
}

/* ------------------------------------------------------------------ *
 * 内置赛道（与 simcore/tracks/*.json 保持一致）
 * ------------------------------------------------------------------ */

fn bundled_tracks() -> Vec<(String, Track)> {
    vec![
        ("track01".into(), track01()),
        ("track02".into(), track02()),
        ("track03".into(), track03()),
    ]
}

fn track01() -> Track {
    let mut pts = Vec::new();
    let n = 120usize;
    let (rx, ry) = (3.0f64, 4.5f64);
    for i in 0..n {
        let t = i as f64 / n as f64 * std::f64::consts::TAU;
        pts.push(car_sim_core::physics::TrackPoint {
            x: rx * t.sin(),
            y: ry * (1.0 - t.cos()),
        });
    }
    Track {
        name: "track01".into(),
        width_m: 0.55,
        line_width_m: 0.03,
        centerline: pts,
        obstacles: vec![],
        start: car_sim_core::physics::StartIdx { index: 0 },
        target_lap_time_s: 25.0,
    }
}

fn track02() -> Track {
    // S 形：y 前进，x 做正弦摆动
    let mut pts = Vec::new();
    let n = 160usize;
    for i in 0..n {
        let u = i as f64 / n as f64;
        let y = u * 20.0;
        let x = 2.2 * (u * std::f64::consts::TAU * 1.5).sin();
        pts.push(car_sim_core::physics::TrackPoint { x, y });
    }
    Track {
        name: "track02".into(),
        width_m: 0.45,
        line_width_m: 0.03,
        centerline: pts,
        obstacles: vec![],
        start: car_sim_core::physics::StartIdx { index: 0 },
        target_lap_time_s: 32.0,
    }
}

fn track03() -> Track {
    // 直角弯 + 障碍物
    let mut pts: Vec<car_sim_core::physics::TrackPoint> = Vec::new();
    // 直道
    for i in 0..40 {
        pts.push(car_sim_core::physics::TrackPoint { x: 0.0, y: i as f64 * 0.15 });
    }
    // 90° 圆角弯（半径 1.0，圆心 (1,6)）
    for i in 0..30 {
        let a = -std::f64::consts::FRAC_PI_2 + (i as f64 / 30.0) * std::f64::consts::FRAC_PI_2;
        pts.push(car_sim_core::physics::TrackPoint {
            x: 1.0 + 1.0 * a.cos(),
            y: 6.0 + 1.0 * a.sin(),
        });
    }
    // 再直道
    for i in 0..40 {
        pts.push(car_sim_core::physics::TrackPoint { x: 2.0 + i as f64 * 0.15, y: 7.0 });
    }
    // 掉头弯回来
    for i in 0..30 {
        let a = 0.0 + (i as f64 / 30.0) * std::f64::consts::PI;
        pts.push(car_sim_core::physics::TrackPoint {
            x: 8.0 + 1.2 * a.cos(),
            y: 7.0 + 1.2 * a.sin(),
        });
    }
    for i in 0..50 {
        pts.push(car_sim_core::physics::TrackPoint { x: 6.8 - i as f64 * 0.15, y: 7.0 });
    }

    let obstacles = vec![
        car_sim_core::physics::Obstacle::Circle { x: 0.55, y: 3.0, r: 0.14 },
        car_sim_core::physics::Obstacle::Box { x: 4.0, y: 7.55, w: 0.4, h: 0.3 },
        car_sim_core::physics::Obstacle::Circle { x: 3.0, y: 6.6, r: 0.12 },
    ];

    Track {
        name: "track03".into(),
        width_m: 0.40,
        line_width_m: 0.03,
        centerline: pts,
        obstacles,
        start: car_sim_core::physics::StartIdx { index: 0 },
        target_lap_time_s: 38.0,
    }
}

/* ------------------------------------------------------------------ *
 * JSON 输出（手写，避免 CLI 依赖 serde）
 * ------------------------------------------------------------------ */

fn report_json(r: &sim::SimReport) -> String {
    let issues: Vec<String> = r
        .r#static
        .issues
        .iter()
        .map(|i| {
            format!(
                "{{\"severity\":\"{}\",\"line\":{},\"rule\":\"{}\",\"message\":\"{}\"}}",
                serde_json_like::escape(&format!("{:?}", i.severity).to_lowercase()),
                i.line,
                i.rule,
                serde_json_like::escape(&i.message)
            )
        })
        .collect();

    let tele: Vec<String> = r
        .sim
        .telemetry
        .iter()
        .map(|p| {
            format!(
                "{{\"t\":{},\"x\":{},\"y\":{},\"yaw\":{},\"v\":{},\"wheel_l\":{},\"wheel_r\":{},\"pwm_l\":{},\"pwm_r\":{},\"sensors\":[{}],\"ultrasonic_cm\":{}}}",
                p.t, p.x, p.y, p.yaw, p.v, p.wheel_l, p.wheel_r, p.pwm_l, p.pwm_r,
                p.sensors.iter().map(|s| s.to_string()).collect::<Vec<_>>().join(","),
                p.ultrasonic_cm
            )
        })
        .collect();

    let logs: Vec<String> = r
        .logs
        .iter()
        .map(|l| format!("\"{}\"", serde_json_like::escape(l)))
        .collect();

    format!(
        concat!(
            "{{\n",
            "  \"run_id\": \"{}\",\n",
            "  \"submitted_at\": \"{}\",\n",
            "  \"code\": {{\"path\":\"{}\",\"bytes\":{},\"sha256\":\"{}\"}},\n",
            "  \"static\": {{\n",
            "    \"issues\": [{}],\n",
            "    \"detected\": {{\"mcu\":\"{}\",\"motor_bindings\":[{}],\"sensor_bindings\":[{}],\"has_main_loop\":{},\"uses_delay\":{}}}\n",
            "  }},\n",
            "  \"sim\": {{\n",
            "    \"track\":\"{}\",\"dt\":{},\"steps\":{},\"time_limit_s\":{},\n",
            "    \"result\":\"{}\",\"failure_reason\":\"{}\",\n",
            "    \"lap_time_s\":{},\"distance_m\":{},\"progress\":{},\n",
            "    \"max_speed_mps\":{},\"avg_speed_mps\":{},\n",
            "    \"off_track_events\":{},\"collisions\":{},\n",
            "    \"telemetry\": [{}]\n",
            "  }},\n",
            "  \"score\": {{\"total\":{},\"breakdown\":{{\"completion\":{},\"speed\":{},\"stability\":{},\"style\":{}}},\"grade\":\"{}\",\"comment\":\"{}\"}},\n",
            "  \"logs\": [{}]\n",
            "}}"
        ),
        r.run_id,
        r.submitted_at,
        serde_json_like::escape(&r.code.path),
        r.code.bytes,
        r.code.sha256,
        issues.join(","),
        r.r#static.detected.mcu,
        r.r#static
            .detected
            .motor_bindings
            .iter()
            .map(|s| format!("\"{s}\""))
            .collect::<Vec<_>>()
            .join(","),
        r.r#static
            .detected
            .sensor_bindings
            .iter()
            .map(|s| format!("\"{s}\""))
            .collect::<Vec<_>>()
            .join(","),
        r.r#static.detected.has_main_loop,
        r.r#static.detected.uses_delay,
        r.sim.track,
        r.sim.dt,
        r.sim.steps,
        r.sim.time_limit_s,
        format!("{:?}", r.sim.result).to_lowercase(),
        serde_json_like::escape(&r.sim.failure_reason),
        r.sim.lap_time_s,
        r.sim.distance_m,
        r.sim.progress,
        r.sim.max_speed_mps,
        r.sim.avg_speed_mps,
        r.sim.off_track_events,
        r.sim.collisions,
        tele.join(","),
        r.score.total,
        r.score.breakdown.completion,
        r.score.breakdown.speed,
        r.score.breakdown.stability,
        r.score.breakdown.style,
        r.score.grade,
        serde_json_like::escape(&r.score.comment),
        logs.join(","),
    )
}

fn report_of(r: &lint::LintReport) -> &lint::LintReport {
    r
}

/* ------------------------------------------------------------------ *
 * 帮助
 * ------------------------------------------------------------------ */

fn print_help() {
    println!("car-sim v{VERSION} —— STM32 智能车仿真器");
    println!();
    println!("用法:");
    println!("  car-sim check <file.c>              只做静态检查");
    println!("  car-sim run <file.c> [选项]          跑仿真并输出报告");
    println!("  car-sim bench <file.c>               在所有内置赛道上跑一遍并对比");
    println!("  car-sim list-tracks                  列出内置赛道");
    println!("  car-sim demo                         用内置示例代码演示一次");
    println!();
    println!("选项:");
    println!("  --track <name>        赛道名或 json 路径（默认 track01）");
    println!("  --dt <秒>             物理步长（默认 0.01）");
    println!("  --time-limit <秒>     仿真时间上限（默认 60）");
    println!("  --seed <数>           随机种子（默认 20251025，保证可复现）");
    println!("  --sample-every <n>    每 n 步采样一次遥测（默认 10）");
    println!("  --out <file.json>     把完整报告写成 JSON");
    println!("  --telemetry-csv <f>   把遥测写成 CSV");
    println!("  --force               有 error 级静态问题也继续仿真");
    println!("  --json                机器可读输出");
    println!("  --quiet               只输出结果文件，不打印报告");
    println!("  --version             显示版本");
    println!();
    println!("退出码:");
    println!("  0 跑完   1 静态检查有错误   2 程序崩溃   3 超时   4 中途失败");
}

/* ------------------------------------------------------------------ *
 * 迷你 JSON 值（仅 check --json 用）
 * ------------------------------------------------------------------ */

mod serde_json_like {
    pub enum Value {
        Str(String),
        Num(f64),
        Bool(bool),
        Arr(Vec<Value>),
        Obj(Map),
    }

    pub type Map = Vec<(String, Value)>;

    pub fn issue(i: &car_sim_core::lint::Issue) -> Value {
        let mut m = Map::new();
        m.insert(
            "severity".into(),
            Value::Str(format!("{:?}", i.severity).to_lowercase()),
        );
        m.insert("line".into(), Value::Num(i.line as f64));
        m.insert("rule".into(), Value::Str(i.rule.clone()));
        m.insert("message".into(), Value::Str(i.message.clone()));
        Value::Obj(m)
    }

    pub fn stringify(v: &Value) -> String {
        match v {
            Value::Str(s) => format!("\"{}\"", escape(s)),
            Value::Num(n) => {
                if n.fract() == 0.0 {
                    format!("{}", *n as i64)
                } else {
                    format!("{n}")
                }
            }
            Value::Bool(b) => b.to_string(),
            Value::Arr(a) => format!(
                "[{}]",
                a.iter().map(stringify).collect::<Vec<_>>().join(",")
            ),
            Value::Obj(m) => format!(
                "{{{}}}",
                m.iter()
                    .map(|(k, v)| format!("\"{}\":{}", escape(k), stringify(v)))
                    .collect::<Vec<_>>()
                    .join(",")
            ),
        }
    }

    pub fn escape(s: &str) -> String {
        s.replace('\\', "\\\\")
            .replace('"', "\\\"")
            .replace('\n', "\\n")
            .replace('\r', "\\r")
            .replace('\t', "\\t")
    }
}

#[cfg(test)]
mod tests {
    // CLI 的行为主要在 simcore 里测过了，这里只测自己手写的 JSON 小工具
    #[test]
    fn json_helpers_parse_track() {
        let s = r#"{"name":"t","width_m":0.5,"centerline":[{"x":0,"y":0},{"x":1,"y":1}],"obstacles":[{"type":"circle","x":2,"y":3,"r":0.2}],"target_lap_time_s":30}"#;
        assert_eq!(super::json_string_field(s, "name"), Some("t".into()));
        assert_eq!(super::json_number_field(s, "width_m"), Some(0.5));
        let arr = super::json_array_after(s, "centerline").unwrap();
        assert_eq!(super::json_objects_in(&arr).len(), 2);
    }

    #[test]
    fn number_field_handles_negative() {
        let s = r#"{"k": -12.5 }"#;
        assert_eq!(super::json_number_field(s, "k"), Some(-12.5));
    }

    #[test]
    fn escape_handles_quotes() {
        assert_eq!(super::serde_json_like::escape("a\"b\n"), "a\\\"b\\n");
    }
}

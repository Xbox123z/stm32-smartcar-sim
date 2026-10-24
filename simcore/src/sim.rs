//! 仿真驱动器：把 C 解释器与物理世界缝在一起，产出评分与遥测

use serde::Serialize;

use crate::csim::parse_source;
use crate::interp::{Bindings, Interp};
use crate::lint::{self, LintReport};
use crate::physics::{CarConfig, Track, World};

/* ------------------------------------------------------------------ *
 * 配置与报告
 * ------------------------------------------------------------------ */

#[derive(Debug, Clone)]
pub struct RunOptions {
    pub track: Track,
    pub dt: f64,
    pub time_limit_s: f64,
    pub seed: u64,
    /// 每多少 tick 采样一次遥测
    pub sample_every: usize,
    /// 连续多少秒进度没有推进就判定卡住
    pub stuck_after_s: f64,
    pub car: CarConfig,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self {
            track: oval_track(),
            dt: 0.01,
            time_limit_s: 60.0,
            seed: 20251025,
            sample_every: 10,
            stuck_after_s: 8.0,
            car: CarConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum RunResult {
    Finished,
    Timeout,
    Failed,
    Crashed,
}

impl RunResult {
    pub fn label(self) -> &'static str {
        match self {
            RunResult::Finished => "完成",
            RunResult::Timeout => "超时",
            RunResult::Failed => "失败",
            RunResult::Crashed => "崩溃",
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct TelemetryPoint {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub wheel_l: f64,
    pub wheel_r: f64,
    pub pwm_l: i32,
    pub pwm_r: i32,
    pub sensors: Vec<i32>,
    pub ultrasonic_cm: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct SimSection {
    pub track: String,
    pub dt: f64,
    pub steps: usize,
    pub time_limit_s: f64,
    pub result: RunResult,
    pub failure_reason: String,
    pub lap_time_s: f64,
    pub distance_m: f64,
    pub progress: f64,
    pub max_speed_mps: f64,
    pub avg_speed_mps: f64,
    pub off_track_events: usize,
    pub collisions: usize,
    /// 赛道目标单圈时间（评分用）
    pub track_target_s: f64,
    pub telemetry: Vec<TelemetryPoint>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ScoreBreakdown {
    pub completion: u8,
    pub speed: u8,
    pub stability: u8,
    pub style: u8,
}

#[derive(Debug, Clone, Serialize)]
pub struct Score {
    pub total: u8,
    pub breakdown: ScoreBreakdown,
    pub grade: String,
    pub comment: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct CodeInfo {
    pub path: String,
    pub bytes: usize,
    pub sha256: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct SimReport {
    pub run_id: String,
    pub submitted_at: String,
    pub code: CodeInfo,
    pub r#static: StaticSection,
    pub sim: SimSection,
    pub score: Score,
    pub logs: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct StaticSection {
    pub issues: Vec<lint::Issue>,
    pub detected: lint::Detected,
}

/* ------------------------------------------------------------------ *
 * 主入口
 * ------------------------------------------------------------------ */

/// 读源码 + 选项，跑一次完整仿真
pub fn run_file(src: &str, path: &str, opts: RunOptions) -> SimReport {
    let lint_report = lint::lint(src);
    run_with_lint(src, path, opts, lint_report)
}

/// 允许调用方复用已算好的 lint（CLI 的 check+run 会连着做）
pub fn run_with_lint(
    src: &str,
    path: &str,
    opts: RunOptions,
    lint_report: LintReport,
) -> SimReport {
    let (prog, _parse_errs) = parse_source(src);

    let mut world = World::new(opts.track.clone(), opts.car.clone(), opts.dt, opts.seed);
    let mut interp = Interp::new(&prog, &mut world, Bindings::default());

    let max_steps = (opts.time_limit_s / opts.dt) as usize;
    let mut telemetry: Vec<TelemetryPoint> = Vec::new();
    let mut steps = 0usize;
    let mut result = RunResult::Timeout;
    let mut failure_reason = String::new();
    let mut crashed_msg: Option<String> = None;

    let mut last_progress = 0.0f64;
    let mut last_progress_time = 0.0f64;
    let mut stuck = false;

    // 启动：跑到第一个 yield（或结束）
    let mut pending: Option<f64> = match interp.start() {
        Ok(v) => v,
        Err(e) => {
            crashed_msg = Some(e);
            None
        }
    };

    while crashed_msg.is_none() && steps < max_steps {
        // ---- 推进物理 ----
        let advance_ms = pending.unwrap_or(opts.dt * 1000.0);
        let n_steps = ((advance_ms / 1000.0) / opts.dt).round().max(0.0) as usize;
        let n_steps = n_steps.min(max_steps - steps);
        for _ in 0..n_steps {
            world.step();
            steps += 1;
            if steps % opts.sample_every == 0 {
                telemetry.push(sample(&mut world, interp.pwm_l, interp.pwm_r));
            }
        }

        if steps >= max_steps {
            result = RunResult::Timeout;
            failure_reason = format!("在 {:.1}s 内没有跑完（时间上限）", opts.time_limit_s);
            break;
        }

        // ---- 结束条件 ----
        if world.progress() >= 0.995 {
            result = RunResult::Finished;
            break;
        }

        // 卡住检测
        if world.progress() > last_progress + 1e-6 {
            last_progress = world.progress();
            last_progress_time = world.time;
        } else if world.time - last_progress_time > opts.stuck_after_s {
            stuck = true;
        }
        if stuck {
            result = RunResult::Failed;
            failure_reason = format!(
                "进度停在 {:.1}% 超过 {:.0}s，小车卡住了（可能撞墙、丢线或电机没转）",
                world.progress() * 100.0,
                opts.stuck_after_s
            );
            break;
        }

        if world.off_track_events > 40 {
            result = RunResult::Failed;
            failure_reason = format!(
                "脱离赛道 {} 次，控制策略基本失效",
                world.off_track_events
            );
            break;
        }

        // ---- 继续执行代码，直到下一个 yield ----
        pending = match interp.resume() {
            Ok(v) => v,
            Err(e) => {
                crashed_msg = Some(e);
                None
            }
        };

        // 代码跑完了（main 返回）但车还没到终点
        if pending.is_none() {
            if interp.finished && world.progress() < 0.995 {
                result = RunResult::Failed;
                failure_reason = format!(
                    "程序结束了，但小车只跑到 {:.1}%（主循环可能提前退出）",
                    world.progress() * 100.0
                );
            }
            break;
        }
    }

    if let Some(msg) = crashed_msg {
        result = RunResult::Crashed;
        failure_reason = msg;
    }

    let lap_time_s = if result == RunResult::Finished {
        world.time
    } else {
        0.0
    };
    let avg_speed = if world.time > 0.0 {
        world.distance / world.time
    } else {
        0.0
    };

    let sim = SimSection {
        track: opts.track.name.clone(),
        dt: opts.dt,
        steps,
        time_limit_s: opts.time_limit_s,
        result,
        failure_reason,
        lap_time_s: r2(lap_time_s),
        distance_m: r3(world.distance),
        progress: r4(world.progress()),
        max_speed_mps: r3(world.max_speed),
        avg_speed_mps: r3(avg_speed),
        off_track_events: world.off_track_events,
        collisions: world.collisions,
        track_target_s: opts.track.target_lap_time_s,
        telemetry,
    };

    let score = score(&sim, &lint_report);

    SimReport {
        run_id: run_id(opts.seed),
        submitted_at: rfc3339_now(),
        code: CodeInfo {
            path: path.to_string(),
            bytes: src.len(),
            sha256: sha256(src),
        },
        r#static: StaticSection {
            issues: lint_report.issues.clone(),
            detected: lint_report.detected.clone(),
        },
        sim,
        score,
        logs: interp.logs.clone(),
    }
}

fn sample(w: &mut World, pwm_l: i32, pwm_r: i32) -> TelemetryPoint {
    TelemetryPoint {
        t: r3(w.time),
        x: r4(w.car.x),
        y: r4(w.car.y),
        yaw: r4(w.car.yaw),
        v: r4(w.car.v),
        wheel_l: r4(w.car.wheel_l),
        wheel_r: r4(w.car.wheel_r),
        pwm_l,
        pwm_r,
        sensors: w.read_line_sensors(),
        ultrasonic_cm: r2(w.read_ultrasonic_cm()),
    }
}

/* ------------------------------------------------------------------ *
 * 评分
 * ------------------------------------------------------------------ */

pub fn score(sim: &SimSection, lint: &LintReport) -> Score {
    // 完成度 40
    let completion = (sim.progress * 40.0).round() as u8;

    // 速度 20：与赛道目标时间比
    let target = sim.track_target_s.max(1.0);
    let speed = if sim.result == RunResult::Finished && sim.lap_time_s > 0.0 {
        let ratio = target / sim.lap_time_s; // 越快越高
        (ratio * 20.0).clamp(0.0, 20.0).round() as u8
    } else {
        // 没跑完就按推进速度给部分分
        let eff = if sim.time_limit_s > 0.0 {
            sim.distance_m / sim.time_limit_s / 0.4
        } else {
            0.0
        };
        (eff * 12.0).clamp(0.0, 12.0).round() as u8
    };

    // 稳定性 20
    let penalty = sim.off_track_events as f64 * 1.5 + sim.collisions as f64 * 3.0;
    let stability = (20.0 - penalty).clamp(0.0, 20.0).round() as u8;

    // 代码风格 20：按 lint 扣分
    let (e, w, _i) = lint::count(&lint.issues);
    let style = (20.0 - e as f64 * 6.0 - w as f64 * 2.0).clamp(0.0, 20.0).round() as u8;

    let total = (completion as u32 + speed as u32 + stability as u32 + style as u32).min(100) as u8;

    let grade = match total {
        95..=100 => "S",
        85..=94 => "A",
        70..=84 => "B",
        50..=69 => "C",
        _ => "D",
    }
    .to_string();

    let comment = build_comment(total, &grade, sim, (e, w));

    Score {
        total,
        breakdown: ScoreBreakdown {
            completion,
            speed,
            stability,
            style,
        },
        grade,
        comment,
    }
}

fn build_comment(total: u8, grade: &str, sim: &SimSection, (e, w): (usize, usize)) -> String {
    let mut parts: Vec<String> = Vec::new();

    parts.push(match sim.result {
        RunResult::Finished => format!("跑完了全程，用时 {:.1}s。", sim.lap_time_s),
        RunResult::Timeout => format!("时间到，只完成了 {:.0}%。", sim.progress * 100.0),
        RunResult::Failed => format!("中途失败：{}。", sim.failure_reason),
        RunResult::Crashed => format!("程序崩溃：{}。", sim.failure_reason),
    });

    if sim.off_track_events > 0 {
        parts.push(format!("脱离赛道 {} 次，建议把 Kp 调小或加一点 Kd 抑制震荡。", sim.off_track_events));
    }
    if sim.collisions > 0 {
        parts.push(format!("发生 {} 次碰撞，注意避障逻辑。", sim.collisions));
    }
    if sim.off_track_events == 0 && sim.collisions == 0 && sim.result == RunResult::Finished {
        parts.push("全程没有出界也没有碰撞，非常稳。".to_string());
    }
    if e > 0 {
        parts.push(format!("还有 {} 个必须修的错误。", e));
    } else if w > 0 {
        parts.push(format!("有 {} 处可以改进的地方。", w));
    } else {
        parts.push("静态检查全部通过。".to_string());
    }

    let head = match grade {
        "S" => "完美，可以直接上赛场了！",
        "A" => "很不错，再微调一下就更好了。",
        "B" => "基本能跑，细节还有提升空间。",
        "C" => "能跑但不够稳，建议重点检查控制参数。",
        _ => "目前还跑不通，建议先对着错误列表逐条修。",
    };

    format!("{}（{} 分 / {} 级）{}", head, total, grade, parts.join(" "))
}

/* ------------------------------------------------------------------ *
 * 内置赛道
 * ------------------------------------------------------------------ */

/// 简易椭圆赛道（无障碍物，适合入门）
pub fn oval_track() -> Track {
    let mut pts = Vec::new();
    let n = 120;
    let (rx, ry) = (3.0, 4.5);
    for i in 0..n {
        let t = i as f64 / n as f64 * std::f64::consts::TAU;
        pts.push(crate::physics::TrackPoint {
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
        start: crate::physics::StartIdx { index: 0 },
        target_lap_time_s: 25.0,
    }
}

/* ------------------------------------------------------------------ *
 * 杂项
 * ------------------------------------------------------------------ */

fn r2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}
fn r3(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}
fn r4(v: f64) -> f64 {
    (v * 10000.0).round() / 10000.0
}

/// 简单的确定性 run_id（不引入 uuid 依赖）
fn run_id(seed: u64) -> String {
    let mut x = seed ^ 0x9E3779B97F4A7C15;
    let mut s = String::new();
    for i in 0..32 {
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        let v = x.wrapping_mul(0x2545F4914F6CDD1D);
        s.push_str(&format!("{:x}", (v >> (i % 8)) & 0xF));
        if i == 7 || i == 11 || i == 15 || i == 19 {
            s.push('-');
        }
    }
    s
}

/// 稳定的时间戳（不引入 time crate：用系统时钟，失败则回退）
fn rfc3339_now() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    // 固定偏移写成 Z，够用（前端只用于展示）
    format_unix(secs)
}

fn format_unix(secs: u64) -> String {
    // 把 unix 秒转成 YYYY-MM-DDTHH:MM:SSZ（简化实现，忽略闰秒）
    let days = secs / 86400;
    let rem = secs % 86400;
    let (y, m, d) = civil_from_days(days as i64);
    let h = rem / 3600;
    let mi = (rem % 3600) / 60;
    let s = rem % 60;
    format!("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z", y, m, d, h, mi, s)
}

/// Howard Hinnant 的 civil_from_days 算法
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719468;
    let era = if z >= 0 { z } else { z - 146096 } / 146097;
    let doe = (z - era * 146097) as i64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d)
}

/// 精简 SHA-256（只为生成稳定指纹，不用于安全场景）
pub fn sha256(data: &str) -> String {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];

    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];

    let mut msg = data.as_bytes().to_vec();
    let bit_len = (msg.len() as u64) * 8;
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bit_len.to_be_bytes());

    for chunk in msg.chunks(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                chunk[i * 4],
                chunk[i * 4 + 1],
                chunk[i * 4 + 2],
                chunk[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let mut a = h[0];
        let mut b = h[1];
        let mut c = h[2];
        let mut d = h[3];
        let mut e = h[4];
        let mut f = h[5];
        let mut g = h[6];
        let mut hh = h[7];

        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }

        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
        h[5] = h[5].wrapping_add(f);
        h[6] = h[6].wrapping_add(g);
        h[7] = h[7].wrapping_add(hh);
    }

    h.iter().map(|x| format!("{:08x}", x)).collect()
}

/// 遥测导出为 CSV
pub fn telemetry_csv(report: &SimReport) -> String {
    let mut s = String::from("t,x,y,yaw,v,wheel_l,wheel_r,pwm_l,pwm_r,s0,s1,s2,s3,s4,ultrasonic_cm\n");
    for p in &report.sim.telemetry {
        let sen: Vec<String> = (0..5)
            .map(|i| p.sensors.get(i).copied().unwrap_or(0).to_string())
            .collect();
        s.push_str(&format!(
            "{},{},{},{},{},{},{},{},{},{},{}\n",
            p.t,
            p.x,
            p.y,
            p.yaw,
            p.v,
            p.wheel_l,
            p.wheel_r,
            p.pwm_l,
            p.pwm_r,
            sen.join(","),
            p.ultrasonic_cm
        ));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn good_code() -> &'static str {
        r#"
float Kp = 300.0, Kd = 60.0;
float last = 0;
int main(void){
  while(1){
    float pos = Read_Line();
    float out = Kp * pos + Kd * (pos - last);
    last = pos;
    Motor_Set(450 + (int)out, 450 - (int)out);
    HAL_Delay(10);
  }
}
"#
    }

    #[test]
    fn good_code_finishes() {
        let r = run_file(good_code(), "main.c", RunOptions::default());
        assert_eq!(r.sim.result, RunResult::Finished, "{:?}", r.sim.failure_reason);
        assert!(r.sim.progress > 0.99);
        assert!(!r.sim.telemetry.is_empty());
    }

    #[test]
    fn report_has_expected_shape() {
        let r = run_file(good_code(), "main.c", RunOptions::default());
        assert!(!r.run_id.is_empty());
        assert!(r.submitted_at.contains('T'));
        assert_eq!(r.code.path, "main.c");
        assert_eq!(r.code.sha256.len(), 64);
        assert!(r.score.total <= 100);
        assert!(matches!(r.score.grade.as_str(), "S" | "A" | "B" | "C" | "D"));
    }

    #[test]
    fn no_motor_means_failure() {
        let src = "int main(void){ while(1){ int x = Sensor_Read(2); HAL_Delay(10); } }";
        let r = run_file(src, "main.c", RunOptions::default());
        assert_ne!(r.sim.result, RunResult::Finished);
        assert_eq!(r.sim.progress, 0.0);
    }

    #[test]
    fn busy_loop_crashes() {
        let src = "int main(void){ while(1){ Motor_Set(500,500); } }";
        let r = run_file(src, "main.c", RunOptions::default());
        assert_eq!(r.sim.result, RunResult::Crashed);
        assert!(r.sim.failure_reason.contains("预算"));
    }

    #[test]
    fn timeout_when_too_slow() {
        let opts = RunOptions { time_limit_s: 1.0, ..Default::default() };
        let r = run_file(good_code(), "main.c", opts);
        assert_eq!(r.sim.result, RunResult::Timeout);
    }

    #[test]
    fn telemetry_is_sampled() {
        let opts = RunOptions { sample_every: 10, ..Default::default() };
        let r = run_file(good_code(), "main.c", opts);
        assert!(r.sim.telemetry.len() > 5);
        let p = &r.sim.telemetry[0];
        assert_eq!(p.sensors.len(), 5);
    }

    #[test]
    fn csv_has_header_and_rows() {
        let r = run_file(good_code(), "main.c", RunOptions::default());
        let csv = telemetry_csv(&r);
        assert!(csv.starts_with("t,x,y,yaw"));
        assert_eq!(csv.lines().count(), r.sim.telemetry.len() + 1);
    }

    #[test]
    fn score_bounds_and_grade() {
        let r = run_file(good_code(), "main.c", RunOptions::default());
        assert!(r.score.total <= 100);
        assert!(r.score.breakdown.completion <= 40);
        assert!(r.score.breakdown.speed <= 20);
        assert!(r.score.breakdown.stability <= 20);
        assert!(r.score.breakdown.style <= 20);
    }

    #[test]
    fn lint_errors_reduce_style_score() {
        let bad = "int main(void){ while(1){ Motor_Set(500,500); } }";
        let r = run_file(bad, "main.c", RunOptions::default());
        assert_eq!(r.sim.result, RunResult::Crashed);
        assert!(r.score.breakdown.style < 20);
    }

    #[test]
    fn deterministic_run() {
        let a = run_file(good_code(), "main.c", RunOptions::default());
        let b = run_file(good_code(), "main.c", RunOptions::default());
        assert_eq!(a.sim.steps, b.sim.steps);
        assert_eq!(a.sim.distance_m, b.sim.distance_m);
        assert_eq!(a.sim.telemetry.len(), b.sim.telemetry.len());
    }

    #[test]
    fn sha256_known_vector() {
        assert_eq!(
            sha256(""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            sha256("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn civil_from_days_known() {
        // 1970-01-01
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        // 2000-01-01 = day 10957
        assert_eq!(civil_from_days(10957), (2000, 1, 1));
        // 2025-10-25
        assert_eq!(civil_from_days(20397), (2025, 10, 25));
    }

    #[test]
    fn oval_track_shape() {
        let t = oval_track();
        assert!(t.length() > 10.0);
        assert_eq!(t.centerline.len(), 120);
        assert_eq!(t.name, "track01");
    }
}

//! 终端输出的漂亮排版

use car_sim_core::lint;
use car_sim_core::sim::SimReport;

/// 把整份报告渲染成可读文本
pub fn render_report(r: &SimReport) -> String {
    use std::fmt::Write;
    let mut s = String::new();

    let _ = writeln!(s, "{}", "═".repeat(62));
    let _ = writeln!(s, "  🏎️  STM32 智能车仿真报告");
    let _ = writeln!(s, "{}", "═".repeat(62));
    let _ = writeln!(s);
    let _ = writeln!(s, "  📄 {}   ({} 字节)", r.code.path, r.code.bytes);
    let _ = writeln!(s, "  🆔 {}", r.run_id);
    let _ = writeln!(s, "  🕐 {}", r.submitted_at);
    let _ = writeln!(s);

    // ---- 静态检查 ----
    let (e, w, i) = lint::count(&r.r#static.issues);
    let _ = writeln!(s, "  🔍 静态检查    {} 错误 · {} 警告 · {} 提示", e, w, i);
    let _ = writeln!(s, "     芯片        {}", r.r#static.detected.mcu);
    let _ = writeln!(s, "     电机        {}", r.r#static.detected.motor_bindings.join(", "));
    if !r.r#static.issues.is_empty() {
        let _ = writeln!(s);
        for issue in r.r#static.issues.iter().take(12) {
            let icon = match issue.severity {
                lint::Severity::Error => "❌",
                lint::Severity::Warn => "⚠️ ",
                lint::Severity::Info => "💡",
            };
            let _ = writeln!(s, "     {} [{}] 第 {} 行", icon, issue.rule, issue.line);
            let _ = writeln!(s, "        {}", issue.message);
        }
        if r.r#static.issues.len() > 12 {
            let _ = writeln!(s, "     …还有 {} 条", r.r#static.issues.len() - 12);
        }
    }
    let _ = writeln!(s);

    // ---- 仿真 ----
    let sim = &r.sim;
    let icon = match sim.result {
        car_sim_core::sim::RunResult::Finished => "✅",
        car_sim_core::sim::RunResult::Timeout => "⏰",
        car_sim_core::sim::RunResult::Failed => "❌",
        car_sim_core::sim::RunResult::Crashed => "💥",
    };
    let _ = writeln!(s, "  🏁 仿真结果");
    let _ = writeln!(s, "     赛道        {}", sim.track);
    let _ = writeln!(s, "     结果        {} {}", icon, sim.result.label());
    if !sim.failure_reason.is_empty() {
        let _ = writeln!(s, "     原因        {}", sim.failure_reason);
    }
    let _ = writeln!(
        s,
        "     进度        {:>5.1}%   （距离 {:.2} m）",
        sim.progress * 100.0,
        sim.distance_m
    );
    if sim.result == car_sim_core::sim::RunResult::Finished {
        let _ = writeln!(s, "     用时        {:.2} s", sim.lap_time_s);
    }
    let _ = writeln!(
        s,
        "     速度        平均 {:.2} m/s   最高 {:.2} m/s",
        sim.avg_speed_mps, sim.max_speed_mps
    );
    let _ = writeln!(
        s,
        "     出界/碰撞   {} 次 / {} 次",
        sim.off_track_events, sim.collisions
    );
    let _ = writeln!(s, "     步数        {}（dt={}，{} 个遥测点）", sim.steps, sim.dt, sim.telemetry.len());
    let _ = writeln!(s);

    // ---- 评分 ----
    let b = &r.score.breakdown;
    let _ = writeln!(s, "  🎯 评分        {} / 100   （{} 级）", r.score.total, r.score.grade);
    let _ = writeln!(s, "     完成度      {:>3} / 40  {}", b.completion, bar(b.completion, 40));
    let _ = writeln!(s, "     速度        {:>3} / 20  {}", b.speed, bar(b.speed, 20));
    let _ = writeln!(s, "     稳定性      {:>3} / 20  {}", b.stability, bar(b.stability, 20));
    let _ = writeln!(s, "     代码风格    {:>3} / 20  {}", b.style, bar(b.style, 20));
    let _ = writeln!(s);
    let _ = writeln!(s, "  💬 {}", r.score.comment);

    // ---- 日志 ----
    if !r.logs.is_empty() {
        let _ = writeln!(s);
        let _ = writeln!(s, "  📜 程序输出（前 {} 条）", r.logs.len().min(10));
        for l in r.logs.iter().take(10) {
            let _ = writeln!(s, "     {}", l);
        }
    }

    let _ = writeln!(s);
    let _ = writeln!(s, "{}", "═".repeat(62));
    s
}

fn bar(v: u8, max: u8) -> String {
    let filled = ((v as f32 / max as f32) * 20.0).round() as usize;
    format!("{}{}", "█".repeat(filled), "░".repeat(20 - filled.min(20)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use car_sim_core::sim::RunOptions;

    fn report() -> SimReport {
        car_sim_core::sim::run_file(
            "int main(void){ while(1){ int p = Sensor_Read(2); Motor_Set(400,400); HAL_Delay(10); } }",
            "t.c",
            RunOptions {
                time_limit_s: 2.0,
                ..Default::default()
            },
        )
    }

    #[test]
    fn render_includes_key_sections() {
        let s = render_report(&report());
        assert!(s.contains("仿真报告"));
        assert!(s.contains("静态检查"));
        assert!(s.contains("评分"));
    }

    #[test]
    fn bar_is_20_chars() {
        assert_eq!(bar(0, 40).chars().count(), 20);
        assert_eq!(bar(40, 40).chars().count(), 20);
    }
}

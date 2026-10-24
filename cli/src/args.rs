//! 极简命令行参数解析（不引入 clap，保持 CLI 零额外依赖）

#[derive(Debug)]
pub struct Args {
    pub command: Option<String>,
    /// 位置参数
    pub free: Vec<String>,
    pub track: String,
    pub dt: f64,
    pub time_limit: f64,
    pub seed: u64,
    pub sample_every: usize,
    pub out: Option<String>,
    pub csv: Option<String>,
    pub json: bool,
    pub quiet: bool,
    pub force: bool,
}

impl Args {
    pub fn parse(argv: &[String]) -> Self {
        let mut a = Self {
            command: None,
            free: Vec::new(),
            track: "track01".into(),
            dt: 0.01,
            time_limit: 60.0,
            seed: 20251025,
            sample_every: 10,
            out: None,
            csv: None,
            json: false,
            quiet: false,
            force: false,
        };

        let mut i = 1usize; // 跳过程序名
        while i < argv.len() {
            let s = argv[i].as_str();
            match s {
                "--track" => {
                    if let Some(v) = argv.get(i + 1) {
                        a.track = v.clone();
                        i += 1;
                    }
                }
                "--dt" => {
                    if let Some(v) = argv.get(i + 1).and_then(|x| x.parse().ok()) {
                        a.dt = v;
                        i += 1;
                    }
                }
                "--time-limit" => {
                    if let Some(v) = argv.get(i + 1).and_then(|x| x.parse().ok()) {
                        a.time_limit = v;
                        i += 1;
                    }
                }
                "--seed" => {
                    if let Some(v) = argv.get(i + 1).and_then(|x| x.parse().ok()) {
                        a.seed = v;
                        i += 1;
                    }
                }
                "--sample-every" => {
                    if let Some(v) = argv.get(i + 1).and_then(|x| x.parse().ok()) {
                        a.sample_every = v;
                        i += 1;
                    }
                }
                "--out" => {
                    if let Some(v) = argv.get(i + 1) {
                        a.out = Some(v.clone());
                        i += 1;
                    }
                }
                "--telemetry-csv" => {
                    if let Some(v) = argv.get(i + 1) {
                        a.csv = Some(v.clone());
                        i += 1;
                    }
                }
                "--json" => a.json = true,
                "--quiet" => a.quiet = true,
                "--force" => a.force = true,
                "--version" | "-V" => {
                    a.command = Some("version".into());
                    return a;
                }
                "--help" | "-h" => {
                    a.command = None;
                    return a;
                }
                other if other.starts_with('-') => {
                    // 未知选项：忽略，避免因为拼写错误直接崩掉
                }
                _ => {
                    if a.command.is_none() {
                        a.command = Some(s.to_string());
                    } else {
                        a.free.push(s.to_string());
                    }
                }
            }
            i += 1;
        }
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(s: &[&str]) -> Vec<String> {
        s.iter().map(|x| x.to_string()).collect()
    }

    #[test]
    fn parses_command_and_file() {
        let a = Args::parse(&v(&["car-sim", "run", "main.c"]));
        assert_eq!(a.command.as_deref(), Some("run"));
        assert_eq!(a.free, vec!["main.c".to_string()]);
    }

    #[test]
    fn parses_options() {
        let a = Args::parse(&v(&[
            "car-sim", "run", "main.c", "--track", "track02", "--dt", "0.005",
            "--time-limit", "30", "--seed", "7", "--out", "r.json", "--json",
        ]));
        assert_eq!(a.track, "track02");
        assert_eq!(a.dt, 0.005);
        assert_eq!(a.time_limit, 30.0);
        assert_eq!(a.seed, 7);
        assert_eq!(a.out.as_deref(), Some("r.json"));
        assert!(a.json);
    }

    #[test]
    fn defaults_are_sane() {
        let a = Args::parse(&v(&["car-sim", "check", "a.c"]));
        assert_eq!(a.track, "track01");
        assert_eq!(a.dt, 0.01);
        assert_eq!(a.time_limit, 60.0);
        assert_eq!(a.sample_every, 10);
        assert!(!a.json);
        assert!(!a.quiet);
        assert!(!a.force);
    }

    #[test]
    fn flags_without_values() {
        let a = Args::parse(&v(&["car-sim", "run", "a.c", "--quiet", "--force"]));
        assert!(a.quiet);
        assert!(a.force);
    }

    #[test]
    fn unknown_option_is_ignored() {
        let a = Args::parse(&v(&["car-sim", "run", "a.c", "--whatever"]));
        assert_eq!(a.command.as_deref(), Some("run"));
    }

    #[test]
    fn version_flag() {
        let a = Args::parse(&v(&["car-sim", "--version"]));
        assert_eq!(a.command.as_deref(), Some("version"));
    }

    #[test]
    fn csv_option() {
        let a = Args::parse(&v(&["car-sim", "run", "a.c", "--telemetry-csv", "t.csv"]));
        assert_eq!(a.csv.as_deref(), Some("t.csv"));
    }
}

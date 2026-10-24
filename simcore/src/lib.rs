//! STM32 智能车仿真内核
//!
//! 三块拼图：
//!   - [`physics`] 真实物理世界（车辆动力学 + 赛道 + 传感器）
//!   - [`csim`]     C 子集解释器（跑学生写的 STM32 代码）
//!   - [`sim`]      把两者缝起来的仿真驱动器

pub mod csim;
pub mod lint;
pub mod physics;
pub mod sim;

pub use physics::{CarConfig, CarState, Obstacle, Track, TrackPoint, World};
pub use sim::{run_file, RunOptions, SimReport};

/// 内核版本号
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 确定性伪随机数（LCG），保证同种子同结果
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let mut x = self.0;
        x ^= x >> 33;
        x = x.wrapping_mul(0xff51afd7ed558ccd);
        x ^= x >> 33;
        x
    }

    /// [0, 1)
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// [-1, 1]
    pub fn next_signed(&mut self) -> f64 {
        self.next_f64() * 2.0 - 1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_deterministic() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..50 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn rng_range() {
        let mut r = Rng::new(7);
        for _ in 0..1000 {
            let v = r.next_f64();
            assert!((0.0..1.0).contains(&v));
            let s = r.next_signed();
            assert!((-1.0..=1.0).contains(&s));
        }
    }
}

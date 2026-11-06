//! 平面の姿勢と速度

use serde::{Deserialize, Serialize};

/// 平面の姿勢(位置と向き)。座標系の変換としても使う
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose2 {
    pub const fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    /// `self` の座標系で表した `other` を、`self` の親の座標系で表す(self ∘ other)
    pub fn compose(&self, other: &Pose2) -> Pose2 {
        let (s, c) = self.theta.sin_cos();
        Pose2 {
            x: self.x + c * other.x - s * other.y,
            y: self.y + s * other.x + c * other.y,
            theta: normalize_angle(self.theta + other.theta),
        }
    }

    /// 逆の変換
    pub fn inverse(&self) -> Pose2 {
        let (s, c) = self.theta.sin_cos();
        Pose2 {
            x: -c * self.x - s * self.y,
            y: s * self.x - c * self.y,
            theta: normalize_angle(-self.theta),
        }
    }

    /// `self` の座標系の点を親の座標系に
    pub fn transform_point(&self, x: f64, y: f64) -> (f64, f64) {
        let (s, c) = self.theta.sin_cos();
        (self.x + c * x - s * y, self.y + s * x + c * y)
    }

    pub fn distance(&self, other: &Pose2) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

/// 角度を (-π, π] に
pub fn normalize_angle(a: f64) -> f64 {
    let mut a = a % std::f64::consts::TAU;
    if a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    } else if a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

/// ロボットの座標系の速度(vx 前、vy 左、vth 左回り)
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Velocity {
    pub vx: f64,
    pub vy: f64,
    pub vth: f64,
}

impl Velocity {
    pub const ZERO: Velocity = Velocity::new(0.0, 0.0, 0.0);

    pub const fn new(vx: f64, vy: f64, vth: f64) -> Self {
        Self { vx, vy, vth }
    }

    /// 並進の速さ
    pub fn trans(&self) -> f64 {
        self.vx.hypot(self.vy)
    }
}

/// 点 (px, py) から線分 a-b までの距離
pub fn point_segment_distance(px: f64, py: f64, a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((px - a.0) * dx + (py - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (px - (a.0 + t * dx)).hypot(py - (a.1 + t * dy))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn close(a: &Pose2, b: &Pose2) -> bool {
        (a.x - b.x).abs() < 1e-9
            && (a.y - b.y).abs() < 1e-9
            && normalize_angle(a.theta - b.theta).abs() < 1e-9
    }

    #[test]
    fn compose_and_inverse() {
        let a = Pose2::new(1.0, 2.0, FRAC_PI_2);
        let b = Pose2::new(1.0, 0.0, 0.0);
        assert!(close(&a.compose(&b), &Pose2::new(1.0, 3.0, FRAC_PI_2)));
        assert!(close(&a.compose(&a.inverse()), &Pose2::default()));
        assert!(close(&a.inverse().compose(&a), &Pose2::default()));
        let (x, y) = a.transform_point(0.0, 1.0);
        assert!((x - 0.0).abs() < 1e-9 && (y - 2.0).abs() < 1e-9);
    }

    #[test]
    fn angles() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-9);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-9);
        assert!((normalize_angle(0.5) - 0.5).abs() < 1e-12);
        assert!((normalize_angle(-7.0) - (-7.0 + 2.0 * PI)).abs() < 1e-9);
    }

    #[test]
    fn segment_distance() {
        assert!((point_segment_distance(0.5, 1.0, (0.0, 0.0), (1.0, 0.0)) - 1.0).abs() < 1e-12);
        assert!((point_segment_distance(2.0, 0.0, (0.0, 0.0), (1.0, 0.0)) - 1.0).abs() < 1e-12);
        assert!((point_segment_distance(3.0, 4.0, (0.0, 0.0), (0.0, 0.0)) - 5.0).abs() < 1e-12);
    }
}

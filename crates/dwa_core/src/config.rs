//! パラメータ。名前と既定値は ROS dwa_local_planner(と base_local_planner・costmap_2d の膨張)に合わせる

use serde::{Deserialize, Serialize};

/// DWA のパラメータ。単位は m、rad、s
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DwaConfig {
    // ---- 速度と加速度の制限 ----
    pub max_vel_x: f64,
    pub min_vel_x: f64,
    pub max_vel_y: f64,
    pub min_vel_y: f64,
    /// 並進の速さの上限
    pub max_vel_trans: f64,
    /// 並進の速さがこれより遅く、回転も `min_vel_theta` より遅い軌道は使わない
    pub min_vel_trans: f64,
    pub max_vel_theta: f64,
    pub min_vel_theta: f64,
    pub acc_lim_x: f64,
    pub acc_lim_y: f64,
    pub acc_lim_theta: f64,
    /// 止まったとみなす速さ(到着してから回るときに使う)
    pub trans_stopped_vel: f64,
    pub theta_stopped_vel: f64,
    /// false なら差動二輪(vy は 0 だけ)
    pub holonomic_robot: bool,

    // ---- 軌道の積分 ----
    /// 何秒先まで軌道を伸ばすか
    pub sim_time: f64,
    /// 軌道の点の間隔
    pub sim_granularity: f64,
    pub angular_sim_granularity: f64,
    pub vx_samples: usize,
    pub vy_samples: usize,
    pub vth_samples: usize,
    /// 制御の周期(動的窓の幅 = 加速度 × 1 / controller_frequency)
    pub controller_frequency: f64,

    // ---- コスト ----
    pub path_distance_bias: f64,
    pub goal_distance_bias: f64,
    pub occdist_scale: f64,
    /// 向きを評価する前方点の距離
    pub forward_point_distance: f64,

    // ---- 到着 ----
    pub xy_goal_tolerance: f64,
    pub yaw_goal_tolerance: f64,
    /// 一度 xy が入ったら、向きを合わせる間に少しずれても入ったままにする
    pub latch_xy_goal_tolerance: bool,

    // ---- ロボットと障害物 ----
    /// ロボットを円とみなした半径。障害物の点がこの内に入る軌道は使わない
    pub robot_radius: f64,
    /// ここまで障害物のコストを付ける(costmap_2d の inflation_radius)
    pub inflation_radius: f64,
    pub cost_scaling_factor: f64,

    // ---- 局所の窓とパスの刈り込み ----
    /// 局所ゴールを探す窓(ロボットを中心にした正方形)の一辺。ROS の局所コストマップの大きさに当たる
    pub local_window: f64,
    /// 通り過ぎたパスの点を捨てる
    pub prune_plan: bool,
}

impl Default for DwaConfig {
    fn default() -> Self {
        Self {
            max_vel_x: 0.55,
            min_vel_x: 0.0,
            max_vel_y: 0.1,
            min_vel_y: -0.1,
            max_vel_trans: 0.55,
            min_vel_trans: 0.1,
            max_vel_theta: 1.0,
            min_vel_theta: 0.4,
            acc_lim_x: 2.5,
            acc_lim_y: 2.5,
            acc_lim_theta: 3.2,
            trans_stopped_vel: 0.1,
            theta_stopped_vel: 0.1,
            holonomic_robot: false,
            sim_time: 1.7,
            sim_granularity: 0.025,
            angular_sim_granularity: 0.1,
            vx_samples: 3,
            vy_samples: 10,
            vth_samples: 20,
            controller_frequency: 10.0,
            path_distance_bias: 32.0,
            goal_distance_bias: 24.0,
            occdist_scale: 0.01,
            forward_point_distance: 0.325,
            xy_goal_tolerance: 0.10,
            yaw_goal_tolerance: 0.10,
            latch_xy_goal_tolerance: false,
            robot_radius: 0.2,
            inflation_radius: 0.55,
            cost_scaling_factor: 10.0,
            local_window: 3.0,
            prune_plan: true,
        }
    }
}

impl DwaConfig {
    /// 制御の周期
    pub fn sim_period(&self) -> f64 {
        1.0 / self.controller_frequency
    }

    /// 値の範囲を確かめる
    pub fn validate(&self) -> Result<(), String> {
        let positive = [
            ("max_vel_trans", self.max_vel_trans),
            ("max_vel_theta", self.max_vel_theta),
            ("acc_lim_x", self.acc_lim_x),
            ("acc_lim_theta", self.acc_lim_theta),
            ("sim_time", self.sim_time),
            ("sim_granularity", self.sim_granularity),
            ("angular_sim_granularity", self.angular_sim_granularity),
            ("controller_frequency", self.controller_frequency),
            ("xy_goal_tolerance", self.xy_goal_tolerance),
            ("yaw_goal_tolerance", self.yaw_goal_tolerance),
            ("local_window", self.local_window),
        ];
        for (name, v) in positive {
            if !(v.is_finite() && v > 0.0) {
                return Err(format!("{name} must be positive, got {v}"));
            }
        }
        let non_negative = [
            ("min_vel_trans", self.min_vel_trans),
            ("min_vel_theta", self.min_vel_theta),
            ("path_distance_bias", self.path_distance_bias),
            ("goal_distance_bias", self.goal_distance_bias),
            ("occdist_scale", self.occdist_scale),
            ("forward_point_distance", self.forward_point_distance),
            ("robot_radius", self.robot_radius),
            ("inflation_radius", self.inflation_radius),
            ("cost_scaling_factor", self.cost_scaling_factor),
            ("trans_stopped_vel", self.trans_stopped_vel),
            ("theta_stopped_vel", self.theta_stopped_vel),
        ];
        for (name, v) in non_negative {
            if !(v.is_finite() && v >= 0.0) {
                return Err(format!("{name} must be zero or positive, got {v}"));
            }
        }
        if self.min_vel_x > self.max_vel_x {
            return Err(format!(
                "min_vel_x ({}) must not exceed max_vel_x ({})",
                self.min_vel_x, self.max_vel_x
            ));
        }
        if self.min_vel_y > self.max_vel_y {
            return Err(format!(
                "min_vel_y ({}) must not exceed max_vel_y ({})",
                self.min_vel_y, self.max_vel_y
            ));
        }
        if self.holonomic_robot && self.acc_lim_y <= 0.0 {
            return Err("acc_lim_y must be positive for a holonomic robot".to_owned());
        }
        if self.vx_samples == 0
            || self.vth_samples == 0
            || (self.holonomic_robot && self.vy_samples == 0)
        {
            return Err("vx_samples, vy_samples and vth_samples must be at least 1".to_owned());
        }
        if self.vx_samples * self.vy_samples.max(1) * self.vth_samples > 10_000 {
            return Err(
                "too many samples (vx_samples * vy_samples * vth_samples > 10000)".to_owned(),
            );
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_valid_and_partial_json_works() {
        DwaConfig::default().validate().unwrap();
        let c: DwaConfig = serde_json::from_str(r#"{"max_vel_x": 0.3, "vx_samples": 6}"#).unwrap();
        assert_eq!(c.max_vel_x, 0.3);
        assert_eq!(c.vx_samples, 6);
        assert_eq!(c.vth_samples, 20);
        assert!(serde_json::from_str::<DwaConfig>(r#"{"max_vel_z": 1.0}"#).is_err());
    }

    #[test]
    fn rejects_bad_values() {
        let c = DwaConfig {
            sim_time: 0.0,
            ..Default::default()
        };
        assert!(c.validate().unwrap_err().contains("sim_time"));
        let c = DwaConfig {
            min_vel_x: 1.0,
            max_vel_x: 0.5,
            ..Default::default()
        };
        assert!(c.validate().is_err());
        let c = DwaConfig {
            vth_samples: 0,
            ..Default::default()
        };
        assert!(c.validate().is_err());
    }
}

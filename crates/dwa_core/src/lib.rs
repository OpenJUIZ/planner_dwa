//! DWA(Dynamic Window Approach)のローカルプランナー。juiz に依存しない。
//!
//! ROS の dwa_local_planner(軌道の評価は base_local_planner)と同じ考え方・パラメータ名・既定値で、
//! グローバルパスとロボットの姿勢・速度・障害物の点から速度指令を求める。使い方は [`DwaPlanner`]。

pub mod config;
pub mod geometry;
pub mod planner;

pub use config::DwaConfig;
pub use geometry::{normalize_angle, Pose2, Velocity};
pub use planner::{DwaOutput, DwaPlanner, Status, Trajectory};

/// このクレートの版
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

//! planner_dwa の juiz コンポーネント。
//!
//! コンテナ `dwa_planner` がグローバルパス・自己位置・オドメトリ・scan を覚え、`dwa_compute` のたびに
//! DWA(dwa_core)で速度指令を求める。中身は [`node::DwaNode`]。
//!
//! | ContainerProcess | 引数 | 戻り値 |
//! | --- | --- | --- |
//! | `dwa_set_path` | `path`(planner_dijkstra の経路) | なし(トピック path を受ける) |
//! | `dwa_set_goal` | `goal: Pose2DStamped` | なし(今の姿勢から直線のパス) |
//! | `dwa_set_pose` | `pose: Pose2DStamped` | なし(トピック amcl_pose) |
//! | `dwa_set_odometry` | `odom: Pose2DStamped` | なし(トピック odom) |
//! | `dwa_set_scan` | `scan: LaserScan` | なし(トピック scan) |
//! | `dwa_set_laser_pose` | `pose: PoseStamped` | なし(トピック lidar_mount) |
//! | `dwa_compute` | なし | `Twist2D`(トピック cmd_vel に出す) |
//! | `dwa_cancel` | なし | なし |
//! | `dwa_set_params` | `params`(object) | 今のパラメータ |
//! | `dwa_get_status` | なし | 短い状態(トピック dwa_status に出せる) |
//! | `dwa_get_state` | なし | 画面用の状態 |

pub mod bridge;
pub mod node;

use juiz_sdk::anyhow::anyhow;
use juiz_sdk::prelude::*;
use juiz_std_types::geometry::{Pose2DStamped, PoseStamped, Twist2D};
use juiz_std_types::sensor::LaserScan;

use dwa_core::DwaConfig;
pub use dwa_core::VERSION;
pub use node::{DwaNode, NodeSettings, PoseSource};

fn err(e: String) -> JuizError {
    JuizError::InvalidValueError { message: e }
}

/// コンテナの引数から作る(テストからも使う)
#[allow(clippy::too_many_arguments)]
pub fn build_node(
    params: Option<&Value>,
    pose_source: Option<&Value>,
    frame_id: Option<&Value>,
    sensor_timeout: Option<&Value>,
    require_scan: Option<&Value>,
    stuck_time: Option<&Value>,
    bridge: Option<&Value>,
) -> Result<DwaNode, String> {
    let config: DwaConfig = match params {
        None | Some(Value::Null) => DwaConfig::default(),
        Some(v) => {
            juiz_sdk::serde_json::from_value(v.clone()).map_err(|e| format!("params: {e}"))?
        }
    };
    let mut s = NodeSettings::default();
    match pose_source {
        None | Some(Value::Null) => {}
        Some(Value::String(v)) => s.pose_source = PoseSource::parse(v)?,
        Some(other) => return Err(format!("pose_source must be a string, got {other}")),
    }
    match frame_id {
        None | Some(Value::Null) => {}
        Some(Value::String(v)) => s.frame_id = v.clone(),
        Some(other) => return Err(format!("frame_id must be a string, got {other}")),
    }
    match sensor_timeout {
        None | Some(Value::Null) => {}
        Some(v) => {
            s.sensor_timeout = v
                .as_f64()
                .filter(|t| *t > 0.0)
                .ok_or_else(|| format!("sensor_timeout must be a positive number, got {v}"))?
        }
    }
    match require_scan {
        None | Some(Value::Null) => {}
        Some(Value::Bool(b)) => s.require_scan = *b,
        Some(other) => return Err(format!("require_scan must be true or false, got {other}")),
    }
    match stuck_time {
        None | Some(Value::Null) => {}
        Some(v) => {
            s.stuck_time = v
                .as_f64()
                .filter(|t| *t >= 0.0)
                .ok_or_else(|| format!("stuck_time must be zero or a positive number, got {v}"))?
        }
    }
    let bridge = bridge::BridgeConfig::from_value(bridge)?;
    DwaNode::new(config, s, bridge)
}

#[juiz_component_container(
    description = "DWA local planner: follows a global path (map frame) and outputs velocity commands, modeled on ROS dwa_local_planner"
    arguments = {
        default = {
            pose_source = "amcl"
            frame_id = "map"
        }
        description = {
            params = "DWA parameters with the names of ROS dwa_local_planner (max_vel_x, acc_lim_theta, sim_time, vx_samples, path_distance_bias, ...) plus robot_radius, inflation_radius, local_window"
            pose_source = "amcl (amcl_pose and odom, default) or odom (use odom as the map pose)"
            frame_id = "frame_id of the map"
            sensor_timeout = "Stop when odom or scan is older than this [s] (default 1.0)"
            require_scan = "Do not move before a scan arrives (default true)"
            stuck_time = "Report the status stuck when following but not moving 5 cm for this time [s] (default 5, 0 disables)"
            bridge = "Workaround for juiz#449: {amcl: <URL of localization_amcl>, planner: <URL of planner_dijkstra>, rate} to read amcl_get_pose and planner_get_path over HTTP"
        }
    }
)]
#[allow(clippy::too_many_arguments)]
fn dwa_planner(
    name: String,
    pose_source: Option<Value>,
    frame_id: Option<Value>,
    params: Option<Value>,
    sensor_timeout: Option<Value>,
    require_scan: Option<Value>,
    stuck_time: Option<Value>,
    bridge: Option<Value>,
) -> JuizResult<Box<DwaNode>> {
    let _ = name;
    let n = build_node(
        params.as_ref(),
        pose_source.as_ref(),
        frame_id.as_ref(),
        sensor_timeout.as_ref(),
        require_scan.as_ref(),
        stuck_time.as_ref(),
        bridge.as_ref(),
    )
    .map_err(|e| anyhow!(err(e)))?;
    Ok(Box::new(n))
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Set the global path (the path of planner_dijkstra: {ok, poses: [Pose2D], ...}). ok: false or empty poses stops the robot"
    arguments = { default = "{\"path\": {\"ok\": true, \"poses\": []}}" }
)]
fn dwa_set_path(container: &mut ContainerImpl<DwaNode>, path: Value) -> JuizResult<()> {
    container.set_path(&path).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Set a straight path from the current pose to the goal (for the browser view and tests)"
    arguments = {
        default = "{\"goal\": {\"header\": {\"stamp\": {\"sec\": 0, \"nanosec\": 0}, \"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": 0.0, \"y\": 0.0}, \"theta\": 0.0}}}"
    }
)]
fn dwa_set_goal(container: &mut ContainerImpl<DwaNode>, goal: Pose2DStamped) -> JuizResult<()> {
    container.set_goal(&goal).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Set the robot pose in the map frame (subscribe it to the topic amcl_pose)"
    arguments = {
        default = "{\"pose\": {\"header\": {\"stamp\": {\"sec\": 0, \"nanosec\": 0}, \"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": 0.0, \"y\": 0.0}, \"theta\": 0.0}}}"
    }
)]
fn dwa_set_pose(container: &mut ContainerImpl<DwaNode>, pose: Pose2DStamped) -> JuizResult<()> {
    // ContainerImpl::set_pose は Transformations の書き込みなので別の名前にしてある
    container.set_robot_pose(pose);
    Ok(())
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Set the odometry (subscribe it to the topic odom of mobile_sim)"
    arguments = {
        default = "{\"odom\": {\"header\": {\"stamp\": {\"sec\": 0, \"nanosec\": 0}, \"frame_id\": \"odom0\"}, \"pose\": {\"position\": {\"x\": 0.0, \"y\": 0.0}, \"theta\": 0.0}}}"
    }
)]
fn dwa_set_odometry(container: &mut ContainerImpl<DwaNode>, odom: Pose2DStamped) -> JuizResult<()> {
    container.set_odometry(odom);
    Ok(())
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Set the laser scan used as obstacles (subscribe it to the topic scan of mobile_sim)"
)]
fn dwa_set_scan(container: &mut ContainerImpl<DwaNode>, scan: LaserScan) -> JuizResult<()> {
    container.set_scan(scan);
    Ok(())
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Set the mount of the LiDAR seen from the robot (subscribe it to the topic lidar_mount of mobile_sim)"
)]
fn dwa_set_laser_pose(container: &mut ContainerImpl<DwaNode>, pose: PoseStamped) -> JuizResult<()> {
    container.set_laser_pose(&pose);
    Ok(())
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Compute the velocity command by DWA (bind it to an execution context and publish it to cmd_vel). Zero when something is missing"
)]
fn dwa_compute(container: &mut ContainerImpl<DwaNode>) -> JuizResult<Twist2D> {
    Ok(container.compute())
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Drop the path and stop"
)]
fn dwa_cancel(container: &mut ContainerImpl<DwaNode>) -> JuizResult<()> {
    container.cancel();
    Ok(())
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Change some DWA parameters ({name: value}). Returns all the parameters"
    arguments = { default = "{\"params\": {}}" }
)]
fn dwa_set_params(container: &mut ContainerImpl<DwaNode>, params: Value) -> JuizResult<Value> {
    container.set_params(&params).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Short status: status, has_path, arrived, goal_distance, cmd, local_goal (can be published to dwa_status)"
)]
fn dwa_get_status(container: &ContainerImpl<DwaNode>) -> JuizResult<Value> {
    Ok(container.status())
}

#[juiz_component_container_process(
    container_type = "dwa_planner"
    description = "Pose, path, obstacles, candidate and chosen trajectories, parameters (for the browser view)"
)]
fn dwa_get_state(container: &ContainerImpl<DwaNode>) -> JuizResult<Value> {
    Ok(container.state())
}

juiz_component_manifest!(
    component_name = "planner_dwa"
    containers = {
        dwa_planner = [
            dwa_set_path,
            dwa_set_goal,
            dwa_set_pose,
            dwa_set_odometry,
            dwa_set_scan,
            dwa_set_laser_pose,
            dwa_compute,
            dwa_cancel,
            dwa_set_params,
            dwa_get_status,
            dwa_get_state
        ]
    }
);

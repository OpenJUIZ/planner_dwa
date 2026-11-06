//! コンテナ `dwa_planner` の中身。トピックで受けた値を覚え、周期ごとに dwa_core で速度指令を求める

use std::collections::VecDeque;
use std::time::Instant;

use dwa_core::{DwaConfig, DwaOutput, DwaPlanner, Pose2, Status, Velocity};
use juiz_sdk::serde_json::{self, json, Map, Value};
use juiz_std_types::common::{Header, Time};
use juiz_std_types::geometry::{Pose2DStamped, PoseStamped, Twist2D, Vector2};
use juiz_std_types::sensor::LaserScan;

use crate::bridge::{Bridge, BridgeConfig};

/// 自己位置をどこから得るか
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoseSource {
    /// amcl_pose(map)と odom から(map→odom を odom に掛ける)
    Amcl,
    /// odom をそのまま map の姿勢とみなす
    Odom,
}

impl PoseSource {
    pub fn parse(s: &str) -> Result<Self, String> {
        match s {
            "amcl" => Ok(Self::Amcl),
            "odom" => Ok(Self::Odom),
            other => Err(format!("pose_source must be amcl or odom, got {other}")),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Amcl => "amcl",
            Self::Odom => "odom",
        }
    }
}

/// コンテナの設定(DWA のパラメータ以外)
#[derive(Clone, Debug)]
pub struct NodeSettings {
    pub pose_source: PoseSource,
    /// パスと局所の軌道の frame_id
    pub frame_id: String,
    /// odom・scan がこれより古ければ止まる [s]
    pub sensor_timeout: f64,
    /// scan が無いうちは動かない
    pub require_scan: bool,
    /// パスを追っているのに、この時間 [s] で `stuck_distance` も進まなければ状態を stuck にする(0 で見ない)
    pub stuck_time: f64,
    pub stuck_distance: f64,
}

impl Default for NodeSettings {
    fn default() -> Self {
        Self {
            pose_source: PoseSource::Amcl,
            frame_id: "map".to_owned(),
            sensor_timeout: 1.0,
            require_scan: true,
            stuck_time: 5.0,
            stuck_distance: 0.05,
        }
    }
}

/// トピックが来なくなってから bridge の値を使うまでの時間 [s]
const TOPIC_PRIORITY: f64 = 1.0;
/// odom を覚えておく長さ [s](sim 時刻)
const ODOM_HISTORY: f64 = 5.0;
/// 画面に返す障害物の点・候補の軌道の点の数の上限
const MAX_OBSTACLES_IN_STATE: usize = 400;

#[derive(Clone, Debug)]
struct Odom {
    stamp: f64,
    pose: Pose2,
}

/// コンテナ `dwa_planner` の中身
pub struct DwaNode {
    planner: DwaPlanner,
    settings: NodeSettings,
    // ---- 受けた値 ----
    odom: VecDeque<Odom>,
    odom_at: Option<Instant>,
    velocity: Option<Velocity>,
    map_to_odom: Option<Pose2>,
    amcl_pose: Option<Pose2DStamped>,
    pose_at: Option<Instant>,
    scan: Option<LaserScan>,
    scan_at: Option<Instant>,
    laser_pose: Pose2,
    path_at: Option<Instant>,
    path_message: String,
    // ---- bridge ----
    bridge: Option<Bridge>,
    bridge_pose_seq: u64,
    bridge_path_sig: Option<String>,
    // ---- 結果 ----
    last: Option<DwaOutput>,
    last_status: String,
    last_cmd: Velocity,
    last_pose: Option<Pose2>,
    obstacles: Vec<(f64, f64)>,
    compute_ms: f64,
    count: u64,
    /// 立ち往生を見るための、追っている間の姿勢の履歴
    trail: VecDeque<(Instant, Pose2)>,
    path_count: u64,
}

impl DwaNode {
    pub fn new(
        config: DwaConfig,
        settings: NodeSettings,
        bridge: Option<BridgeConfig>,
    ) -> Result<Self, String> {
        Ok(Self {
            planner: DwaPlanner::new(config)?,
            settings,
            odom: VecDeque::new(),
            odom_at: None,
            velocity: None,
            map_to_odom: None,
            amcl_pose: None,
            pose_at: None,
            scan: None,
            scan_at: None,
            laser_pose: Pose2::default(),
            path_at: None,
            path_message: String::new(),
            bridge: bridge.map(Bridge::start),
            bridge_pose_seq: 0,
            bridge_path_sig: None,
            last: None,
            last_status: "idle".to_owned(),
            last_cmd: Velocity::ZERO,
            last_pose: None,
            obstacles: Vec::new(),
            compute_ms: 0.0,
            count: 0,
            trail: VecDeque::new(),
            path_count: 0,
        })
    }

    pub fn config(&self) -> &DwaConfig {
        self.planner.config()
    }

    // ---- 入力 ----

    /// planner_dijkstra の経路(`{ok, message, poses: [Pose2D], ...}`)、または `{poses: [...]}`。
    /// `ok: false` や空の poses ならパスを捨てて止まる
    pub fn set_path(&mut self, path: &Value) -> Result<(), String> {
        self.path_at = Some(Instant::now());
        self.apply_path(path)
    }

    fn apply_path(&mut self, path: &Value) -> Result<(), String> {
        let poses = parse_path(path)?;
        let ok = path.get("ok").and_then(Value::as_bool).unwrap_or(true);
        self.path_message = path
            .get("message")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_owned();
        self.path_count += 1;
        self.trail.clear();
        if ok && !poses.is_empty() {
            self.planner.set_path(poses);
        } else {
            self.planner.clear();
        }
        Ok(())
    }

    /// 今の姿勢から目標まで直線のパスを置く(画面・試験用)
    pub fn set_goal(&mut self, goal: &Pose2DStamped) -> Result<(), String> {
        let start = self
            .current_pose(Instant::now())
            .map_err(|e| format!("cannot set a goal: {e}"))?;
        let g = to_pose2(&goal.pose);
        let n = (start.distance(&g) / 0.05).ceil().max(1.0) as usize;
        let th = (g.y - start.y).atan2(g.x - start.x);
        let mut poses: Vec<Pose2> = (0..=n)
            .map(|i| {
                let t = i as f64 / n as f64;
                Pose2::new(
                    start.x + (g.x - start.x) * t,
                    start.y + (g.y - start.y) * t,
                    th,
                )
            })
            .collect();
        poses.last_mut().unwrap().theta = g.theta;
        self.path_at = Some(Instant::now());
        self.path_message = "straight line (dwa_set_goal)".to_owned();
        self.path_count += 1;
        self.planner.set_path(poses);
        Ok(())
    }

    pub fn cancel(&mut self) {
        self.planner.clear();
        self.path_message = "cancelled".to_owned();
        self.last_cmd = Velocity::ZERO;
    }

    /// 自己位置(map)。同じ時刻の odom との差を map→odom として覚える
    pub fn set_robot_pose(&mut self, pose: Pose2DStamped) {
        self.pose_at = Some(Instant::now());
        self.apply_pose(pose);
    }

    fn apply_pose(&mut self, pose: Pose2DStamped) {
        let stamp = to_sec(&pose.header.stamp);
        let p = to_pose2(&pose.pose);
        if let Some(o) = self.odom_near(stamp) {
            self.map_to_odom = Some(p.compose(&o.inverse()));
        } else {
            // odom がまだ無い: odom を原点とみなす
            self.map_to_odom = Some(p);
        }
        self.amcl_pose = Some(pose);
    }

    /// オドメトリ(odom 座標系)。差分から今の速度も求める
    pub fn set_odometry(&mut self, odom: Pose2DStamped) {
        self.odom_at = Some(Instant::now());
        let cur = Odom {
            stamp: to_sec(&odom.header.stamp),
            pose: to_pose2(&odom.pose),
        };
        if let Some(prev) = self.odom.back() {
            let dt = cur.stamp - prev.stamp;
            if dt < -1e-6 {
                // 時刻が戻った(シミュレータを作り直した): 覚えた値を捨てる
                self.odom.clear();
                self.velocity = None;
            } else if dt > 1e-6 {
                let d = prev.pose.inverse().compose(&cur.pose);
                let v = Velocity::new(d.x / dt, d.y / dt, d.theta / dt);
                // 少しならす
                self.velocity = Some(match self.velocity {
                    Some(old) => Velocity::new(
                        0.5 * (old.vx + v.vx),
                        0.5 * (old.vy + v.vy),
                        0.5 * (old.vth + v.vth),
                    ),
                    None => v,
                });
            }
        }
        self.odom.push_back(cur);
        let newest = self.odom.back().unwrap().stamp;
        while self.odom.len() > 2 && newest - self.odom.front().unwrap().stamp > ODOM_HISTORY {
            self.odom.pop_front();
        }
    }

    pub fn set_scan(&mut self, scan: LaserScan) {
        self.scan_at = Some(Instant::now());
        self.scan = Some(scan);
    }

    /// LiDAR の取付位置(ロボットから見た姿勢)
    pub fn set_laser_pose(&mut self, pose: &PoseStamped) {
        let q = &pose.pose.orientation;
        let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        self.laser_pose = Pose2::new(pose.pose.position.x, pose.pose.position.y, yaw);
    }

    pub fn set_params(&mut self, params: &Value) -> Result<Value, String> {
        let mut base = serde_json::to_value(self.planner.config()).map_err(|e| e.to_string())?;
        merge(&mut base, params)?;
        let cfg: DwaConfig = serde_json::from_value(base).map_err(|e| e.to_string())?;
        self.planner.set_config(cfg)?;
        serde_json::to_value(self.planner.config()).map_err(|e| e.to_string())
    }

    // ---- 1 周期 ----

    /// 速度指令を求める。足りないものがあれば止まる(0 を返す)
    pub fn compute(&mut self) -> Twist2D {
        let now = Instant::now();
        let t0 = Instant::now();
        self.poll_bridge(now);
        self.count += 1;
        let cmd = match self.compute_inner(now) {
            Ok(out) => {
                self.last_status = status_name(out.status).to_owned();
                // 進める軌道が無くてその場で回るのと、回って追うのを繰り返すことがあるので、両方を立ち往生の見張りに入れる
                let trying = matches!(out.status, Status::Following | Status::NoTrajectory);
                if trying && self.is_stuck(now) {
                    self.last_status = "stuck".to_owned();
                }
                if !trying {
                    self.trail.clear();
                }
                let cmd = out.cmd;
                self.last = Some(out);
                cmd
            }
            Err(reason) => {
                self.trail.clear();
                self.last_status = reason;
                self.last = None;
                Velocity::ZERO
            }
        };
        self.last_cmd = cmd;
        self.compute_ms = t0.elapsed().as_secs_f64() * 1e3;
        twist(&cmd)
    }

    fn compute_inner(&mut self, now: Instant) -> Result<DwaOutput, String> {
        let fresh = |at: Option<Instant>| {
            at.is_some_and(|t| (now - t).as_secs_f64() <= self.settings.sensor_timeout)
        };
        let pose = self.current_pose(now);
        self.last_pose = pose.as_ref().ok().copied();
        // 障害物の点は、パスが無くても画面に出せるように毎周期求める
        self.obstacles = match (&self.scan, &pose) {
            (Some(scan), Ok(p)) if fresh(self.scan_at) => {
                scan_points(scan, &p.compose(&self.laser_pose))
            }
            _ => Vec::new(),
        };
        if !self.planner.has_path() {
            // パスが無ければ止まっているだけ(センサを待たない)
            return Ok(self.planner.compute(
                &self.last_pose.unwrap_or_default(),
                &Velocity::ZERO,
                &[],
            ));
        }
        let pose = pose?;
        if !fresh(self.odom_at) {
            return Err("waiting_for_odom".to_owned());
        }
        if self.settings.require_scan && !fresh(self.scan_at) {
            return Err("waiting_for_scan".to_owned());
        }
        let vel = self.velocity.unwrap_or(self.last_cmd);
        Ok(self.planner.compute(&pose, &vel, &self.obstacles))
    }

    /// 今の姿勢(map)
    pub fn current_pose(&self, _now: Instant) -> Result<Pose2, String> {
        let odom = self.odom.back().map(|o| o.pose);
        match self.settings.pose_source {
            PoseSource::Odom => odom.ok_or_else(|| "waiting_for_odom".to_owned()),
            PoseSource::Amcl => {
                let m2o = self
                    .map_to_odom
                    .ok_or_else(|| "waiting_for_pose".to_owned())?;
                Ok(match odom {
                    Some(o) => m2o.compose(&o),
                    None => m2o,
                })
            }
        }
    }

    /// トピックが来ていないときだけ、bridge が読んだ値を使う
    fn poll_bridge(&mut self, now: Instant) {
        let Some(bridge) = &self.bridge else { return };
        let inbox = bridge.inbox();
        let quiet =
            |at: Option<Instant>| at.is_none_or(|t| (now - t).as_secs_f64() > TOPIC_PRIORITY);
        if quiet(self.pose_at) {
            if let Some((seq, v)) = inbox.pose {
                if seq != self.bridge_pose_seq {
                    self.bridge_pose_seq = seq;
                    if let Ok(p) = <Pose2DStamped as juiz_sdk::types::JuizType>::from_value(&v) {
                        self.apply_pose(p);
                    }
                }
            }
        }
        if quiet(self.path_at) {
            if let Some((_, v)) = inbox.path {
                // planner_get_path は同じ経路を返し続けるので、変わったときだけ使う(cancel したものを戻さない)
                let sig = path_signature(&v);
                if self.bridge_path_sig.as_deref() != Some(sig.as_str()) {
                    let first = self.bridge_path_sig.is_none();
                    self.bridge_path_sig = Some(sig);
                    // 起動前に求めた古い経路(または「まだ無い」)は走らない
                    let planned = v.get("ok").and_then(Value::as_bool).unwrap_or(false);
                    if !first || !planned {
                        let _ = self.apply_path(&v);
                    }
                }
            }
        }
    }

    // ---- 出力 ----

    /// manager 向けの短い状態(トピック dwa_status に出せる)
    pub fn status(&self) -> Value {
        let out = self.last.as_ref();
        json!({
            "status": self.last_status,
            "has_path": self.planner.has_path(),
            "arrived": self.planner.is_arrived(),
            "goal_distance": out.and_then(|o| o.goal_distance),
            "cmd": vel_json(&self.last_cmd),
            "local_goal": out.and_then(|o| o.local_goal).map(|p| pose_json(&p)),
            "count": self.count,
        })
    }

    /// 画面用の状態
    pub fn state(&self) -> Value {
        let out = self.last.as_ref();
        let stride = (self.obstacles.len() / MAX_OBSTACLES_IN_STATE).max(1);
        let path = self.planner.path();
        let path_stride = (path.len() / 500).max(1);
        let traj = |t: &dwa_core::Trajectory, every: usize| {
            let n = t.points.len();
            let pts: Vec<Value> = t
                .points
                .iter()
                .enumerate()
                .filter(|(i, _)| i % every == 0 || *i == n - 1)
                .map(|(_, p)| json!([p.x, p.y]))
                .collect();
            json!({
                "vel": vel_json(&t.vel),
                "valid": t.is_valid(),
                "cost": t.cost,
                "path_cost": t.path_cost,
                "goal_cost": t.goal_cost,
                "occ_cost": t.occ_cost,
                "points": pts,
            })
        };
        let bridge = self.bridge.as_ref().map(|b| {
            let i = b.inbox();
            json!({
                "config": b.config.to_json(),
                "amcl_error": i.amcl_error,
                "planner_error": i.planner_error,
                "has_pose": i.pose.is_some(),
                "has_path": i.path.is_some(),
            })
        });
        json!({
            "status": self.last_status,
            "frame_id": self.settings.frame_id,
            "pose_source": self.settings.pose_source.name(),
            "pose": self.last_pose.map(|p| pose_json(&p)),
            "velocity": self.velocity.map(|v| vel_json(&v)),
            "cmd": vel_json(&self.last_cmd),
            "map_to_odom": self.map_to_odom.map(|p| pose_json(&p)),
            "amcl_pose": self.amcl_pose.as_ref().map(|p| pose_json(&to_pose2(&p.pose))),
            "laser_pose": pose_json(&self.laser_pose),
            "path": path.iter().step_by(path_stride).chain(path.last()).map(|p| json!([p.x, p.y])).collect::<Vec<_>>(),
            "path_goal": path.last().map(pose_json),
            "path_message": self.path_message,
            "path_count": self.path_count,
            "progress": self.planner.progress(),
            "arrived": self.planner.is_arrived(),
            "local_goal": out.and_then(|o| o.local_goal).map(|p| pose_json(&p)),
            "goal_distance": out.and_then(|o| o.goal_distance),
            "best": out.and_then(|o| o.best.as_ref()).map(|t| traj(t, 1)),
            "candidates": out.map(|o| o.candidates.iter().map(|t| traj(t, 8)).collect::<Vec<_>>()).unwrap_or_default(),
            "obstacles": self.obstacles.iter().step_by(stride).map(|(x, y)| json!([x, y])).collect::<Vec<_>>(),
            "params": serde_json::to_value(self.planner.config()).unwrap_or(Value::Null),
            "bridge": bridge,
            "compute_ms": self.compute_ms,
            "count": self.count,
        })
    }

    /// 追っている間の姿勢を覚え、`stuck_time` の間ほとんど進んでいなければ true
    fn is_stuck(&mut self, now: Instant) -> bool {
        let t = self.settings.stuck_time;
        let Some(pose) = self.last_pose else {
            return false;
        };
        if t <= 0.0 {
            return false;
        }
        self.trail.push_back((now, pose));
        while self.trail.len() > 1 && (now - self.trail[1].0).as_secs_f64() >= t {
            self.trail.pop_front();
        }
        let (t0, p0) = self.trail[0];
        (now - t0).as_secs_f64() >= t
            && self
                .trail
                .iter()
                .all(|(_, p)| p.distance(&p0) < self.settings.stuck_distance)
    }

    fn odom_near(&self, stamp: f64) -> Option<Pose2> {
        // 同じ時刻の odom(無ければいちばん近いもの。0.5 s より離れていれば最新)
        let best = self
            .odom
            .iter()
            .min_by(|a, b| (a.stamp - stamp).abs().total_cmp(&(b.stamp - stamp).abs()))?;
        if (best.stamp - stamp).abs() <= 0.5 {
            Some(best.pose)
        } else {
            self.odom.back().map(|o| o.pose)
        }
    }
}

fn status_name(s: Status) -> &'static str {
    match s {
        Status::Idle => "idle",
        Status::Following => "following",
        Status::Rotating => "rotating",
        Status::Arrived => "arrived",
        Status::NoTrajectory => "no_trajectory",
    }
}

/// 経路の JSON から姿勢の列を読む
pub fn parse_path(path: &Value) -> Result<Vec<Pose2>, String> {
    let poses = match path.get("poses") {
        None | Some(Value::Null) => return Ok(Vec::new()),
        Some(Value::Array(a)) => a,
        Some(other) => return Err(format!("path.poses must be an array, got {other}")),
    };
    poses
        .iter()
        .enumerate()
        .map(|(i, p)| {
            // Pose2D({position: {x, y}, theta})、PoseStamped 風の {pose: ...}、[x, y(, theta)] を受ける
            let p = p.get("pose").unwrap_or(p);
            if let Some(a) = p.as_array() {
                let n = |k: usize| a.get(k).and_then(Value::as_f64);
                return match (n(0), n(1)) {
                    (Some(x), Some(y)) => Ok(Pose2::new(x, y, n(2).unwrap_or(0.0))),
                    _ => Err(format!("path.poses[{i}] must be [x, y, theta]")),
                };
            }
            let pos = p.get("position").unwrap_or(p);
            let x = pos.get("x").and_then(Value::as_f64);
            let y = pos.get("y").and_then(Value::as_f64);
            match (x, y) {
                (Some(x), Some(y)) => Ok(Pose2::new(
                    x,
                    y,
                    p.get("theta").and_then(Value::as_f64).unwrap_or(0.0),
                )),
                _ => Err(format!("path.poses[{i}] has no position.x/y")),
            }
        })
        .collect()
}

/// 経路が変わったかを見るための印(時刻・点の数・最後の点)
fn path_signature(v: &Value) -> String {
    let stamp = v.pointer("/header/stamp").cloned().unwrap_or(Value::Null);
    let n = v
        .get("poses")
        .and_then(Value::as_array)
        .map(|a| a.len())
        .unwrap_or(0);
    let last = v
        .get("poses")
        .and_then(Value::as_array)
        .and_then(|a| a.last())
        .cloned()
        .unwrap_or(Value::Null);
    format!(
        "{stamp}|{n}|{last}|{}",
        v.get("ok").cloned().unwrap_or(Value::Null)
    )
}

/// scan の点を map 座標系に。`lidar` は map から見た LiDAR の姿勢
pub fn scan_points(scan: &LaserScan, lidar: &Pose2) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(scan.ranges.len());
    for (i, r) in scan.ranges.iter().enumerate() {
        let r = *r as f64;
        if !r.is_finite() || r < scan.range_min as f64 || r >= scan.range_max as f64 {
            continue;
        }
        let a = scan.angle_min as f64 + i as f64 * scan.angle_increment as f64;
        out.push(lidar.transform_point(r * a.cos(), r * a.sin()));
    }
    out
}

fn merge(base: &mut Value, patch: &Value) -> Result<(), String> {
    let Value::Object(p) = patch else {
        return Err(format!("params must be an object, got {patch}"));
    };
    let b: &mut Map<String, Value> = base
        .as_object_mut()
        .ok_or("internal: config is not an object")?;
    for (k, v) in p {
        if !b.contains_key(k) {
            return Err(format!("unknown parameter: {k}"));
        }
        b.insert(k.clone(), v.clone());
    }
    Ok(())
}

pub fn to_pose2(p: &juiz_std_types::geometry::Pose2D) -> Pose2 {
    Pose2::new(p.position.x, p.position.y, p.theta)
}

fn to_sec(t: &Time) -> f64 {
    t.sec as f64 + t.nanosec as f64 * 1e-9
}

pub fn twist(v: &Velocity) -> Twist2D {
    Twist2D {
        linear: Vector2 { x: v.vx, y: v.vy },
        angular: v.vth,
    }
}

fn pose_json(p: &Pose2) -> Value {
    json!({"x": p.x, "y": p.y, "theta": p.theta})
}

fn vel_json(v: &Velocity) -> Value {
    json!({"vx": v.vx, "vy": v.vy, "vth": v.vth})
}

/// テスト用
pub fn stamped(x: f64, y: f64, theta: f64, sec: f64, frame: &str) -> Pose2DStamped {
    Pose2DStamped {
        header: Header {
            stamp: Time {
                sec: sec.floor() as i64,
                nanosec: ((sec - sec.floor()) * 1e9).round() as i32,
            },
            frame_id: frame.to_owned(),
        },
        pose: juiz_std_types::geometry::Pose2D {
            position: juiz_std_types::geometry::Point2D { x, y },
            theta,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use juiz_std_types::geometry::{Point3D, Pose, Quaternion};

    fn node(source: PoseSource) -> DwaNode {
        let cfg = DwaConfig {
            max_vel_x: 0.5,
            max_vel_trans: 0.5,
            acc_lim_x: 1.0,
            acc_lim_theta: 3.0,
            min_vel_theta: 0.2,
            vx_samples: 6,
            ..Default::default()
        };
        let settings = NodeSettings {
            pose_source: source,
            ..Default::default()
        };
        DwaNode::new(cfg, settings, None).unwrap()
    }

    fn scan(ranges: Vec<f32>) -> LaserScan {
        let n = ranges.len();
        LaserScan {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "lidar0".to_owned(),
            },
            angle_min: -1.0,
            angle_max: 1.0,
            angle_increment: 2.0 / (n as f32 - 1.0),
            range_min: 0.05,
            range_max: 8.0,
            ranges,
            time_increment: 0.0,
            scan_time: 0.0,
            intensities: vec![],
        }
    }

    fn path_json(points: &[(f64, f64)]) -> Value {
        json!({
            "ok": true,
            "message": "",
            "header": {"stamp": {"sec": 1, "nanosec": 0}, "frame_id": "map"},
            "poses": points.iter().map(|(x, y)| json!({"position": {"x": x, "y": y}, "theta": 0.0})).collect::<Vec<_>>(),
        })
    }

    #[test]
    fn waits_for_inputs_then_drives() {
        let mut n = node(PoseSource::Amcl);
        // パスが無ければ idle
        assert_eq!(n.compute().linear.x, 0.0);
        assert_eq!(n.status()["status"], "idle");
        n.set_path(&path_json(&[(0.0, 0.0), (1.0, 0.0), (3.0, 0.0)]))
            .unwrap();
        n.compute();
        assert_eq!(n.status()["status"], "waiting_for_pose");
        n.set_odometry(stamped(0.0, 0.0, 0.0, 10.0, "odom0"));
        n.compute();
        assert_eq!(n.status()["status"], "waiting_for_pose");
        n.set_robot_pose(stamped(0.0, 0.0, 0.0, 10.0, "map"));
        n.compute();
        assert_eq!(n.status()["status"], "waiting_for_scan");
        n.set_scan(scan(vec![f32::INFINITY; 31]));
        let cmd = n.compute();
        assert_eq!(n.status()["status"], "following");
        assert!(cmd.linear.x > 0.0, "{cmd:?}");
        let st = n.state();
        assert!(st["best"]["points"].as_array().unwrap().len() > 2);
        assert!(!st["candidates"].as_array().unwrap().is_empty());
        n.cancel();
        assert_eq!(n.compute().linear.x, 0.0);
        assert_eq!(n.status()["status"], "idle");
    }

    #[test]
    fn combines_amcl_pose_and_odometry() {
        let mut n = node(PoseSource::Amcl);
        // odom は map から (1, 2, 90°) ずれている
        n.set_odometry(stamped(0.0, 0.0, 0.0, 1.0, "odom0"));
        n.set_odometry(stamped(0.5, 0.0, 0.0, 2.0, "odom0"));
        let m2o = Pose2::new(1.0, 2.0, std::f64::consts::FRAC_PI_2);
        let at1 = m2o.compose(&Pose2::new(0.0, 0.0, 0.0));
        // AMCL は時刻 1 の姿勢を出した(時刻 2 の odom ではなく時刻 1 の odom と組む)
        n.set_robot_pose(stamped(at1.x, at1.y, at1.theta, 1.0, "map"));
        let p = n.current_pose(Instant::now()).unwrap();
        let want = m2o.compose(&Pose2::new(0.5, 0.0, 0.0));
        assert!(p.distance(&want) < 1e-9, "{p:?} {want:?}");
        // odom の差分の速度
        let v = n.velocity.unwrap();
        assert!((v.vx - 0.5).abs() < 1e-9, "{v:?}");
        // odom で動けば姿勢も動く
        n.set_odometry(stamped(1.0, 0.0, 0.0, 3.0, "odom0"));
        let p = n.current_pose(Instant::now()).unwrap();
        assert!(p.distance(&m2o.compose(&Pose2::new(1.0, 0.0, 0.0))) < 1e-9);
    }

    #[test]
    fn odom_source_and_scan_obstacles() {
        let mut n = node(PoseSource::Odom);
        n.set_odometry(stamped(1.0, 1.0, 0.0, 1.0, "odom0"));
        assert_eq!(
            n.current_pose(Instant::now()).unwrap(),
            Pose2::new(1.0, 1.0, 0.0)
        );
        // LiDAR は前に 0.1 m、向きは左へ 90°
        let s = std::f64::consts::FRAC_PI_4.sin();
        n.set_laser_pose(&PoseStamped {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "robot0".to_owned(),
            },
            pose: Pose {
                position: Point3D {
                    x: 0.1,
                    y: 0.0,
                    z: 0.2,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: s,
                    w: s,
                },
            },
        });
        // 正面(角度 0、真ん中のビーム)に 1 m
        let mut r = vec![f32::INFINITY; 3];
        r[1] = 1.0;
        let pts = scan_points(&scan(r), &Pose2::new(1.0, 1.0, 0.0).compose(&n.laser_pose));
        assert_eq!(pts.len(), 1);
        assert!(
            (pts[0].0 - 1.1).abs() < 1e-6 && (pts[0].1 - 2.0).abs() < 1e-6,
            "{pts:?}"
        );
    }

    #[test]
    fn set_goal_draws_a_straight_path_and_failed_paths_stop() {
        let mut n = node(PoseSource::Odom);
        assert!(n.set_goal(&stamped(1.0, 0.0, 0.0, 0.0, "map")).is_err());
        n.set_odometry(stamped(0.0, 0.0, 0.0, 1.0, "odom0"));
        n.set_goal(&stamped(1.0, 1.0, 1.0, 0.0, "map")).unwrap();
        let path = n.planner.path();
        assert_eq!(path.len(), 30);
        assert_eq!(path.last().unwrap().theta, 1.0);
        n.set_path(&json!({"ok": false, "message": "the goal is on an obstacle", "poses": []}))
            .unwrap();
        assert!(!n.planner.has_path());
        assert_eq!(n.state()["path_message"], "the goal is on an obstacle");
        assert!(n.set_path(&json!({"poses": "x"})).is_err());
        assert_eq!(parse_path(&json!({"poses": [[1.0, 2.0], {"pose": {"position": {"x": 3, "y": 4}, "theta": 0.5}}]})).unwrap(),
            vec![Pose2::new(1.0, 2.0, 0.0), Pose2::new(3.0, 4.0, 0.5)]);
    }

    #[test]
    fn reports_stuck_when_no_trajectory_lasts() {
        let mut n = node(PoseSource::Odom);
        n.settings.stuck_time = 0.3;
        n.set_odometry(stamped(0.0, 0.0, 0.0, 1.0, "odom0"));
        // ロボットの半径より近くに点があり、どの軌道もぶつかる
        n.set_scan(scan(vec![0.15; 21]));
        n.set_path(&path_json(&[(0.0, 0.0), (3.0, 0.0)])).unwrap();
        n.compute();
        assert_eq!(n.status()["status"], "no_trajectory");
        std::thread::sleep(std::time::Duration::from_millis(350));
        n.set_odometry(stamped(0.0, 0.0, 0.0, 1.35, "odom0"));
        n.set_scan(scan(vec![0.15; 21]));
        n.compute();
        assert_eq!(n.status()["status"], "stuck");
    }

    #[test]
    fn reports_stuck_when_not_moving() {
        let mut n = node(PoseSource::Odom);
        n.settings.stuck_time = 0.3;
        n.settings.require_scan = false;
        n.set_odometry(stamped(0.0, 0.0, 0.0, 1.0, "odom0"));
        n.set_path(&path_json(&[(0.0, 0.0), (3.0, 0.0)])).unwrap();
        n.compute();
        assert_eq!(n.status()["status"], "following");
        std::thread::sleep(std::time::Duration::from_millis(350));
        // odom が古くならないように同じ姿勢を出し直す
        n.set_odometry(stamped(0.0, 0.0, 0.0, 1.0, "odom0"));
        n.compute();
        assert_eq!(n.status()["status"], "stuck");
        // 動けば戻る
        n.set_odometry(stamped(0.2, 0.0, 0.0, 1.5, "odom0"));
        n.compute();
        assert_eq!(n.status()["status"], "following");
    }

    #[test]
    fn set_params_merges() {
        let mut n = node(PoseSource::Odom);
        let v = n.set_params(&json!({"max_vel_x": 0.3})).unwrap();
        assert_eq!(v["max_vel_x"], 0.3);
        assert_eq!(v["vx_samples"], 6);
        assert!(n.set_params(&json!({"nope": 1})).is_err());
        assert!(n.set_params(&json!({"sim_time": -1.0})).is_err());
        assert_eq!(n.config().max_vel_x, 0.3);
    }
}

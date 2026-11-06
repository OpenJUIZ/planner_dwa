//! DWA のローカルプランナー本体
//!
//! ROS dwa_local_planner と同じ流れ:
//! 1. グローバルパスから通り過ぎた点を捨て、ロボットのまわりの窓(`local_window`)に入る部分を局所パスにし、その最後を局所ゴールにする
//! 2. 今の速度から 1 周期で届く速度の範囲(動的窓)を格子状にサンプルし、各速度で `sim_time` 先まで等速で積分する
//! 3. 障害物の点がロボットの円に入る軌道を捨て、残りをコストで比べる:
//!    `path_distance_bias`·(終点と前方点の局所パスからの距離) + `goal_distance_bias`·(終点と前方点の局所ゴールまでの距離)
//!    + `occdist_scale`·(軌道の上での障害物コストの最大)
//! 4. 最後の点に `xy_goal_tolerance` まで近づいたら、止まってからその場で回って向きを合わせる(ROS の LatchedStopRotateController)

use serde::Serialize;

use crate::config::DwaConfig;
use crate::geometry::{normalize_angle, point_segment_distance, Pose2, Velocity};

/// 障害物のコスト(costmap_2d と同じ値)
pub const LETHAL_COST: f64 = 254.0;
pub const INSCRIBED_COST: f64 = 253.0;

/// 1 周期の結果の状態
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    /// パスが無い(止まっている)
    Idle,
    /// パスを追っている
    Following,
    /// 最後の点に着いたので、止まってから向きを合わせている
    Rotating,
    /// 着いた(止まっている)
    Arrived,
    /// 使える軌道が無い(止まっている)
    NoTrajectory,
}

/// 評価した軌道
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Trajectory {
    pub vel: Velocity,
    /// 始点(今の姿勢)から `sim_time` 先までの姿勢
    pub points: Vec<Pose2>,
    /// 合計のコスト。負なら使えない(衝突など)
    pub cost: f64,
    pub path_cost: f64,
    pub goal_cost: f64,
    pub occ_cost: f64,
}

impl Trajectory {
    pub fn is_valid(&self) -> bool {
        self.cost >= 0.0
    }

    pub fn end(&self) -> Pose2 {
        *self.points.last().unwrap()
    }
}

/// 1 周期の結果
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct DwaOutput {
    pub cmd: Velocity,
    pub status: Status,
    /// 選んだ軌道(Following のとき)
    pub best: Option<Trajectory>,
    /// 評価したすべての軌道(画面用)
    pub candidates: Vec<Trajectory>,
    pub local_goal: Option<Pose2>,
    /// 局所パス(グローバルパスのうち窓に入る部分)
    pub local_path: Vec<Pose2>,
    /// 最後の点までの直線距離
    pub goal_distance: Option<f64>,
}

/// DWA のローカルプランナー。グローバルパスと、パスのどこまで進んだかを覚える
#[derive(Clone, Debug)]
pub struct DwaPlanner {
    config: DwaConfig,
    path: Vec<Pose2>,
    /// パスのうち、ここより前は通り過ぎた
    progress: usize,
    /// xy が入ったまま(latch_xy_goal_tolerance)
    xy_latched: bool,
    /// 着いた
    arrived: bool,
}

impl DwaPlanner {
    pub fn new(config: DwaConfig) -> Result<Self, String> {
        config.validate()?;
        Ok(Self {
            config,
            path: Vec::new(),
            progress: 0,
            xy_latched: false,
            arrived: false,
        })
    }

    pub fn config(&self) -> &DwaConfig {
        &self.config
    }

    pub fn set_config(&mut self, config: DwaConfig) -> Result<(), String> {
        config.validate()?;
        self.config = config;
        Ok(())
    }

    /// 新しいグローバルパス(map 座標系)。空ならパスを捨てる
    pub fn set_path(&mut self, path: Vec<Pose2>) {
        self.path = path;
        self.progress = 0;
        self.xy_latched = false;
        self.arrived = false;
    }

    pub fn clear(&mut self) {
        self.set_path(Vec::new());
    }

    pub fn path(&self) -> &[Pose2] {
        &self.path
    }

    pub fn progress(&self) -> usize {
        self.progress
    }

    pub fn has_path(&self) -> bool {
        !self.path.is_empty()
    }

    pub fn is_arrived(&self) -> bool {
        self.arrived
    }

    /// 1 周期分。`pose` は map 座標系の姿勢、`vel` はロボット座標系の今の速度、`obstacles` は map 座標系の障害物の点
    pub fn compute(&mut self, pose: &Pose2, vel: &Velocity, obstacles: &[(f64, f64)]) -> DwaOutput {
        let mut out = DwaOutput {
            cmd: Velocity::ZERO,
            status: Status::Idle,
            best: None,
            candidates: Vec::new(),
            local_goal: None,
            local_path: Vec::new(),
            goal_distance: None,
        };
        let Some(goal) = self.path.last().copied() else {
            return out;
        };
        out.goal_distance = Some(pose.distance(&goal));
        if self.arrived {
            out.status = Status::Arrived;
            return out;
        }

        // 1) 刈り込みと局所パス
        if self.config.prune_plan {
            self.prune(pose);
        }
        let local = self.local_path(pose);
        let local_goal = *local.last().unwrap();
        out.local_goal = Some(local_goal);
        out.local_path = local.clone();

        // 4) 最後の点に着いていれば、止まってから向きを合わせる
        let xy_reached = pose.distance(&goal) <= self.config.xy_goal_tolerance;
        if xy_reached && self.config.latch_xy_goal_tolerance {
            self.xy_latched = true;
        }
        if xy_reached || self.xy_latched {
            let (cmd, done) = self.stop_and_rotate(pose, vel, &goal, obstacles);
            out.cmd = cmd;
            if done {
                self.arrived = true;
                out.status = Status::Arrived;
            } else {
                out.status = Status::Rotating;
            }
            return out;
        }

        // 2) 3) 動的窓の中の軌道を比べる
        let near: Vec<(f64, f64)> = self.nearby_obstacles(pose, obstacles);
        let mut best: Option<usize> = None;
        for v in self.sample_velocities(vel) {
            let mut t = self.generate(pose, &v);
            self.score(&mut t, &local, &local_goal, &goal, &near);
            if t.is_valid() && best.is_none_or(|b| t.cost < out.candidates[b].cost) {
                best = Some(out.candidates.len());
            }
            out.candidates.push(t);
        }
        match best {
            Some(b) => {
                let t = out.candidates[b].clone();
                out.cmd = t.vel;
                out.status = Status::Following;
                out.best = Some(t);
            }
            None => out.status = Status::NoTrajectory,
        }
        out
    }

    /// 通り過ぎたパスの点を捨てる。今の位置にいちばん近い点(先へ `local_window` の範囲で探す)より前を通り過ぎたとする
    fn prune(&mut self, pose: &Pose2) {
        let reach = self.config.local_window;
        let mut best = self.progress;
        let mut best_d = f64::INFINITY;
        let mut along = 0.0;
        for i in self.progress..self.path.len() {
            if i > self.progress {
                along += self.path[i].distance(&self.path[i - 1]);
                if along > reach {
                    break;
                }
            }
            let d = pose.distance(&self.path[i]);
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        self.progress = best;
    }

    /// 進み具合から先のパスのうち、ロボットを中心にした窓に続けて入っている部分。少なくとも 1 点
    fn local_path(&self, pose: &Pose2) -> Vec<Pose2> {
        let half = self.config.local_window / 2.0;
        let inside = |p: &Pose2| (p.x - pose.x).abs() <= half && (p.y - pose.y).abs() <= half;
        let mut out = Vec::new();
        for p in &self.path[self.progress..] {
            if !inside(p) {
                break;
            }
            out.push(*p);
        }
        if out.is_empty() {
            out.push(self.path[self.progress]);
        }
        out
    }

    /// 軌道が届きうる範囲の障害物だけ
    fn nearby_obstacles(&self, pose: &Pose2, obstacles: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let reach = self.config.max_vel_trans.max(self.config.max_vel_x.abs())
            * self.config.sim_time
            + self.config.inflation_radius.max(self.config.robot_radius)
            + 0.1;
        obstacles
            .iter()
            .copied()
            .filter(|(x, y)| (x - pose.x).hypot(y - pose.y) <= reach)
            .collect()
    }

    /// 動的窓の中の速度
    pub fn sample_velocities(&self, vel: &Velocity) -> Vec<Velocity> {
        let c = &self.config;
        let dt = c.sim_period();
        let vx = range(
            (vel.vx - c.acc_lim_x * dt).max(c.min_vel_x),
            (vel.vx + c.acc_lim_x * dt).min(c.max_vel_x),
            c.vx_samples,
        );
        let vy = if c.holonomic_robot {
            range(
                (vel.vy - c.acc_lim_y * dt).max(c.min_vel_y),
                (vel.vy + c.acc_lim_y * dt).min(c.max_vel_y),
                c.vy_samples,
            )
        } else {
            vec![0.0]
        };
        let vth = range(
            (vel.vth - c.acc_lim_theta * dt).max(-c.max_vel_theta),
            (vel.vth + c.acc_lim_theta * dt).min(c.max_vel_theta),
            c.vth_samples,
        );
        let mut out = Vec::with_capacity(vx.len() * vy.len() * vth.len());
        for &x in &vx {
            for &y in &vy {
                for &th in &vth {
                    let v = Velocity::new(x, y, th);
                    // 並進が遅すぎ、回転も遅すぎるもの、速すぎるものは使わない(ROS の SimpleTrajectoryGenerator と同じ)
                    let trans = v.trans();
                    if trans > c.max_vel_trans + 1e-9 {
                        continue;
                    }
                    if trans < c.min_vel_trans && th.abs() < c.min_vel_theta {
                        continue;
                    }
                    out.push(v);
                }
            }
        }
        out
    }

    /// 速度 `v` で `sim_time` 先まで等速で進んだ軌道
    pub fn generate(&self, pose: &Pose2, v: &Velocity) -> Trajectory {
        let c = &self.config;
        let dist = v.trans() * c.sim_time;
        let ang = v.vth.abs() * c.sim_time;
        let steps = ((dist / c.sim_granularity).max(ang / c.angular_sim_granularity))
            .ceil()
            .max(1.0) as usize;
        let dt = c.sim_time / steps as f64;
        let mut points = Vec::with_capacity(steps + 1);
        let mut p = *pose;
        points.push(p);
        for _ in 0..steps {
            let (s, co) = p.theta.sin_cos();
            p.x += (v.vx * co - v.vy * s) * dt;
            p.y += (v.vx * s + v.vy * co) * dt;
            p.theta = normalize_angle(p.theta + v.vth * dt);
            points.push(p);
        }
        Trajectory {
            vel: *v,
            points,
            cost: 0.0,
            path_cost: 0.0,
            goal_cost: 0.0,
            occ_cost: 0.0,
        }
    }

    /// 軌道のコストを付ける。衝突するものは cost < 0
    fn score(
        &self,
        t: &mut Trajectory,
        local: &[Pose2],
        local_goal: &Pose2,
        goal: &Pose2,
        obstacles: &[(f64, f64)],
    ) {
        let c = &self.config;
        let mut occ: f64 = 0.0;
        for p in &t.points {
            let cost = self.obstacle_cost(p, obstacles);
            if cost < 0.0 {
                t.cost = -1.0;
                t.occ_cost = LETHAL_COST;
                return;
            }
            occ = occ.max(cost);
        }
        let end = t.end();
        let path_d = |p: &Pose2| path_distance(p, local);
        let mut path_cost = path_d(&end);
        let mut goal_cost = end.distance(local_goal);
        // 前方点: 向きの良さを測る。最後の点の近くでは使わない(ROS と同じ)
        if c.forward_point_distance > 0.0 && end.distance(goal) > c.forward_point_distance {
            let fp = end.compose(&Pose2::new(c.forward_point_distance, 0.0, 0.0));
            path_cost += path_d(&fp);
            goal_cost += fp.distance(local_goal);
        }
        t.path_cost = path_cost;
        t.goal_cost = goal_cost;
        t.occ_cost = occ;
        t.cost = c.path_distance_bias * path_cost
            + c.goal_distance_bias * goal_cost
            + c.occdist_scale * occ;
    }

    /// 姿勢 `p` での障害物のコスト(costmap_2d の膨張と同じ式)。ロボットの円に障害物が入れば -1
    pub fn obstacle_cost(&self, p: &Pose2, obstacles: &[(f64, f64)]) -> f64 {
        let c = &self.config;
        let mut d = f64::INFINITY;
        for (x, y) in obstacles {
            d = d.min((x - p.x).hypot(y - p.y));
        }
        if d <= c.robot_radius {
            -1.0
        } else if d <= c.inflation_radius {
            (INSCRIBED_COST - 1.0) * (-c.cost_scaling_factor * (d - c.robot_radius)).exp()
        } else {
            0.0
        }
    }

    /// 着いたあと: 止まっていなければ減速し、止まっていればその場で回って向きを合わせる。(指令, 終わったか)
    fn stop_and_rotate(
        &self,
        pose: &Pose2,
        vel: &Velocity,
        goal: &Pose2,
        obstacles: &[(f64, f64)],
    ) -> (Velocity, bool) {
        let c = &self.config;
        let dt = c.sim_period();
        let diff = normalize_angle(goal.theta - pose.theta);
        let stopped = vel.trans() <= c.trans_stopped_vel && vel.vth.abs() <= c.theta_stopped_vel;
        if diff.abs() <= c.yaw_goal_tolerance && stopped {
            return (Velocity::ZERO, true);
        }
        if vel.trans() > c.trans_stopped_vel {
            // 並進を加速度の制限の中で止める。回転も止める
            let slow = |v: f64, a: f64| {
                if v > 0.0 {
                    (v - a * dt).max(0.0)
                } else {
                    (v + a * dt).min(0.0)
                }
            };
            return (
                Velocity::new(
                    slow(vel.vx, c.acc_lim_x),
                    slow(vel.vy, c.acc_lim_y),
                    slow(vel.vth, c.acc_lim_theta),
                ),
                false,
            );
        }
        if diff.abs() <= c.yaw_goal_tolerance {
            // 向きは合ったが、まだ回っている
            return (Velocity::ZERO, false);
        }
        // その場で回る(ROS の LatchedStopRotateController::rotateToGoal と同じ):
        // 目標の速さを加速度の制限に収め、ちょうど止まれる速さ sqrt(2 a |diff|) で抑え、最後に [min_vel_theta, max_vel_theta] に収める
        let target = diff
            .abs()
            .clamp(c.min_vel_theta.min(c.max_vel_theta), c.max_vel_theta);
        let cur = vel.vth.abs();
        let speed = target
            .clamp(
                (cur - c.acc_lim_theta * dt).max(0.0),
                cur + c.acc_lim_theta * dt,
            )
            .min((2.0 * c.acc_lim_theta * diff.abs()).sqrt())
            .clamp(c.min_vel_theta.min(c.max_vel_theta), c.max_vel_theta);
        let cmd = Velocity::new(0.0, 0.0, speed.copysign(diff));
        // 回ると障害物に当たるなら止まる(円のロボットでは起きないが、念のため)
        if self.obstacle_cost(pose, obstacles) < 0.0 {
            return (Velocity::ZERO, false);
        }
        (cmd, false)
    }
}

/// 局所パス(折れ線)からの距離
fn path_distance(p: &Pose2, local: &[Pose2]) -> f64 {
    if local.len() == 1 {
        return p.distance(&local[0]);
    }
    local
        .windows(2)
        .map(|w| point_segment_distance(p.x, p.y, (w[0].x, w[0].y), (w[1].x, w[1].y)))
        .fold(f64::INFINITY, f64::min)
}

/// [lo, hi] を n 個に等分した値。0 をまたぐときは 0 も入れる(ROS の VelocityIterator と同じ)
fn range(lo: f64, hi: f64, n: usize) -> Vec<f64> {
    if hi < lo {
        return vec![];
    }
    if n <= 1 || (hi - lo).abs() < 1e-12 {
        return vec![(lo + hi) / 2.0];
    }
    let mut v: Vec<f64> = (0..n)
        .map(|i| lo + (hi - lo) * i as f64 / (n - 1) as f64)
        .collect();
    if lo < 0.0 && hi > 0.0 && !v.iter().any(|x| x.abs() < 1e-12) {
        v.push(0.0);
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    /// mobile_sim の既定のロボットに合わせた設定(app.conf と同じ)
    fn config() -> DwaConfig {
        DwaConfig {
            max_vel_x: 0.5,
            max_vel_trans: 0.5,
            max_vel_theta: 1.0,
            acc_lim_x: 1.0,
            acc_lim_theta: 3.0,
            min_vel_theta: 0.2,
            vx_samples: 6,
            ..Default::default()
        }
    }

    fn line(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Pose2> {
        let n = ((x1 - x0).hypot(y1 - y0) / 0.05).ceil() as usize;
        let th = (y1 - y0).atan2(x1 - x0);
        (0..=n)
            .map(|i| {
                let t = i as f64 / n as f64;
                Pose2::new(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, th)
            })
            .collect()
    }

    /// 指令どおりに動く(加速度の制限つき)ロボットで、着くか `steps` 周期まで走らせる。(最後の姿勢, 最後の結果, 障害物までの最小距離)
    fn drive(
        p: &mut DwaPlanner,
        start: Pose2,
        obstacles: &[(f64, f64)],
        steps: usize,
    ) -> (Pose2, DwaOutput, f64) {
        let dt = p.config().sim_period();
        let (ax, ath) = (1.0, 3.0);
        let mut pose = start;
        let mut vel = Velocity::ZERO;
        let mut min_d = f64::INFINITY;
        let mut out = p.compute(&pose, &vel, obstacles);
        for _ in 0..steps {
            if out.status == Status::Arrived || out.status == Status::NoTrajectory {
                break;
            }
            let cmd = out.cmd;
            vel.vx += (cmd.vx - vel.vx).clamp(-ax * dt, ax * dt);
            vel.vth += (cmd.vth - vel.vth).clamp(-ath * dt, ath * dt);
            let sub = 10;
            for _ in 0..sub {
                let h = dt / sub as f64;
                pose.x += vel.vx * pose.theta.cos() * h;
                pose.y += vel.vx * pose.theta.sin() * h;
                pose.theta = normalize_angle(pose.theta + vel.vth * h);
                for (x, y) in obstacles {
                    min_d = min_d.min((x - pose.x).hypot(y - pose.y));
                }
            }
            out = p.compute(&pose, &vel, obstacles);
            if std::env::var("DWA_DEBUG").is_ok() {
                eprintln!(
                    "{pose:?} {vel:?} {:?} {:?}",
                    out.status,
                    out.best
                        .as_ref()
                        .map(|b| (b.path_cost, b.goal_cost, b.occ_cost))
                );
            }
        }
        (pose, out, min_d)
    }

    #[test]
    fn idle_without_a_path() {
        let mut p = DwaPlanner::new(config()).unwrap();
        let out = p.compute(&Pose2::default(), &Velocity::ZERO, &[]);
        assert_eq!(out.status, Status::Idle);
        assert_eq!(out.cmd, Velocity::ZERO);
        assert!(!p.has_path());
    }

    #[test]
    fn samples_the_dynamic_window() {
        let p = DwaPlanner::new(config()).unwrap();
        let vs = p.sample_velocities(&Velocity::ZERO);
        // vx は [0, 0.1]、vth は [-0.3, 0.3](0 を含む)
        assert!(vs
            .iter()
            .all(|v| (0.0..=0.1 + 1e-9).contains(&v.vx) && v.vth.abs() <= 0.3 + 1e-9));
        assert!(vs.iter().all(|v| v.vy == 0.0));
        // 並進も回転も遅いもの(vx < 0.1 かつ |vth| < 0.2)は除く
        assert!(vs
            .iter()
            .all(|v| v.vx >= 0.1 - 1e-9 || v.vth.abs() >= 0.2 - 1e-9));
        assert!(
            vs.iter().any(|v| v.vx == 0.0 && v.vth.abs() > 0.29),
            "turning in place is sampled"
        );
        let moving = p.sample_velocities(&Velocity::new(0.5, 0.0, 0.0));
        assert!(moving
            .iter()
            .all(|v| (0.4 - 1e-9..=0.5 + 1e-9).contains(&v.vx)));
        assert_eq!(range(-1.0, 1.0, 4).len(), 5);
        assert_eq!(range(0.0, 1.0, 1), vec![0.5]);
        assert!(range(1.0, 0.0, 3).is_empty());
    }

    #[test]
    fn integrates_constant_velocity() {
        let p = DwaPlanner::new(config()).unwrap();
        let t = p.generate(
            &Pose2::new(1.0, 1.0, FRAC_PI_2),
            &Velocity::new(0.5, 0.0, 0.0),
        );
        let end = t.end();
        assert!((end.x - 1.0).abs() < 1e-9 && (end.y - (1.0 + 0.5 * 1.7)).abs() < 1e-9);
        assert_eq!(t.points.len(), 35); // 0.85 m / 0.025 m + 始点
                                        // 円弧: 半径 v/ω
        let t = p.generate(&Pose2::default(), &Velocity::new(0.5, 0.0, 0.5));
        let e = t.end();
        let r = 1.0;
        // オイラー積分(ROS と同じ)なので、刻みの分だけずれる
        assert!(
            (e.x - r * (0.85f64).sin()).abs() < 1e-2
                && (e.y - r * (1.0 - (0.85f64).cos())).abs() < 1e-2,
            "{e:?}"
        );
    }

    #[test]
    fn follows_a_straight_path_and_stops_at_the_goal() {
        let mut p = DwaPlanner::new(config()).unwrap();
        let mut path = line(0.0, 0.0, 3.0, 0.0);
        path.last_mut().unwrap().theta = FRAC_PI_2;
        p.set_path(path);
        let (pose, out, _) = drive(&mut p, Pose2::new(0.0, 0.0, 0.0), &[], 400);
        assert_eq!(out.status, Status::Arrived, "{pose:?}");
        assert!(
            pose.distance(&Pose2::new(3.0, 0.0, 0.0)) <= 0.1 + 0.05,
            "{pose:?}"
        );
        assert!(
            normalize_angle(pose.theta - FRAC_PI_2).abs() <= 0.1 + 0.05,
            "{pose:?}"
        );
        assert_eq!(out.cmd, Velocity::ZERO);
        assert!(p.is_arrived());
        // 着いたあとも止まったまま
        assert_eq!(
            p.compute(&pose, &Velocity::ZERO, &[]).status,
            Status::Arrived
        );
    }

    #[test]
    fn reaches_full_speed_on_a_long_path() {
        let mut p = DwaPlanner::new(config()).unwrap();
        p.set_path(line(0.0, 0.0, 10.0, 0.0));
        let mut vel = Velocity::ZERO;
        let mut pose = Pose2::default();
        for _ in 0..20 {
            let out = p.compute(&pose, &vel, &[]);
            assert_eq!(out.status, Status::Following);
            vel = out.cmd;
            pose = pose.compose(&Pose2::new(vel.vx * 0.1, 0.0, vel.vth * 0.1));
        }
        assert!((vel.vx - 0.5).abs() < 1e-9, "{vel:?}");
        assert!(vel.vth.abs() < 0.2, "{vel:?}");
        assert!(p.progress() > 0);
    }

    fn pillar(cx: f64, cy: f64, r: f64) -> Vec<(f64, f64)> {
        (0..24)
            .map(|i| {
                let a = i as f64 * PI / 12.0;
                (cx + r * a.cos(), cy + r * a.sin())
            })
            .collect()
    }

    #[test]
    fn swerves_around_an_obstacle_near_the_path() {
        let mut p = DwaPlanner::new(config()).unwrap();
        p.set_path(line(0.0, 0.0, 5.0, 0.0));
        // パスの脇(0.28 m)に半径 0.1 m の柱。まっすぐ進むと隙間は 0.18 m でロボットの半径 0.2 m に足りない
        let obstacles = pillar(2.5, 0.28, 0.1);
        let (pose, out, min_d) = drive(&mut p, Pose2::default(), &obstacles, 600);
        assert_eq!(out.status, Status::Arrived, "{pose:?} {:?}", out.status);
        assert!(min_d > 0.2, "came within {min_d} m of the obstacle");
    }

    #[test]
    fn stops_before_an_obstacle_blocking_the_path() {
        // パスの上に柱。DWA は回り込めない(グローバルパスを引き直すのは planner の役目)が、ぶつからずに止まる
        let mut p = DwaPlanner::new(config()).unwrap();
        p.set_path(line(0.0, 0.0, 5.0, 0.0));
        let obstacles = pillar(2.5, 0.0, 0.15);
        let (pose, _, min_d) = drive(&mut p, Pose2::default(), &obstacles, 300);
        assert!(min_d > 0.2, "came within {min_d} m of the obstacle");
        assert!(pose.x < 2.5, "{pose:?}");
    }

    #[test]
    fn turns_in_place_when_the_path_is_behind() {
        let mut p = DwaPlanner::new(config()).unwrap();
        p.set_path(line(0.0, 0.0, -3.0, 0.0));
        let out = p.compute(&Pose2::default(), &Velocity::ZERO, &[]);
        assert_eq!(out.status, Status::Following);
        assert!(
            out.cmd.vth.abs() > 0.2 && out.cmd.vx <= 0.05,
            "{:?}",
            out.cmd
        );
        let (pose, out, _) = drive(&mut p, Pose2::default(), &[], 600);
        assert_eq!(out.status, Status::Arrived, "{pose:?}");
        assert!(pose.distance(&Pose2::new(-3.0, 0.0, 0.0)) < 0.15);
    }

    #[test]
    fn rotates_to_the_goal_heading_after_stopping() {
        let mut p = DwaPlanner::new(config()).unwrap();
        p.set_path(vec![Pose2::new(0.0, 0.0, 0.0), Pose2::new(0.05, 0.0, PI)]);
        // まだ動いている: まず減速
        let out = p.compute(&Pose2::default(), &Velocity::new(0.4, 0.0, 0.0), &[]);
        assert_eq!(out.status, Status::Rotating);
        assert!(
            (out.cmd.vx - 0.3).abs() < 1e-9 && out.cmd.vth == 0.0,
            "{:?}",
            out.cmd
        );
        // 止まった: その場で回る(最初は加速度の制限 3.0 rad/s² × 0.1 s)
        let out = p.compute(&Pose2::default(), &Velocity::ZERO, &[]);
        assert_eq!(out.status, Status::Rotating);
        assert_eq!(out.cmd.vx, 0.0);
        assert!((out.cmd.vth.abs() - 0.3).abs() < 1e-9, "{:?}", out.cmd);
        // 回っている: 最大まで上がり、向きが近づくと止まれる速さに落とす
        let out = p.compute(&Pose2::default(), &Velocity::new(0.0, 0.0, 1.0), &[]);
        assert!((out.cmd.vth.abs() - 1.0).abs() < 1e-9, "{:?}", out.cmd);
        // 向きが近づくと減速する(加速度の制限で 1 周期に 0.3 rad/s まで)
        let out = p.compute(
            &Pose2::new(0.0, 0.0, PI - 0.2),
            &Velocity::new(0.0, 0.0, 1.0),
            &[],
        );
        assert!((out.cmd.vth - 0.7).abs() < 1e-9, "{:?}", out.cmd);
        // 向きが合って止まれば着いた
        let out = p.compute(&Pose2::new(0.0, 0.0, PI - 0.05), &Velocity::ZERO, &[]);
        assert_eq!(out.status, Status::Arrived);
        assert!(p.is_arrived());
    }

    #[test]
    fn stops_when_every_trajectory_collides() {
        let mut p = DwaPlanner::new(config()).unwrap();
        p.set_path(line(0.0, 0.0, 3.0, 0.0));
        // ロボットの円の中に障害物
        let out = p.compute(&Pose2::default(), &Velocity::ZERO, &[(0.1, 0.0)]);
        assert_eq!(out.status, Status::NoTrajectory);
        assert_eq!(out.cmd, Velocity::ZERO);
        assert!(!out.candidates.is_empty() && out.candidates.iter().all(|t| !t.is_valid()));
    }

    #[test]
    fn obstacle_cost_follows_the_inflation() {
        let p = DwaPlanner::new(config()).unwrap();
        let o = Pose2::default();
        assert_eq!(p.obstacle_cost(&o, &[(0.2, 0.0)]), -1.0);
        assert!((p.obstacle_cost(&o, &[(0.2 + 1e-9, 0.0)]) - 252.0).abs() < 1e-3);
        assert!((p.obstacle_cost(&o, &[(0.3, 0.0)]) - 252.0 * (-1.0f64).exp()).abs() < 1e-9);
        assert_eq!(p.obstacle_cost(&o, &[(0.6, 0.0)]), 0.0);
        assert_eq!(p.obstacle_cost(&o, &[]), 0.0);
    }

    #[test]
    fn prunes_the_passed_part_and_picks_the_local_goal() {
        let mut p = DwaPlanner::new(config()).unwrap();
        p.set_path(line(0.0, 0.0, 10.0, 0.0));
        let out = p.compute(&Pose2::new(4.0, 0.0, 0.0), &Velocity::ZERO, &[]);
        // 窓は 3 m 四方なので、局所ゴールは x = 5.5 付近
        let g = out.local_goal.unwrap();
        assert!((g.x - 5.5).abs() <= 0.05 + 1e-9, "{g:?}");
        // 先へは local_window(3 m)までしか探さないので、4 m 先へ飛んでも 3 m までしか進まない
        assert!(
            (p.path()[p.progress()].x - 3.0).abs() < 0.06,
            "{}",
            p.progress()
        );
        p.compute(&Pose2::new(4.0, 0.0, 0.0), &Velocity::ZERO, &[]);
        assert!((p.path()[p.progress()].x - 4.0).abs() < 1e-9);
        // 後ろへ戻っても進み具合は戻らない
        p.compute(&Pose2::new(1.0, 0.0, 0.0), &Velocity::ZERO, &[]);
        assert!((p.path()[p.progress()].x - 4.0).abs() < 1e-9);
        // 新しいパスで始めから
        p.set_path(line(0.0, 0.0, 1.0, 0.0));
        assert_eq!(p.progress(), 0);
        assert!(!p.is_arrived());
        p.clear();
        assert!(!p.has_path());
    }
}

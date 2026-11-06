# planner_dwa の計画

地図の上のグローバルパス(planner_dijkstra)とロボットの自己位置(localization_amcl)を受け取り、
DWA(Dynamic Window Approach)でロボットの速度指令を求めて mobile_sim に出す juizapp(ローカルプランナー)。
ROS の [dwa_local_planner](https://github.com/ros-planning/navigation/tree/noetic-devel/dwa_local_planner)
(軌道の評価は base_local_planner)を手本にし、パラメータの名前と既定値をそろえる。

依頼: ysuga(2026-11-06)「マップ上のグローバルなパスと、移動ロボットの自己位置を入力されると DWA 法で移動ロボットの速度司令値を計算するモジュールを作りたい」

これで mobile_sim・localization_amcl・planner_dijkstra とつながり、「地図の上で目標を置くとロボットがそこまで走る」一式がそろう。

## 方針

| 項目 | 決めたこと | 理由 |
| --- | --- | --- |
| 言語 | Rust(mobile_sim・planner_dijkstra と同じ) | 外部のライブラリが要らない。juiz 0.2 の Rust プラグインは C ABI なので rustc の版を問わない |
| 形 | juizapp、`kind: app`、`depends: mobile_sim >=0.3.1, <0.4` | 他のアプリと同じ。scan・odom を受け、cmd_vel を mobile_sim に出す |
| ポート | HTTP 8004、IPC 名 `planner_dwa` | mobile_sim 8000、mapper_gmapping 8001、localization_amcl 8002、planner_dijkstra 8003 の次 |
| アルゴリズム | ROS dwa_local_planner と同じ: 動的窓の中の速度を格子状にサンプルし、一定速度で `sim_time` 先まで積分し、コスト最小を選ぶ | 依頼のとおり。パラメータ名・既定値を ROS にそろえると、ROS の調整の知見がそのまま使える |
| コスト | `path_distance_bias`·パスからの距離 + `goal_distance_bias`·局所ゴールまでの距離 + `occdist_scale`·障害物コスト(+ 前方点の同じ 2 つ) | ROS と同じ重み。ROS は格子の数に解像度を掛けて m にするので、ここでも距離は m で数え、ROS の既定の重みがそのまま効く。ROS は地図の格子で幅優先の距離を使うが、ここではパスの折れ線・局所ゴールまでのユークリッド距離にする(地図を持たないので) |
| 障害物 | 最新の scan だけ(初版)。点からの距離が `robot_radius` 以下の軌道は捨て、`inflation_radius` までは ROS の膨張と同じ式でコストを付ける | グローバルパスがすでに壁を避けている前提。地図を重ねた局所コストマップは後の拡張 |
| 自己位置 | トピック `amcl_pose`(map)を受けたときの odom との差から map→odom を求め、最新の `odom` に掛けて使う(ROS の tf と同じ考え) | AMCL の更新は scan ごと。制御周期の間は odom で補う |
| 現在速度 | `odom` の差分から求める(無ければ最後の指令) | ROS も odom の速度を使う |
| 駆動 | 差動二輪を既定(`holonomic_robot: false`)。true で vy もサンプル | mobile_sim の既定は diff_drive |
| 周期 | 自分の System の main_loop_ec(10 Hz)に `dwa_compute` を bind し、結果(Twist2D)をトピック `cmd_vel` に出す | mobile_sim は 0.5 s 指令が途切れると止まるので、planner_dwa が落ちてもロボットは止まる |
| 到着 | `xy_goal_tolerance` に入ったら止まり、その場で回って `yaw_goal_tolerance` に合わせる(ROS の latched stop rotate) | ROS と同じ |
| 立ち往生 | パスを追っているのに(進める軌道が無く止まっている間も含めて) `stuck_time`(既定 5 s)で 5 cm も進まなければ状態を `stuck` にし、トピック `dwa_status` に出す | navigation_manager が見て経路を引き直す・止めるを決める |
| 画面 | http://localhost:8004/: mobile_sim の地図、グローバルパス、scan の点、候補の軌道と選んだ軌道、速度、停止ボタン、主な重みの調整、ドラッグで直線のパスを置く | planner_dijkstra が無くても試せるように |

## 入出力

| 向き | トピック | 型 | 出す側 | 受ける Process |
| --- | --- | --- | --- | --- |
| 入力 | `path` | planner_dijkstra の経路(JSON: `{header, ok, poses: [Pose2D], ...}`、map) | planner_dijkstra | `dwa_set_path` |
| 入力 | `amcl_pose` | Pose2DStamped(map) | localization_amcl | `dwa_set_pose` |
| 入力 | `odom` | Pose2DStamped(odom0) | mobile_sim | `dwa_set_odometry` |
| 入力 | `scan` | LaserScan(lidar0) | mobile_sim | `dwa_set_scan` |
| 入力 | `lidar_mount` | PoseStamped(robot0 から見た lidar0) | mobile_sim | `dwa_set_laser_pose` |
| 出力 | `cmd_vel` | Twist2D | `dwa_compute` | mobile_sim の `robot_set_velocity0` |
| 出力 | `dwa_status` | JSON: `{status, has_path, arrived, goal_distance, cmd, local_goal, count}` | `dwa_get_status` | navigation_manager など |

`status` は `idle`(パス無し)、`following`、`rotating`(ゴールで向きを合わせている)、`arrived`、`no_trajectory`(進める軌道が無く止まっている)、`stuck`、
`waiting_for_pose`・`waiting_for_odom`・`waiting_for_scan`(入力が無いか古いので止まっている)のどれか。
画面用のくわしい状態(選んだ軌道、候補、障害物など)は `dwa_get_state` で読む。

トピックの名前は app.conf の `subscribes`・`publishes` で変えられる(navigation_manager の下では `nav/odom`、`cmd_vel_dwa` など。下の「navigation_manager の下で動かす」)。

### 自己位置の元(`pose_source`)

- `amcl`(既定): `amcl_pose` と `odom` から。`amcl_pose` がまだ無いときは止まっている
- `odom`: `odom` をそのまま map の姿勢とみなす(AMCL を動かさずに試すとき。mobile_sim の odom0 は真の初期姿勢から始まるので、誤差は走った分だけ)

## ContainerProcess(コンテナ `dwa_planner`)

| 名前 | 引数 | 戻り値 | conf でのつなぎ |
| --- | --- | --- | --- |
| `dwa_set_path` | `path`(planner_dijkstra の経路、または `{poses: [Pose2D]}`) | なし | subscribes `path` |
| `dwa_set_goal` | `goal: Pose2DStamped` | なし(今の姿勢から直線のパスを置く。画面・試験用) | |
| `dwa_set_pose` | `pose: Pose2DStamped` | なし | subscribes `amcl_pose` |
| `dwa_set_odometry` | `odom: Pose2DStamped` | なし | subscribes `odom` |
| `dwa_set_scan` | `scan: LaserScan` | なし | subscribes `scan` |
| `dwa_set_laser_pose` | `pose: PoseStamped` | なし | subscribes `lidar_mount` |
| `dwa_compute` | なし | Twist2D | publishes `cmd_vel`、main_loop_ec に bind |
| `dwa_cancel` | なし | なし(パスを捨てて止まる) | |
| `dwa_set_params` | `params`(object) | 今のパラメータ | |
| `dwa_get_status` | なし | 短い状態(上の `dwa_status`、readonly) | publishes `dwa_status`、main_loop_ec に bind |
| `dwa_get_state` | なし | 画面用: 姿勢・速度・パス・局所ゴール・障害物・候補の軌道・選んだ軌道・状態(readonly) | |

コンテナの引数: ROS dwa_local_planner の名前のパラメータ(`max_vel_x`、`acc_lim_theta`、`sim_time`、`vx_samples`、`path_distance_bias` など)、
`robot_radius`、`inflation_radius`、`cost_scaling_factor`、`local_window`(パスを刈り込み局所ゴールを選ぶ窓の一辺 [m])、
`pose_source`、`frame_id`、`sensor_timeout`、`require_scan`、`stuck_time`、`bridge`(下の「わかっている制約」)。

## マイルストーン

| | 内容 | 確かめ方 |
| --- | --- | --- |
| M0 | 雛形: Cargo workspace(`dwa_core`・`planner_dwa`)、CI(fmt・clippy・test) | CI |
| M1 | DWA の中身: 動的窓、軌道の積分、コスト、衝突、到着の判定、パスの刈り込み | 単体テスト(直線のパスを追う、障害物を避ける、ゴールで止まる・向きを合わせる、後ろ向きのパスでその場で回る、軌道が無いとき止まる) |
| M2 | juiz コンポーネントと app.conf。mobile_sim とつなぐ | `scripts/integration_test.py`: odom で直線のパスを走り切る、折れ線のパスをぶつからずに走る、cancel で止まる |
| M3 | ブラウザ画面(http://localhost:8004/) | `scripts/ui_test.js`(Playwright) |
| M4 | `.juizapp` と release.yml(5 ターゲット)。パッケージを juiz run で mobile_sim と動かして試す | release.yml の pack ジョブ |
| M5 | Docker(ghcr.io/openjuiz/planner_dwa、amd64・arm64)と compose(mobile_sim・localization_amcl・planner_dijkstra と一緒に) | docker.yml が compose で結合テスト |

リリース(タグ v0.1.0)は ysuga が GitHub の画面で Release を作る(スレッドからはタグを打てない)。

## navigation_manager の下で動かす

juiz#449 のため、ナビゲーション一式は navigation_manager(HTTP 8005)を根にし、mobile_sim・localization_amcl・planner_dijkstra・planner_dwa を
それぞれ subsystem を持たない葉としてつなぐ形にする。葉どうしのトピックは juiz 0.2.4 では葉→根→葉と渡らないので、
navigation_manager が受けて `nav/...` の名前で出し直す。cmd_vel は navigation_manager が状態に応じて通すか止める。

planner_dwa の側で要るのは conf の書き換えだけで、コードは変えない(葉の conf は navigation_manager のリポジトリに置く):

- `subsystems` を消す(葉)。IPC ブローカー `planner_dwa` は残す
- `bridge: null`
- `subscribes` を navigation_manager が出し直す名前にする(例 `path: nav/path`、`pose: nav/amcl_pose`、`odom: nav/odom`、`scan: nav/scan`、`pose: nav/lidar_mount`)
- `compute0` の `publishes` を `cmd_vel_dwa` にする(navigation_manager が `cmd_vel` に通す)
- `get_status0` は `dwa_status` に出したまま(navigation_manager が立ち往生・到着を見る)

## わかっている制約

### planner_dijkstra の経路は壁すれすれを通る

planner_dijkstra の初版はロボットの大きさを考えないので、部屋の出入口の角を 0.2 m 以下でかすめる経路を出すことがある。
planner_dwa は scan の点から `robot_radius` 以内に入る軌道を捨てるので、そこで `no_trajectory` になってその場で回り、`stuck_time` 後に `stuck` を出す。
planner_dijkstra にフットプリント(膨張)が入れば直る。それまでは目標を出入口から離して置くか、`robot_radius` を小さめにして試す。

### 兄弟の System のトピックが届かない(OpenJUIZ/juiz#449)

juiz のトピックは subsystem の木をたどって届く。どのアプリも mobile_sim を subsystem にする形(星形)では、
mobile_sim のトピック(odom・scan・lidar_mount)は届き、planner_dwa の cmd_vel も mobile_sim に届くが、
localization_amcl の `amcl_pose` と planner_dijkstra の `path` は planner_dwa に届かない
(mobile_sim のマスターシステムは最後につないだ 1 つだけで、兄弟には転送しない)。
planner_dwa が 3 つを全部 subsystem にすると輪になり、起動時の `topic_request_subscribe` が回り続けて止まる。

juiz#449 が直るまでの回避として、`bridge` を設定すると planner_dwa が別スレッドで
localization_amcl の `amcl_get_pose` と planner_dijkstra の `planner_get_path` を HTTP で読む(既定 5 Hz)。
トピックが届いていればトピックを使い、1 秒以上届かないときだけ bridge の値を使う。#449 が直れば bridge は要らない。

## この先(別の issue)

- 地図を重ねた局所コストマップ(scan の死角の障害物)
- 振動の抑制(oscillation_reset_dist)とリカバリ動作(その場で回る・下がる)
- 経路の引き直しの依頼(進めないときに planner_dijkstra に goal を出し直す)
- mobile_sim の画面に選んだ軌道を重ねて描く

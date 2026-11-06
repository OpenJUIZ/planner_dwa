// planner_dwa のブラウザ画面
//
// juiz の HTTP ブローカーの /api/container_process/call で ContainerProcess を呼ぶ。
// - コンテナ dwa_planner: get_state(5 Hz)、set_goal(直線のパス)、cancel、set_params
// - mobile_sim(subsystem としてつながっていれば): get_map と get_map_info(地図)、get_state(真の姿勢)
//   planner_dwa の HTTP 経由で ipc://mobile_sim/... を呼ぶので、mobile_sim 側の CORS の設定は要らない
"use strict";

const POLL_MS = 200;

// 画面で変えられるパラメータ: [名前, 最小, 最大, 刻み]
const PARAMS = [
  ["max_vel_x", 0.05, 1.0, 0.05],
  ["max_vel_theta", 0.2, 1.5, 0.1],
  ["sim_time", 0.5, 4.0, 0.1],
  ["path_distance_bias", 0, 64, 1],
  ["goal_distance_bias", 0, 64, 1],
  ["occdist_scale", 0, 0.2, 0.005],
];

// ---- juiz の呼び出し ----

class JuizClient {
  constructor() {
    this.ids = {}; // "dwa_get_state" などの型名 → 識別子
  }

  // ContainerProcess の一覧(subsystem の分も)から識別子を集める。同じ型が複数あれば最初のもの
  async discover() {
    const res = await fetch("/api/container_process/list?recursive=true");
    if (!res.ok) throw new Error(`container_process/list: ${res.status}`);
    for (const e of await res.json()) {
      const [type] = e.type_name.split(":");
      if (this.ids[type]) continue;
      const ns = e.namespace ? `${e.namespace}/` : "";
      this.ids[type] = `${e.broker_type_name}://${e.broker_name}/container_process/${ns}${e.name}::${e.type_name}`;
    }
    for (const k of ["get_state", "set_goal", "cancel", "set_params"]) {
      if (!this.ids[`dwa_${k}`]) throw new Error(`dwa_${k} が見つかりません`);
    }
  }

  has(type) { return Boolean(this.ids[type]); }

  async call(type, args = {}, asBlob = false) {
    const url = "/api/container_process/call?identifier=" + encodeURIComponent(this.ids[type]);
    const res = await fetch(url, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(args),
    });
    if (!res.ok) throw new Error(`${type}: ${res.status} ${await res.text()}`);
    return asBlob ? res.blob() : res.json();
  }
}

// ---- 座標変換 ----

class View {
  constructor(canvas) {
    this.canvas = canvas;
    this.cx = 0; this.cy = 0; this.size = 10; // 表示する正方形の中心と一辺 [m]
  }

  setRange(xmin, ymin, xmax, ymax) {
    this.cx = (xmin + xmax) / 2;
    this.cy = (ymin + ymax) / 2;
    this.size = Math.max(xmax - xmin, ymax - ymin, 1);
  }

  get pxPerM() { return this.canvas.width / this.size; }

  toCanvas(x, y) {
    const k = this.pxPerM;
    return [(x - this.cx) * k + this.canvas.width / 2, this.canvas.height / 2 - (y - this.cy) * k];
  }

  fromClient(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    const px = ((clientX - rect.left) / rect.width) * this.canvas.width;
    const py = ((clientY - rect.top) / rect.height) * this.canvas.height;
    const k = this.pxPerM;
    return [(px - this.canvas.width / 2) / k + this.cx, (this.canvas.height / 2 - py) / k + this.cy];
  }
}

async function loadImage(blob) {
  const url = URL.createObjectURL(blob);
  try {
    const img = new Image();
    img.src = url;
    await img.decode();
    return img;
  } finally {
    URL.revokeObjectURL(url);
  }
}

// ---- 描画 ----

function polyline(ctx, view, pts, color, width) {
  if (!pts || pts.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    const [px, py] = view.toCanvas(x, y);
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  });
  ctx.stroke();
}

function robot(ctx, view, p, radius, fill, stroke, dashed) {
  const [x, y] = view.toCanvas(p.x, p.y);
  const r = radius * view.pxPerM;
  ctx.setLineDash(dashed ? [4, 3] : []);
  ctx.beginPath();
  ctx.arc(x, y, r, 0, 2 * Math.PI);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + Math.cos(p.theta) * r * 1.6, y - Math.sin(p.theta) * r * 1.6);
  ctx.stroke();
  ctx.setLineDash([]);
}

function arrow(ctx, view, x0, y0, theta, color) {
  const [x, y] = view.toCanvas(x0, y0);
  ctx.fillStyle = color;
  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(x, y, 5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + Math.cos(theta) * 22, y - Math.sin(theta) * 22);
  ctx.stroke();
}

function draw(view, map, st, truth, pick) {
  const ctx = view.canvas.getContext("2d");
  ctx.fillStyle = "#cdcdcd";
  ctx.fillRect(0, 0, view.canvas.width, view.canvas.height);
  if (map.image) {
    const { resolution: r, origin: [ox, oy], width: w, height: h } = map.info;
    const [x0, y0] = view.toCanvas(ox, oy + h * r);
    const k = view.pxPerM * r;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(map.image, x0, y0, w * k, h * k);
  }
  if (!st) return;
  const radius = st.params ? st.params.robot_radius : 0.2;
  polyline(ctx, view, st.path, "#2d6ad6", 3);
  for (const c of st.candidates || []) {
    polyline(ctx, view, c.points, c.valid ? "rgba(90,90,90,0.45)" : "rgba(230,90,90,0.35)", 1);
  }
  if (st.best) polyline(ctx, view, st.best.points, "#12a150", 4);
  ctx.fillStyle = "#d40000";
  for (const [x, y] of st.obstacles || []) {
    const [px, py] = view.toCanvas(x, y);
    ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
  }
  if (st.local_goal) arrow(ctx, view, st.local_goal.x, st.local_goal.y, st.local_goal.theta, "#d4006a");
  if (st.path_goal) arrow(ctx, view, st.path_goal.x, st.path_goal.y, st.path_goal.theta, "#2d6ad6");
  if (truth) robot(ctx, view, truth, radius, null, "#1a8a3a", false);
  if (st.pose) robot(ctx, view, st.pose, radius, "rgba(224,96,0,0.35)", "#e06000", true);
  if (pick && pick.from) {
    const [x, y] = pick.from;
    arrow(ctx, view, x, y, pick.theta || 0, "#d4006a");
  }
}

// ---- 表示 ----

const STATUS_TEXT = {
  idle: "パス無し(止まっている)",
  following: "パスを追っている",
  rotating: "着いた: 向きを合わせている",
  arrived: "着いた",
  no_trajectory: "使える軌道が無い(止まっている)",
  stuck: "立ち往生(進めない)",
  waiting_for_pose: "自己位置(amcl_pose)を待っている",
  waiting_for_odom: "odom を待っている",
  waiting_for_scan: "scan を待っている",
};

const fmtPose = (p) => (p ? `x ${p.x.toFixed(2)}, y ${p.y.toFixed(2)}, θ ${p.theta.toFixed(2)}` : "-");
const fmtVel = (v) => (v ? `vx ${v.vx.toFixed(2)} m/s, ω ${v.vth.toFixed(2)} rad/s` : "-");

function setText(id, text, cls) {
  const e = document.getElementById(id);
  e.textContent = text;
  e.className = cls || "";
}

function showState(st, truth) {
  const bad = ["no_trajectory", "stuck", "waiting_for_pose", "waiting_for_odom", "waiting_for_scan"];
  setText("s_status", STATUS_TEXT[st.status] || st.status,
    bad.includes(st.status) ? "bad" : st.status === "arrived" ? "good" : "");
  setText("s_pose", `${fmtPose(st.pose)}(${st.pose_source})`);
  setText("s_vel", fmtVel(st.velocity));
  setText("s_cmd", fmtVel(st.cmd));
  setText("s_goal", st.goal_distance == null ? "-" : `${st.goal_distance.toFixed(2)} m`);
  setText("s_path", st.path.length ? `${st.path.length} 点${st.path_message ? `(${st.path_message})` : ""}` : (st.path_message || "無し"));
  setText("s_obstacles", String(st.obstacles.length));
  const valid = st.candidates.filter((c) => c.valid).length;
  setText("s_cands", st.candidates.length ? `${valid} / ${st.candidates.length} が使える` : "-");
  setText("s_truth", fmtPose(truth));
  if (st.bridge) {
    const errs = [st.bridge.amcl_error && "AMCL に届かない", st.bridge.planner_error && "planner_dijkstra に届かない"].filter(Boolean);
    setText("s_bridge", errs.length ? errs.join("、") : "読めている", errs.length ? "bad" : "good");
  } else {
    setText("s_bridge", "使っていない");
  }
  setText("s_time", `${st.compute_ms.toFixed(1)} ms`);
}

function buildParams(client, params, message) {
  const box = document.getElementById("params");
  box.textContent = "";
  for (const [name, min, max, step] of PARAMS) {
    const row = document.createElement("label");
    row.className = "param";
    const title = document.createElement("span");
    title.textContent = name;
    const input = document.createElement("input");
    input.type = "range";
    input.min = min; input.max = max; input.step = step;
    input.value = params[name];
    input.id = `p_${name}`;
    const out = document.createElement("output");
    out.textContent = String(params[name]);
    input.addEventListener("input", () => { out.textContent = input.value; });
    input.addEventListener("change", async () => {
      const patch = { [name]: Number(input.value) };
      // 並進の上限は max_vel_x に合わせる
      if (name === "max_vel_x") patch.max_vel_trans = Number(input.value);
      try {
        await client.call("dwa_set_params", { params: patch });
        message(`${name} を ${input.value} にしました`);
      } catch (e) {
        message(String(e));
      }
    });
    row.append(title, input, out);
    box.append(row);
  }
}

// ---- 本体 ----

async function main() {
  const client = new JuizClient();
  const canvas = document.getElementById("map");
  const view = new View(canvas);
  const map = { image: null, info: null };
  const conn = document.getElementById("connection");
  const simConn = document.getElementById("sim_connection");
  const message = (t) => { document.getElementById("message").textContent = t; };
  let st = null;
  let truth = null;
  let pick = null; // 目標を置いている途中 {from: [x, y], theta}

  try {
    await client.discover();
    conn.textContent = "planner_dwa に接続";
    conn.className = "status ok";
  } catch (e) {
    conn.textContent = `接続できません: ${e.message}`;
    conn.className = "status error";
    return;
  }
  if (client.has("mobile_sim_get_map") && client.has("mobile_sim_get_map_info")) {
    try {
      map.info = await client.call("mobile_sim_get_map_info");
      map.image = await loadImage(await client.call("mobile_sim_get_map", {}, true));
      const { resolution: r, origin: [ox, oy], width: w, height: h } = map.info;
      view.setRange(ox, oy, ox + w * r, oy + h * r);
      simConn.textContent = "mobile_sim の地図を表示";
      simConn.className = "status ok";
    } catch (e) {
      simConn.textContent = `mobile_sim の地図を読めません: ${e.message}`;
      simConn.className = "status warn";
    }
  } else {
    simConn.textContent = "mobile_sim が見えません(地図なしで表示)";
    simConn.className = "status warn";
  }

  // ---- 操作 ----
  const goalButton = document.getElementById("set_goal");
  goalButton.addEventListener("click", () => {
    goalButton.classList.toggle("armed");
    canvas.classList.toggle("picking", goalButton.classList.contains("armed"));
  });
  document.getElementById("cancel").addEventListener("click", async () => {
    try {
      await client.call("dwa_cancel");
      message("止めました");
    } catch (e) {
      message(String(e));
    }
  });
  canvas.addEventListener("pointerdown", (ev) => {
    if (!goalButton.classList.contains("armed")) return;
    pick = { from: view.fromClient(ev.clientX, ev.clientY), theta: 0 };
    canvas.setPointerCapture(ev.pointerId);
  });
  canvas.addEventListener("pointermove", (ev) => {
    if (!pick) return;
    const [x, y] = view.fromClient(ev.clientX, ev.clientY);
    if (Math.hypot(x - pick.from[0], y - pick.from[1]) > 0.05) {
      pick.theta = Math.atan2(y - pick.from[1], x - pick.from[0]);
    }
  });
  canvas.addEventListener("pointerup", async () => {
    if (!pick) return;
    const [x, y] = pick.from;
    const theta = pick.theta;
    pick = null;
    goalButton.classList.remove("armed");
    canvas.classList.remove("picking");
    const goal = { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" }, pose: { position: { x, y }, theta } };
    try {
      await client.call("dwa_set_goal", { goal });
      message(`目標 ${fmtPose({ x, y, theta })} へ直線のパスを置きました`);
    } catch (e) {
      message(String(e));
    }
  });

  // ---- 定期の読み出し ----
  let paramsBuilt = false;
  const tick = async () => {
    try {
      st = await client.call("dwa_get_state");
      if (!paramsBuilt && st.params) {
        buildParams(client, st.params, message);
        paramsBuilt = true;
      }
      if (client.has("mobile_sim_get_state")) {
        try {
          const s = await client.call("mobile_sim_get_state");
          truth = s.robots && s.robots[0] ? s.robots[0].pose : null;
        } catch (_) {
          truth = null;
        }
      }
      showState(st, truth);
      conn.textContent = "planner_dwa に接続";
      conn.className = "status ok";
    } catch (e) {
      conn.textContent = `読めません: ${e.message}`;
      conn.className = "status error";
    }
    draw(view, map, st, truth, pick);
    setTimeout(tick, POLL_MS);
  };
  tick();
}

main();

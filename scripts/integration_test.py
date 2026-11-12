"""mobile_sim と planner_dwa を IPC でつないで走らせ、DWA の追従を確かめる(M2 の結合テスト)

planner_dwa は pose_source: odom(AMCL 無し)、bridge 無しで動かす。パス(トピック path)を与えて mobile_sim のロボットを走らせ、
着くこと・ぶつからないこと(mobile_sim の bumper)・向きを合わせること・cancel で止まること・ok: false のパスで止まることを確かめる。

使い方:
  scripts/dev_plugin.sh
  python3 scripts/integration_test.py --mobile-sim <mobile_sim のリポジトリ> [--juiz <juiz のパス>] [--ui SCREENSHOT]
    mobile_sim は app/plugins/<target>/ にプラグインを置いておく(mobile_sim の scripts/dev_plugin.sh)
  python3 scripts/integration_test.py --mobile-sim-juizapp mobile_sim-0.3.4.juizapp --juizapp dist/planner_dwa-0.1.1.juizapp
    パッケージで動かす(ポートは mobile_sim 8000、planner_dwa 8004)
  python3 scripts/integration_test.py --sim-url http://localhost:8000 --dwa-url http://localhost:8004
    もう動いている mobile_sim と planner_dwa を試す(docker compose など。pose_source は odom にしておく)

IPC の名前 mobile_sim と planner_dwa を使うので、同じ機械でそれらを動かしていないこと。
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# AMCL・planner_dijkstra 無しで試すので、自己位置は odom、bridge は使わない
CONF_OVERRIDES = [
    ('"pose_source": "amcl"', '"pose_source": "odom"'),
    ('"amcl": "http://localhost:8002"', '"amcl": null'),
    ('"planner": "http://localhost:8003"', '"planner": null'),
]


def test_conf(text, port=None):
    for a, b in CONF_OVERRIDES:
        if a not in text:
            sys.exit(f"app.conf has no {a}")
        text = text.replace(a, b)
    if port is not None:
        text = text.replace('"port": 8004', f'"port": {port}')
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--juiz", default="juiz")
    ap.add_argument("--mobile-sim", type=Path, help="mobile_sim のリポジトリ(app/app.conf を使う)")
    ap.add_argument("--mobile-sim-juizapp", type=Path, help="mobile_sim の .juizapp(juiz run で動かす)")
    ap.add_argument("--juizapp", type=Path, help="planner_dwa の .juizapp(展開した conf を odom 用に書き換えて juiz -f で動かす。ポートは 8004)")
    ap.add_argument("--sim-port", type=int, default=18000)
    ap.add_argument("--port", type=int, default=18004)
    ap.add_argument("--sim-url", help="もう動いている mobile_sim(--dwa-url と一緒に使う)")
    ap.add_argument("--dwa-url", help="もう動いている planner_dwa(pose_source: odom で)")
    ap.add_argument("--ui", type=Path, metavar="SCREENSHOT",
                    help="続けてブラウザ画面のテスト (scripts/ui_test.js、Playwright が要る) を走らせ、画面を保存する")
    args = ap.parse_args()
    if args.sim_url or args.dwa_url:
        if not (args.sim_url and args.dwa_url):
            sys.exit("give both --sim-url and --dwa-url")
        sim, base = args.sim_url.rstrip("/"), args.dwa_url.rstrip("/")
        for url in (sim, base):
            wait_ready(url, None)
        run(sim, base)
        print("OK")
        return
    if not args.mobile_sim and not args.mobile_sim_juizapp:
        sys.exit("give --mobile-sim or --mobile-sim-juizapp (or --sim-url and --dwa-url)")

    procs = []
    tmps = []
    try:
        if args.mobile_sim_juizapp:
            sim = "http://localhost:8000"
            work = tempfile.mkdtemp()
            env = dict(os.environ, JUIZ_CACHE_DIR=os.path.join(work, "cache"))
            procs.append(start([args.juiz, "run", str(args.mobile_sim_juizapp.resolve())], work, env))
        else:
            sim = f"http://localhost:{args.sim_port}"
            app = args.mobile_sim.resolve() / "app"
            conf = (app / "app.conf").read_text().replace('"port": 8000', f'"port": {args.sim_port}')
            tmp = app / ".dwa_integration_test.conf"
            tmp.write_text(conf)
            tmps.append(tmp)
            procs.append(start([args.juiz, "-f", tmp.name, "-d", "--ipc", "mobile_sim"], app))
        wait_ready(sim, procs[-1])

        if args.juizapp:
            # パッケージを展開し、conf を odom 用に書き換えて動かす(juiz run は conf を書き換えられないので)。
            # プラグイン・画面・マニフェストはパッケージのものをそのまま使う
            base = "http://localhost:8004"
            work = Path(tempfile.mkdtemp())
            tmps.append(work)
            with zipfile.ZipFile(args.juizapp) as z:
                z.extractall(work)
            manifest = (work / "juizapp.yaml").read_text()
            check("kind: app" in manifest and "mobile_sim" in manifest, "the package manifest depends on mobile_sim")
            (work / ".test.conf").write_text(test_conf((work / "app.conf").read_text()))
            env = dict(os.environ, JUIZ_APP_DIR=str(work))
            procs.append(start([args.juiz, "-f", ".test.conf", "-d"], work, env))
        else:
            base = f"http://localhost:{args.port}"
            tmp = ROOT / "app" / ".integration_test.conf"
            tmp.write_text(test_conf((ROOT / "app" / "app.conf").read_text(), args.port))
            tmps.append(tmp)
            procs.append(start([args.juiz, "-f", tmp.name, "-d"], ROOT / "app"))
        wait_ready(base, procs[-1])

        run(sim, base)
        if args.ui:
            print("# ui_test.js")
            subprocess.run(["node", str(ROOT / "scripts" / "ui_test.js"), f"{base}/", str(args.ui.resolve())], check=True)
        print("OK")
    except SystemExit:
        for p in procs:
            dump_log(p)
        raise
    finally:
        for p in reversed(procs):
            p.terminate()
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        for t in tmps:
            if t.is_dir():
                shutil.rmtree(t, ignore_errors=True)
            else:
                t.unlink(missing_ok=True)


def start(cmd, cwd, env=None):
    log = tempfile.TemporaryFile()
    p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
    p.log = log
    p.cmd = cmd
    return p


def dump_log(p):
    p.log.seek(0)
    text = p.log.read().decode(errors="replace")
    print(f"--- log of {' '.join(map(str, p.cmd))} ---\n{text[-4000:]}", file=sys.stderr)


def wait_ready(base, proc):
    for _ in range(150):
        if proc is not None and proc.poll() is not None:
            proc.log.seek(0)
            sys.exit(f"juiz exited ({base}):\n" + proc.log.read().decode(errors="replace")[-4000:])
        try:
            with urllib.request.urlopen(f"{base}/api/container/list", timeout=1) as r:
                if json.load(r):
                    return
        except OSError:
            pass
        time.sleep(0.2)
    sys.exit(f"juiz did not become ready ({base})")


def ident(base, name, type_name, container):
    return f"{base}/container_process/{name}::{type_name}:{container}"


def call(base, name, type_name, container, args=None, raw=False):
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident(base, name, type_name, container), safe="")
    req = urllib.request.Request(
        url, data=json.dumps(args or {}).encode(), method="PATCH", headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        body = r.read()
        return body if raw else json.loads(body)


def call_multipart(base, name, type_name, container, parts):
    """画像の引数を持つ Process を multipart で呼ぶ(画像は image/png、ほかは application/json)"""
    boundary = uuid.uuid4().hex
    body = b""
    for key, (ctype, data) in parts.items():
        body += f"--{boundary}\r\nContent-Disposition: form-data; name=\"{key}\"\r\nContent-Type: {ctype}\r\n\r\n".encode()
        body += data + b"\r\n"
    body += f"--{boundary}--\r\n".encode()
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident(base, name, type_name, container), safe="")
    req = urllib.request.Request(url, data=body, method="PUT",
                                 headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())


def sim_call(sim, name, kind, args=None, raw=False):
    return call(sim, name, f"mobile_sim_{kind}", "sim0", args, raw)


def dwa_call(base, name, kind, args=None, raw=False):
    return call(base, name, f"dwa_{kind}", "dwa0", args, raw)


def push_topic(base, topic, value):
    url = f"{base}/api/topic/push?topic_name={topic}"
    req = urllib.request.Request(url, data=json.dumps({"input": value}).encode(), method="PATCH",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        r.read()


def check(cond, msg):
    if not cond:
        sys.exit("FAILED: " + msg)
    print("ok -", msg)


def pose(x, y, theta=0.0):
    return {"position": {"x": x, "y": y}, "theta": theta}


def stamped(x, y, theta=0.0, frame="map"):
    return {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame}, "pose": pose(x, y, theta)}


def xy(p):
    return p["position"]["x"], p["position"]["y"]


def wait_for(fn, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = fn()
        if v:
            return v
        time.sleep(0.1)
    return None


def path_msg(points, final_theta=0.0, ok=True, message=""):
    """折れ線を 5 cm 刻みにした経路(planner_dijkstra と同じ形)"""
    poses = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist((x0, y0), (x1, y1)) / 0.05))
        th = math.atan2(y1 - y0, x1 - x0)
        poses += [{"position": {"x": x0 + (x1 - x0) * i / n, "y": y0 + (y1 - y0) * i / n}, "theta": th} for i in range(n)]
    if points:
        poses.append({"position": {"x": points[-1][0], "y": points[-1][1]}, "theta": final_theta})
    now = time.time()
    return {"header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}, "frame_id": "map"},
            "ok": ok, "message": message, "poses": poses}


def truth(sim):
    r = sim_call(sim, "get_state0", "get_state")["robots"][0]
    return r


def drive(sim, base, timeout, label):
    """着くまで(または timeout)待つ。途中で bumper が立てば失敗"""
    t0 = time.time()
    statuses = {"start"}
    while time.time() - t0 < timeout:
        r = truth(sim)
        check_quiet(not r["bumper"], f"{label}: the robot hit something at {r['pose']}")
        st = dwa_call(base, "get_status0", "get_status")
        statuses.add(st["status"])
        # 前のパスの arrived が残っている間は待つ(新しいパスで一度ほかの状態になってから)
        if st["status"] == "arrived" and statuses - {"start", "arrived"}:
            return st, r, statuses
        time.sleep(0.1)
    sys.exit(f"FAILED: {label}: not arrived in {timeout} s (status {statuses}, robot {truth(sim)['pose']})")


def check_quiet(cond, msg):
    if not cond:
        sys.exit("FAILED: " + msg)


def run(sim, base):
    # 0) パスが無ければ止まっている。odom が届いている(mobile_sim は起動したまま、既定の初期位置 -2, 0 にいること)
    st = wait_for(lambda: (lambda s: s if s["pose"] else None)(dwa_call(base, "get_state0", "get_state")), timeout=10)
    check(st is not None and math.dist((st["pose"]["x"], st["pose"]["y"]), (-2.0, 0.0)) < 0.05,
          "the odom of mobile_sim reaches planner_dwa (topic odom)")
    st = wait_for(lambda: (lambda s: s if s["obstacles"] else None)(dwa_call(base, "get_state0", "get_state")), timeout=10)
    check(st is not None and len(st["obstacles"]) > 50, "the scan reaches planner_dwa (topic scan)")
    check(st["status"] == "idle" and st["cmd"]["vx"] == 0.0, "idle without a path")

    # 1) まっすぐ: 左の部屋から仕切り壁の出入口(x = 1.5、y = -1〜1)の真ん中を通って右の部屋へ。最後は北を向く
    push_topic(base, "path", path_msg([(-2.0, 0.0), (3.0, 0.0)], final_theta=math.pi / 2))
    st, r, statuses = drive(sim, base, 60, "straight")
    # planner_dwa が使う姿勢(odom)では許容誤差に入り、真の姿勢は odom の誤差(数 % の滑り)の分だけずれる
    q = dwa_call(base, "get_state0", "get_state")["pose"]
    check(math.dist((q["x"], q["y"]), (3.0, 0.0)) <= 0.15 and abs(math.remainder(q["theta"] - math.pi / 2, 2 * math.pi)) <= 0.15,
          f"straight: arrived at (3, 0, pi/2) in odom ({q['x']:.2f}, {q['y']:.2f}, {q['theta']:.2f})")
    p = r["pose"]
    check(math.dist((p["x"], p["y"]), (3.0, 0.0)) < 0.6, f"straight: the true pose is near (3, 0) ({p['x']:.2f}, {p['y']:.2f})")
    check(abs(math.remainder(p["theta"] - math.pi / 2, 2 * math.pi)) < 0.4, f"straight: faces north ({p['theta']:.2f})")
    check("following" in statuses, f"straight: followed the path ({sorted(statuses)})")
    check(st["cmd"] == {"vx": 0.0, "vy": 0.0, "vth": 0.0}, "straight: stops at the goal")
    time.sleep(0.5)
    v = truth(sim)["velocity"]
    check(abs(v["vx"]) < 1e-3 and abs(v["omega"]) < 1e-3, f"straight: mobile_sim stopped (topic cmd_vel) {v}")

    # 2) 折れ線: 北を向いたまま、後ろ(西)へ戻るパス。その場で回ってから、出入口を通って左上へ
    push_topic(base, "path", path_msg([(3.0, 0.0), (0.0, 0.0), (-0.5, 2.0)], final_theta=0.0))
    st, r, statuses = drive(sim, base, 60, "polyline")
    q = dwa_call(base, "get_state0", "get_state")["pose"]
    check(math.dist((q["x"], q["y"]), (-0.5, 2.0)) <= 0.15, f"polyline: arrived at (-0.5, 2) in odom ({q['x']:.2f}, {q['y']:.2f})")
    check("rotating" in statuses or "following" in statuses, f"polyline: statuses {sorted(statuses)}")

    # 3) cancel: 走っている途中で止める
    push_topic(base, "path", path_msg([(-0.5, 2.0), (-0.5, -2.0)], final_theta=0.0))
    ok = wait_for(lambda: truth(sim)["velocity"]["vx"] > 0.2 or None, timeout=10)
    check(ok is not None, "cancel: the robot starts moving")
    dwa_call(base, "cancel0", "cancel")
    time.sleep(1.5)
    v = truth(sim)["velocity"]
    st = dwa_call(base, "get_status0", "get_status")
    check(st["status"] == "idle" and abs(v["vx"]) < 1e-3, f"cancel: stops ({st['status']}, {v})")

    # 4) planner_dijkstra が経路を求められなかったとき(ok: false)は止まる
    push_topic(base, "path", path_msg([(-0.5, 1.0), (-0.5, -2.0)]))
    wait_for(lambda: truth(sim)["velocity"]["vx"] > 0.1 or None, timeout=10)
    push_topic(base, "path", {"ok": False, "message": "the goal is on an obstacle", "poses": []})
    time.sleep(1.5)
    st = dwa_call(base, "get_state0", "get_state")
    check(st["status"] == "idle" and st["path_message"] == "the goal is on an obstacle", "a failed path (ok: false) stops the robot")
    check(abs(truth(sim)["velocity"]["vx"]) < 1e-3, "a failed path: mobile_sim stopped")

    # 5) dwa_set_goal(直線のパス)とパラメータの変更
    params = dwa_call(base, "set_params0", "set_params", {"params": {"max_vel_x": 0.25, "max_vel_trans": 0.25}})
    check(params["max_vel_x"] == 0.25 and params["vx_samples"] == 6, "set_params changes some parameters")
    here = dwa_call(base, "get_state0", "get_state")["pose"]
    dwa_call(base, "set_goal0", "set_goal", {"goal": stamped(here["x"], here["y"] - 1.0, -math.pi / 2)})
    top = 0.0
    t0 = time.time()
    while time.time() - t0 < 40:
        top = max(top, truth(sim)["velocity"]["vx"])
        if dwa_call(base, "get_status0", "get_status")["status"] == "arrived":
            break
        time.sleep(0.1)
    check(dwa_call(base, "get_status0", "get_status")["status"] == "arrived", "set_goal: arrived")
    check(0.1 < top <= 0.25 + 1e-6, f"set_goal: the speed follows max_vel_x 0.25 (max {top:.3f})")
    dwa_call(base, "set_params0", "set_params", {"params": {"max_vel_x": 0.5, "max_vel_trans": 0.5}})
    st = dwa_call(base, "get_state0", "get_state")
    check(st["compute_ms"] < 50, f"one cycle takes {st['compute_ms']:.1f} ms")


if __name__ == "__main__":
    main()

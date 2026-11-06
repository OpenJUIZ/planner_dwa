#!/bin/sh
# mobile_sim が起動するのを待ってから planner_dwa を動かす。
# app.conf の IPC の subsystem を MOBILE_SIM_URL の HTTP に置き換え、pose_source を DWA_POSE_SOURCE にした conf を作り、juiz -f で動かす
# (mobile_sim のイメージは juiz run で動いていないので、juiz run の depends の確かめは使えない)。
# bridge の URL はプラグインが環境変数 DWA_BRIDGE_AMCL_URL・DWA_BRIDGE_PLANNER_URL から読む
set -eu
url="${MOBILE_SIM_URL:?MOBILE_SIM_URL を http://<host>:<port> の形で渡してください}"
conf=/opt/planner_dwa/.docker.conf
python3 - "$url" "$conf" "${DWA_POSE_SOURCE:-amcl}" <<'PY'
import sys, time, urllib.parse, urllib.request
url, conf, pose_source = sys.argv[1].rstrip("/"), sys.argv[2], sys.argv[3]
host = urllib.parse.urlparse(url).netloc
src = open("/opt/planner_dwa/app.conf").read()
ipc_subsystem = '''  - "package": "mobile_sim"
    "type_name": "ipc"
    "name": "mobile_sim"'''
ipc_broker = '''"brokers":
  - "type_name": "ipc"
    "name": "planner_dwa"
'''
pose = '"pose_source": "amcl"'
if not host or ipc_subsystem not in src or ipc_broker not in src or pose not in src:
    sys.exit("cannot make the conf: check MOBILE_SIM_URL and the subsystems/brokers/pose_source of app.conf")
if pose_source not in ("amcl", "odom"):
    sys.exit(f"DWA_POSE_SOURCE must be amcl or odom, got {pose_source}")
src = (src.replace(ipc_subsystem, f'  - "type_name": "http"\n    "name": "{host}"')
          .replace(ipc_broker, "")
          .replace(pose, f'"pose_source": "{pose_source}"'))
open(conf, "w").write(src)
for _ in range(300):
    try:
        urllib.request.urlopen(url + "/api/container/list", timeout=2).read()
        break
    except Exception:
        time.sleep(0.2)
else:
    sys.exit(f"mobile_sim did not answer at {url}")
PY
exec juiz -f "$conf" -d "$@"

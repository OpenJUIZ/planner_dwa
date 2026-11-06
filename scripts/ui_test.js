// ブラウザ画面の確認(M3)。mobile_sim と planner_dwa(pose_source: odom)を起動しておき、Playwright で開いて操作する
// 使い方: node scripts/ui_test.js [http://localhost:8004/] [スクリーンショットの保存先]
const { chromium } = require("playwright");

const url = process.argv[2] || "http://localhost:8004/";
const shot = process.argv[3] || "ui_test.png";

function check(cond, msg) {
  if (!cond) { console.error("FAILED: " + msg); process.exit(1); }
  console.log("ok -", msg);
}

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1100, height: 900 } });
  const errors = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url);
  await page.waitForSelector("#connection.ok", { timeout: 10000 });
  check(true, "connected to juiz");
  await page.waitForSelector("#sim_connection.ok", { timeout: 10000 });
  check(true, "shows the map of mobile_sim through the subsystem");
  await page.waitForFunction(() => /^x /.test(document.getElementById("s_pose").textContent));
  await page.waitForFunction(() => /^x /.test(document.getElementById("s_truth").textContent));
  check(Number(await page.textContent("#s_obstacles")) > 50, "shows the scan points");
  await page.waitForSelector("#p_max_vel_x");
  check(true, "builds the parameter sliders");

  // 世界座標 → ページ上の位置(地図は 10 m 四方、原点 -5, -5 をキャンバスいっぱいに描いている)
  const box = await page.locator("#map").boundingBox();
  const toPage = (x, y) => [box.x + ((x + 5) / 10) * box.width, box.y + ((5 - y) / 10) * box.height];
  const pose = await page.textContent("#s_pose");
  const m = /x (-?[\d.]+), y (-?[\d.]+)/.exec(pose);
  const [px, py] = [Number(m[1]), Number(m[2])];
  // 今の位置から 0.8 m 南(北の壁から離れる向き)に、東向きの目標を置く
  const gy = py > -2 ? py - 0.8 : py + 0.8;
  await page.click("#set_goal");
  check(await page.$eval("#set_goal", (b) => b.classList.contains("armed")), "the button arms the goal picker");
  const [sx, sy] = toPage(px, gy);
  await page.mouse.move(sx, sy);
  await page.mouse.down();
  await page.mouse.move(sx + 30, sy, { steps: 5 });
  await page.mouse.up();
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("直線のパス"));
  await page.waitForFunction(() => document.getElementById("s_status").textContent === "パスを追っている", null, { timeout: 5000 });
  check(true, "a straight path is set and followed");
  await page.waitForTimeout(500);
  // 選んだ軌道(緑)とパス(青)が描かれている
  const px_ = await page.evaluate(() => {
    const c = document.getElementById("map");
    const d = c.getContext("2d").getImageData(0, 0, c.width, c.height).data;
    let green = 0, blue = 0;
    for (let i = 0; i < d.length; i += 4) {
      if (d[i] < 60 && d[i + 1] > 140 && d[i + 2] < 110) green++;
      if (d[i] < 80 && d[i + 1] > 80 && d[i + 1] < 130 && d[i + 2] > 190) blue++;
    }
    return { green, blue };
  });
  check(px_.green > 30 && px_.blue > 30, `canvas shows the chosen trajectory and the path (${JSON.stringify(px_)})`);
  await page.screenshot({ path: shot });
  await page.waitForFunction(() => document.getElementById("s_status").textContent === "着いた", null, { timeout: 30000 });
  check(true, "arrives");

  // パラメータを変える
  await page.$eval("#p_max_vel_x", (e) => { e.value = "0.3"; e.dispatchEvent(new Event("input")); e.dispatchEvent(new Event("change")); });
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("max_vel_x を 0.3"));
  check(true, "a slider changes max_vel_x");
  await page.$eval("#p_max_vel_x", (e) => { e.value = "0.5"; e.dispatchEvent(new Event("change")); });

  // 停止ボタン
  await page.click("#set_goal");
  const [tx, ty] = toPage(px, py);
  await page.mouse.move(tx, ty);
  await page.mouse.down();
  await page.mouse.up();
  await page.waitForFunction(() => document.getElementById("s_status").textContent !== "着いた", null, { timeout: 5000 });
  await page.click("#cancel");
  await page.waitForFunction(() => document.getElementById("s_status").textContent.startsWith("パス無し"), null, { timeout: 5000 });
  check(true, "the stop button drops the path");

  check(errors.length === 0, `no page errors (${errors.join("; ")})`);
  await browser.close();
  console.log("OK");
})().catch((e) => { console.error(e); process.exit(1); });

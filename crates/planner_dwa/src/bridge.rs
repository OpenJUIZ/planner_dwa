//! juiz#449 の回避: localization_amcl と planner_dijkstra の Process を HTTP で読む
//!
//! juiz のトピックは subsystem の木をたどって届くので、どのアプリも mobile_sim だけを subsystem にした形(星形)では、
//! 兄弟の localization_amcl の `amcl_pose` と planner_dijkstra の `path` が planner_dwa に届かない。
//! `bridge` を設定すると、別スレッドがそれらの HTTP ブローカーの `/api/container_process/call` で
//! `amcl_get_pose` と `planner_get_path` を周期的に呼び、受け箱に置く。コンテナはトピックが来ていないときだけそれを使う。
//! navigation_manager の下(木の葉)で動かすときは要らない。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use juiz_sdk::serde_json::{self, json, Value};

/// 読む先の 1 つ
#[derive(Clone, Debug, PartialEq)]
pub struct Source {
    /// juiz の HTTP ブローカー(例 http://localhost:8002)
    pub url: String,
    /// ContainerProcess の識別子のうち `container_process/` より後(例 get_pose0::amcl_get_pose:amcl0)
    pub process: String,
}

/// bridge の設定
#[derive(Clone, Debug, PartialEq)]
pub struct BridgeConfig {
    pub amcl: Option<Source>,
    pub planner: Option<Source>,
    /// 1 秒に何回読むか
    pub rate: f64,
}

pub const DEFAULT_AMCL_PROCESS: &str = "get_pose0::amcl_get_pose:amcl0";
pub const DEFAULT_PLANNER_PROCESS: &str = "get_path0::planner_get_path:planner0";
/// 環境変数で URL を上書きできる(Docker で使う)
pub const AMCL_URL_ENV: &str = "DWA_BRIDGE_AMCL_URL";
pub const PLANNER_URL_ENV: &str = "DWA_BRIDGE_PLANNER_URL";

impl BridgeConfig {
    /// conf の `bridge: {amcl: <URL>, planner: <URL>, amcl_process, planner_process, rate}` から。
    /// null・false・無しなら None。URL は環境変数 DWA_BRIDGE_AMCL_URL・DWA_BRIDGE_PLANNER_URL で上書きでき、空にすると使わない
    pub fn from_value(v: Option<&Value>) -> Result<Option<Self>, String> {
        let obj = match v {
            None | Some(Value::Null) | Some(Value::Bool(false)) => return Ok(None),
            Some(Value::Object(o)) => o,
            Some(other) => return Err(format!("bridge must be an object or null, got {other}")),
        };
        for k in obj.keys() {
            if !["amcl", "planner", "amcl_process", "planner_process", "rate"].contains(&k.as_str())
            {
                return Err(format!("unknown key in bridge: {k}"));
            }
        }
        let text = |k: &str| -> Result<Option<String>, String> {
            match obj.get(k) {
                // juiz は conf の null を {} にして渡す
                None | Some(Value::Null) => Ok(None),
                Some(Value::Object(o)) if o.is_empty() => Ok(None),
                Some(Value::String(s)) => Ok(Some(s.clone())),
                Some(other) => Err(format!("bridge.{k} must be a string, got {other}")),
            }
        };
        let source = |url_key: &str,
                      env: &str,
                      proc_key: &str,
                      default: &str|
         -> Result<Option<Source>, String> {
            let url = match std::env::var(env) {
                Ok(u) => Some(u),
                Err(_) => text(url_key)?,
            };
            let url = match url {
                Some(u) if !u.trim().is_empty() => u.trim().trim_end_matches('/').to_owned(),
                _ => return Ok(None),
            };
            if !url.starts_with("http://") {
                return Err(format!(
                    "bridge.{url_key} must start with http://, got {url}"
                ));
            }
            Ok(Some(Source {
                url,
                process: text(proc_key)?.unwrap_or_else(|| default.to_owned()),
            }))
        };
        let rate = match obj.get("rate") {
            None | Some(Value::Null) => 5.0,
            Some(Value::Object(o)) if o.is_empty() => 5.0,
            Some(v) => v
                .as_f64()
                .filter(|r| r.is_finite() && *r > 0.0)
                .ok_or_else(|| format!("bridge.rate must be a positive number, got {v}"))?,
        };
        let cfg = BridgeConfig {
            amcl: source("amcl", AMCL_URL_ENV, "amcl_process", DEFAULT_AMCL_PROCESS)?,
            planner: source(
                "planner",
                PLANNER_URL_ENV,
                "planner_process",
                DEFAULT_PLANNER_PROCESS,
            )?,
            rate,
        };
        if cfg.amcl.is_none() && cfg.planner.is_none() {
            return Ok(None);
        }
        Ok(Some(cfg))
    }

    pub fn to_json(&self) -> Value {
        let s = |s: &Option<Source>| {
            s.as_ref()
                .map(|s| json!({"url": s.url, "process": s.process}))
        };
        json!({"amcl": s(&self.amcl), "planner": s(&self.planner), "rate": self.rate})
    }
}

/// 読んだ値。`seq` は読むたびに増える
#[derive(Clone, Debug, Default)]
pub struct Inbox {
    pub pose: Option<(u64, Value)>,
    pub path: Option<(u64, Value)>,
    pub amcl_error: Option<String>,
    pub planner_error: Option<String>,
    pub last_read: Option<Instant>,
}

/// 読むスレッド。落とすと止まる
pub struct Bridge {
    pub config: BridgeConfig,
    inbox: Arc<Mutex<Inbox>>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl Bridge {
    pub fn start(config: BridgeConfig) -> Self {
        let inbox = Arc::new(Mutex::new(Inbox::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (c, i, s) = (config.clone(), inbox.clone(), stop.clone());
        let handle = std::thread::Builder::new()
            .name("planner_dwa_bridge".to_owned())
            .spawn(move || run(c, i, s))
            .ok();
        Self {
            config,
            inbox,
            stop,
            handle,
        }
    }

    pub fn inbox(&self) -> Inbox {
        self.inbox.lock().map(|i| i.clone()).unwrap_or_default()
    }
}

impl Drop for Bridge {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn run(config: BridgeConfig, inbox: Arc<Mutex<Inbox>>, stop: Arc<AtomicBool>) {
    let agent = ureq::AgentBuilder::new()
        .timeout(Duration::from_millis(800))
        .build();
    let period = Duration::from_secs_f64(1.0 / config.rate);
    let mut seq = 0u64;
    while !stop.load(Ordering::SeqCst) {
        let t0 = Instant::now();
        seq += 1;
        if let Some(src) = &config.amcl {
            let r = call(&agent, src);
            if let Ok(mut i) = inbox.lock() {
                match r {
                    Ok(v) => {
                        i.pose = Some((seq, v));
                        i.amcl_error = None;
                    }
                    Err(e) => i.amcl_error = Some(e),
                }
            }
        }
        if let Some(src) = &config.planner {
            let r = call(&agent, src);
            if let Ok(mut i) = inbox.lock() {
                match r {
                    Ok(v) => {
                        i.path = Some((seq, v));
                        i.planner_error = None;
                    }
                    Err(e) => i.planner_error = Some(e),
                }
            }
        }
        if let Ok(mut i) = inbox.lock() {
            i.last_read = Some(Instant::now());
        }
        // 止める合図を見ながら待つ
        while t0.elapsed() < period && !stop.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
}

/// ContainerProcess を HTTP で呼ぶ(引数なし)
fn call(agent: &ureq::Agent, src: &Source) -> Result<Value, String> {
    let url = format!(
        "{}/api/container_process/call?identifier={}",
        src.url,
        percent_encode(&identifier(src))
    );
    let body = agent
        .request("PATCH", &url)
        .set("Content-Type", "application/json")
        .send_string("{}")
        .map_err(|e| e.to_string())?
        .into_string()
        .map_err(|e| e.to_string())?;
    serde_json::from_str(&body).map_err(|e| format!("{e}: {body}"))
}

/// HTTP ブローカーの ContainerProcess の識別子(http://<host:port>/container_process/<process>)
pub fn identifier(src: &Source) -> String {
    let host = src.url.trim_start_matches("http://");
    let host = host.split('/').next().unwrap_or(host);
    format!("http://{host}/container_process/{}", src.process)
}

fn percent_encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len() * 3);
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || b"-_.~".contains(&b) {
            out.push(b as char);
        } else {
            out.push_str(&format!("%{b:02X}"));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_config() {
        assert_eq!(BridgeConfig::from_value(None).unwrap(), None);
        assert_eq!(BridgeConfig::from_value(Some(&json!(false))).unwrap(), None);
        let c =
            BridgeConfig::from_value(Some(&json!({"amcl": "http://localhost:8002/", "rate": 2})))
                .unwrap()
                .unwrap();
        let a = c.amcl.clone().unwrap();
        assert_eq!(a.url, "http://localhost:8002");
        assert_eq!(a.process, DEFAULT_AMCL_PROCESS);
        assert_eq!(c.planner, None);
        assert_eq!(c.rate, 2.0);
        assert_eq!(
            identifier(&a),
            "http://localhost:8002/container_process/get_pose0::amcl_get_pose:amcl0"
        );
        assert!(BridgeConfig::from_value(Some(&json!({"amcl": "localhost:8002"}))).is_err());
        assert!(BridgeConfig::from_value(Some(&json!({"foo": 1}))).is_err());
        assert!(BridgeConfig::from_value(Some(&json!({"amcl": "http://a", "rate": 0}))).is_err());
        assert_eq!(
            BridgeConfig::from_value(Some(&json!({"amcl": ""}))).unwrap(),
            None
        );
        assert_eq!(percent_encode("a:b/c d"), "a%3Ab%2Fc%20d");
    }

    #[test]
    fn records_errors_when_nothing_answers() {
        // 使われていないはずのポート
        let b = Bridge::start(BridgeConfig {
            amcl: Some(Source {
                url: "http://127.0.0.1:9".to_owned(),
                process: DEFAULT_AMCL_PROCESS.to_owned(),
            }),
            planner: None,
            rate: 20.0,
        });
        let t0 = Instant::now();
        while b.inbox().amcl_error.is_none() && t0.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(20));
        }
        let i = b.inbox();
        assert!(i.amcl_error.is_some());
        assert!(i.pose.is_none());
        drop(b);
    }
}

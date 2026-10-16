#!/usr/bin/env python3
"""
DISCRIMINATION ENGINE + OPERATOR DISPLAY   —   runs on the laptop

Receives two independent streams and reconciles them:

    observations   UDP 47809   from the tripod sensor node (over the air)
    claims         UDP 47808   from permitted emitters     (over the network)

and serves an operator display at http://localhost:8080

This file contains the same logic as discrim_sim_v2.py — gate, one-to-one
assignment, signature verification, evidence accumulation. It is the piece
that must survive unchanged into Phase 2, which is why it runs here and not
on the sensor node.

Standard library only. No pip install, which matters on a borrowed laptop.

USAGE
    python3 engine.py
    python3 engine.py --port 8080 --site site.json
"""

import argparse
import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CLAIM_PORT = 47808
OBS_PORT   = 47809
MAGIC      = "SLICE1"
STALE_S    = 3.0

state_lock = threading.Lock()
CFG = {}
STATE = {
    "t": 0, "observations": [], "claims": [], "tracks": [],
    "matched": [], "unmatched": [], "events": [],
    "counters": {"escalations": 0, "false_alarms": 0},
    "node_seen": 0,
}
EVIDENCE = {}          # mac -> {"score": float, "escalated": bool, "since": t}


# =====================================================================
#  geometry and propagation
# =====================================================================

def enu(lat, lon, lat0, lon0):
    """Local east/north metres relative to the node. Fine at demo scales."""
    k = 111320.0
    return ((lon - lon0) * k * math.cos(math.radians(lat0)),
            (lat - lat0) * k)


def rssi_to_range(rssi):
    """
    Log-distance path loss:   d = 10 ^ ((P1m - RSSI) / (10 n))

    Returns (distance_m, sigma_m). Sigma is proportional to distance
    because RSSI error is multiplicative in range — which is why the gate
    widens the further away something is. That behaviour is real and worth
    showing, not smoothing over.
    """
    p = CFG["propagation"]
    if rssi is None:
        return None, None
    d = 10 ** ((p["rssi_at_1m"] - rssi) / (10.0 * p["path_loss_n"]))
    sigma = d * math.log(10) / (10.0 * p["path_loss_n"]) * p["rssi_sigma"]
    return d, sigma


def gate_radius(sigma_range):
    g = CFG["gate"]
    return (g["k"] * math.hypot(sigma_range, g["sigma_claim_m"])
            + g["v_max"] * g["latency_s"])


# =====================================================================
#  listeners
# =====================================================================

class Listener(threading.Thread):
    def __init__(self, port, handler):
        super().__init__(daemon=True)
        self.port, self.handler = port, handler

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", self.port))
        while True:
            try:
                data, _ = s.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get("magic") == MAGIC:
                    self.handler(msg)
            except Exception:
                continue


LATEST_OBS = {"t": 0, "obs": []}
CLAIMS = {}


def on_observation(msg):
    with state_lock:
        LATEST_OBS["t"] = msg["t"]
        LATEST_OBS["obs"] = msg["obs"]
        STATE["node_seen"] = time.time()


def on_claim(msg):
    with state_lock:
        CLAIMS[msg["uid"]] = msg


# =====================================================================
#  the engine
# =====================================================================

def log_event(kind, text):
    STATE["events"].insert(0, {"t": time.strftime("%H:%M:%S"),
                               "kind": kind, "text": text})
    del STATE["events"][40:]


def cycle(dt):
    now = time.time()
    node = CFG["node"]
    reg = CFG["registry"]

    with state_lock:
        raw_obs = list(LATEST_OBS["obs"])
        obs_age = now - LATEST_OBS["t"] if LATEST_OBS["t"] else 999
        claims = [c for c in CLAIMS.values() if now - c["t"] <= STALE_S]

    # ---- build tracks from observations ------------------------------
    tracks = []
    for o in raw_obs:
        d, sig = rssi_to_range(o["rssi"])
        if d is None:
            continue
        tracks.append({"mac": o["mac"], "ssid": o["ssid"], "rssi": o["rssi"],
                       "range_m": d, "sigma_m": sig, "gate_m": gate_radius(sig)})

    # ---- expected range for each claim -------------------------------
    reports = []
    for c in claims:
        e, n = enu(c["lat"], c["lon"], node["lat"], node["lon"])
        reports.append({"uid": c["uid"], "mac": c.get("mac"),
                        "range_m": math.hypot(e, n), "claim": c,
                        "registered": c["uid"] in reg})

    # ---- association: gate, then one-to-one --------------------------
    # A report may be claimed by at most one track and vice versa. The
    # cost is the discrepancy between observed range and claimed range.
    pairs = []
    for i, t in enumerate(tracks):
        for j, r in enumerate(reports):
            cost = abs(t["range_m"] - r["range_m"])
            if cost <= t["gate_m"]:
                pairs.append((cost, i, j))
    pairs.sort()

    used_t, used_r, matched = set(), set(), []
    for cost, i, j in pairs:
        if i in used_t or j in used_r:
            continue
        used_t.add(i); used_r.add(j)
        t, r = tracks[i], reports[j]

        # signature verification: does the claimed identity carry the
        # signature the registry recorded for it?
        expected = reg.get(r["uid"], {}).get("signature_mac", "").lower()
        sig_ok = (expected == t["mac"]) if expected and "FILL" not in expected.upper() else None

        matched.append({"mac": t["mac"], "ssid": t["ssid"], "uid": r["uid"],
                        "cost_m": round(cost, 1),
                        "obs_range": round(t["range_m"], 1),
                        "claim_range": round(r["range_m"], 1),
                        "gate_m": round(t["gate_m"], 1),
                        "registered": r["registered"],
                        "signature_ok": sig_ok})

    unmatched_t = [tracks[i] for i in range(len(tracks)) if i not in used_t]
    unmatched_r = [reports[j] for j in range(len(reports)) if j not in used_r]

    # ---- evidence accumulation ---------------------------------------
    ev = CFG["evidence"]
    bad_macs = {t["mac"] for t in unmatched_t}
    for m in matched:
        # a match on position but a wrong signature is stronger evidence
        # than a position gap, so it accrues faster
        if m["signature_ok"] is False or not m["registered"]:
            bad_macs.add(m["mac"])

    seen = {t["mac"] for t in tracks}
    for mac in list(EVIDENCE):
        if mac not in seen:
            del EVIDENCE[mac]

    for t in tracks:
        e = EVIDENCE.setdefault(t["mac"], {"score": 0.0, "escalated": False,
                                           "since": now, "ssid": t["ssid"]})
        if t["mac"] in bad_macs:
            e["score"] = min(ev["escalate_at"] * 1.4,
                             e["score"] + ev["gain_per_second"] * dt)
        else:
            e["score"] = max(0.0, e["score"] - ev["decay_per_second"] * dt)

        if not e["escalated"] and e["score"] >= ev["escalate_at"]:
            e["escalated"] = True
            reason = "holds no permission"
            for m in matched:
                if m["mac"] == t["mac"]:
                    reason = ("claimed identity not in registry"
                              if not m["registered"] else
                              "signature does not match claimed identity")
            log_event("escalate", f"{t['ssid']} ({t['mac']}) — {reason}")
            STATE["counters"]["escalations"] += 1
        if e["escalated"] and e["score"] < ev["escalate_at"] * 0.2:
            e["escalated"] = False
            log_event("clear", f"{t['ssid']} resolved")

    # ---- publish state ------------------------------------------------
    with state_lock:
        STATE["t"] = now
        STATE["obs_age"] = round(obs_age, 1)
        STATE["observations"] = [
            {"ssid": t["ssid"], "mac": t["mac"], "rssi": t["rssi"],
             "range_m": round(t["range_m"], 1),
             "sigma_m": round(t["sigma_m"], 1),
             "gate_m": round(t["gate_m"], 1),
             "score": round(EVIDENCE.get(t["mac"], {}).get("score", 0), 1),
             "escalated": EVIDENCE.get(t["mac"], {}).get("escalated", False)}
            for t in tracks]
        STATE["claims"] = [
            {"uid": r["uid"], "mac": r["mac"],
             "range_m": round(r["range_m"], 1),
             "registered": r["registered"],
             "age": round(now - r["claim"]["t"], 1)} for r in reports]
        STATE["matched"] = matched
        STATE["unmatched"] = [{"ssid": t["ssid"], "mac": t["mac"],
                               "range_m": round(t["range_m"], 1)}
                              for t in unmatched_t]
        STATE["unmatched_reports"] = [r["uid"] for r in unmatched_r]


def engine_loop():
    last = time.time()
    while True:
        now = time.time()
        cycle(min(1.0, now - last))
        last = now
        time.sleep(0.4)


# =====================================================================
#  display
# =====================================================================

PAGE = r"""<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Operator Display</title><style>
:root{--bg:#f4f6f8;--panel:#fff;--ink:#14202c;--body:#5a6773;--line:#dfe5ea;
--navy:#0d1b2a;--teal:#2f7f99;--amber:#c98420;--red:#c1553b;--grey:#8a98a5}
@media(prefers-color-scheme:dark){:root:not([data-theme=light]){
--bg:#0b141d;--panel:#111e2a;--ink:#e8eef3;--body:#9fb0bd;--line:#1e3344}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
.wrap{max-width:1180px;margin:0 auto;padding:16px}
h1{font-size:19px;margin:0 0 2px}.sub{color:var(--body);font-size:12.5px;margin-bottom:14px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
@media(max-width:820px){.grid{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:11px;padding:13px}
.card h2{font-size:11px;letter-spacing:.9px;text-transform:uppercase;color:var(--grey);
margin:0 0 9px;font-weight:700}
table{width:100%;border-collapse:collapse;font-size:13px}
th{text-align:left;font-size:10.5px;letter-spacing:.5px;text-transform:uppercase;
color:var(--grey);padding:4px 6px;font-weight:700}
td{padding:6px;border-top:1px solid var(--line);font-variant-numeric:tabular-nums}
.tag{display:inline-block;padding:1px 7px;border-radius:9px;font-size:11px;font-weight:700}
.ok{background:rgba(47,127,153,.16);color:var(--teal)}
.warn{background:rgba(201,132,32,.18);color:var(--amber)}
.bad{background:rgba(193,85,59,.18);color:var(--red)}
.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:9px;margin-bottom:14px}
.m{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.m .k{font-size:10px;letter-spacing:.6px;text-transform:uppercase;color:var(--grey);font-weight:700}
.m .v{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums}
.bar{height:5px;background:var(--line);border-radius:3px;overflow:hidden;margin-top:4px}
.bar i{display:block;height:100%;background:var(--amber);transition:width .3s}
.bar i.hot{background:var(--red)}
.ev{max-height:190px;overflow-y:auto;font-size:12.5px}
.ev div{padding:5px 0;border-top:1px solid var(--line);display:flex;gap:9px}
.ev .tm{color:var(--grey);font-variant-numeric:tabular-nums}
.ev .escalate{color:var(--red);font-weight:600}
.ev .clear{color:var(--teal)}
.dead{color:var(--red);font-weight:600}
.empty{color:var(--grey);font-size:12.5px;padding:8px 6px}
</style></head><body><div class="wrap">
<h1>Operator Display</h1>
<div class="sub">Tripod node · observations over the air, claims over the network</div>
<div class="metrics">
 <div class="m"><div class="k">Observed</div><div class="v" id="nObs">0</div></div>
 <div class="m"><div class="k">Claimed</div><div class="v" id="nClm">0</div></div>
 <div class="m"><div class="k">Matched</div><div class="v" id="nMat">0</div></div>
 <div class="m"><div class="k">Escalations</div><div class="v" id="nEsc" style="color:var(--red)">0</div></div>
</div>
<div class="grid">
 <div class="card"><h2>Observed — over the air</h2>
  <table id="tObs"><thead><tr><th>SSID</th><th>RSSI</th><th>Range</th><th>Gate</th><th>Evidence</th></tr></thead><tbody></tbody></table>
  <div class="empty" id="eObs">waiting for the sensor node…</div></div>
 <div class="card"><h2>Claimed — over the network</h2>
  <table id="tClm"><thead><tr><th>Identity</th><th>Range</th><th>Age</th><th>Registry</th></tr></thead><tbody></tbody></table>
  <div class="empty" id="eClm">no claims received</div></div>
 <div class="card"><h2>Association</h2>
  <table id="tMat"><thead><tr><th>Track</th><th>Identity</th><th>Δ range</th><th>Gate</th><th>Signature</th></tr></thead><tbody></tbody></table>
  <div class="empty" id="eMat">nothing matched</div></div>
 <div class="card"><h2>Events</h2><div class="ev" id="evs"></div></div>
</div></div>
<script>
function tag(t,c){return '<span class="tag '+c+'">'+t+'</span>'}
async function tick(){
 let s; try{ s = await (await fetch('/api/state')).json(); }catch(e){ return; }
 document.getElementById('nObs').textContent = s.observations.length;
 document.getElementById('nClm').textContent = s.claims.length;
 document.getElementById('nMat').textContent = s.matched.length;
 document.getElementById('nEsc').textContent = s.counters.escalations;

 const ob = document.querySelector('#tObs tbody');
 ob.innerHTML = s.observations.map(o=>{
  const pct = Math.min(100, o.score*10);
  return '<tr><td>'+o.ssid+(o.escalated?' '+tag('ESCALATED','bad'):'')+
   '</td><td>'+o.rssi+'</td><td>'+o.range_m+' m ±'+o.sigma_m+
   '</td><td>'+o.gate_m+' m</td><td><div class="bar"><i class="'+
   (o.score>6?'hot':'')+'" style="width:'+pct+'%"></i></div></td></tr>';
 }).join('');
 document.getElementById('eObs').style.display = s.observations.length?'none':'block';
 document.getElementById('eObs').textContent = s.obs_age>5
   ? 'sensor node silent for '+s.obs_age+'s' : 'waiting for the sensor node…';
 document.getElementById('eObs').className = s.obs_age>5?'empty dead':'empty';

 const cl = document.querySelector('#tClm tbody');
 cl.innerHTML = s.claims.map(c=>'<tr><td>'+c.uid+'</td><td>'+c.range_m+
  ' m</td><td>'+c.age+'s</td><td>'+(c.registered?tag('accredited','ok')
  :tag('UNREGISTERED','bad'))+'</td></tr>').join('');
 document.getElementById('eClm').style.display = s.claims.length?'none':'block';

 const mt = document.querySelector('#tMat tbody');
 mt.innerHTML = s.matched.map(m=>{
  let sig = m.signature_ok===null ? tag('not enrolled','warn')
          : m.signature_ok ? tag('verified','ok') : tag('MISMATCH','bad');
  return '<tr><td>'+m.ssid+'</td><td>'+m.uid+'</td><td>'+m.cost_m+
   ' m</td><td>'+m.gate_m+' m</td><td>'+sig+'</td></tr>';
 }).join('');
 let extra = s.unmatched.map(u=>'<tr><td>'+u.ssid+'</td><td>'+
   tag('NO PERMISSION','bad')+'</td><td>'+u.range_m+' m</td><td>—</td><td>—</td></tr>').join('');
 mt.innerHTML += extra;
 document.getElementById('eMat').style.display =
   (s.matched.length+s.unmatched.length)?'none':'block';

 document.getElementById('evs').innerHTML = s.events.map(e=>
  '<div><span class="tm">'+e.t+'</span><span class="'+e.kind+'">'+e.text+'</span></div>'
 ).join('') || '<div class="empty">no events yet</div>';
}
setInterval(tick, 500); tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/api/state"):
            with state_lock:
                body = json.dumps(STATE).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)


def main():
    global CFG
    ap = argparse.ArgumentParser()
    ap.add_argument("--site", default="site.json")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args()

    with open(a.site) as f:
        CFG = json.load(f)
    CFG["registry"] = {k: v for k, v in CFG["registry"].items()
                       if not k.startswith("_")}

    Listener(OBS_PORT, on_observation).start()
    Listener(CLAIM_PORT, on_claim).start()
    threading.Thread(target=engine_loop, daemon=True).start()

    print(f"  discrimination engine running")
    print(f"  observations  UDP {OBS_PORT}")
    print(f"  claims        UDP {CLAIM_PORT}")
    print(f"  registry      {len(CFG['registry'])} accredited airframe(s)")
    print(f"\n  open  http://localhost:{a.port}\n")
    ThreadingHTTPServer(("", a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()

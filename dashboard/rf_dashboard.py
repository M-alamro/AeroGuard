#!/usr/bin/env python3
"""
rf_dashboard.py - live map and radio console for Gazebo RF comms endpoints.

Uses the same conventions as Rf.py:
  * every radio model has a CommsEndpoint whose <address> is the model name
    and whose <topic> is /<address>/rx
  * the RFComms settings are in
    ~/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config

RUN (with Gazebo already running)
    python3 rf_dashboard.py
The page opens by itself at http://localhost:8765

TRY IT WITHOUT GAZEBO
    python3 rf_dashboard.py --demo

THE PAGE
  Map      Pick a centre node. Every other endpoint is drawn at its real x, y
           offset from it (top-down), labelled with its z.
  Friend or foe
           Every identity broadcast the centre node receives is checked against
           the registry by friend_or_foe.py (keep it in the same folder), and
           friendly drones are checked against the zone their time frame gives
           them. Verdicts, zone status and the zones themselves show on the map,
           in the list and in their own panel.
  Transmit Send one message or start a beacon from any endpoint, to chosen
           endpoints or to all of them, with your own text, rate, padding and
           message count. Beacons run here, in this script, until stopped.
  Listen   A live log of every message RFComms delivers, filtered by receiver,
           sender or text, exportable as CSV.
  Computer vision
           The newest camera frame from your computer-vision program. It sends
           each frame as the body of a POST to http://localhost:8765/cv (JPEG,
           PNG or WebP bytes), for example
               requests.post("http://localhost:8765/cv", data=cv2.imencode(".jpg", img)[1].tobytes())

WHAT "LINKED" MEANS
    RFComms has no connections: every message is delivered or dropped on its
    own, with a probability that falls with distance. The page computes that
    probability with the same formula RFComms uses (log-distance path loss,
    random fading, QPSK bit errors) and calls an endpoint linked to the centre
    when it is at or above the threshold set on the page.

MESSAGE FORMAT
    With "Add ID and SEQ header" ticked, messages look like
        ID:drone1;BID:3;SEQ:42;MSG:your text
    which Rf.py also understands (it reads SEQ). The SEQ numbers let the page
    work out what share of a beacon arrived.
"""
import argparse
import http.server
import importlib
import json
import math
import os
import random
import re
import subprocess
import sys
import threading
import time
import webbrowser
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:                                    # the same code as the terminal program
    import friend_or_foe as fof
    fof.USE_COLOUR = False              # its messages go to the page, not a terminal
except ImportError:
    fof = None


# ---------------------------------------------------------------- settings

DEFAULT_CONFIG = Path.home() / "PX4-Autopilot/src/modules/simulation/gz_bridge/server.config"

# The values RFComms itself uses when one is missing from the config.
RF_DEFAULTS = {"tx_power": 27.0, "l0": 40.0, "fading_exponent": 2.5, "sigma": 10.0,
               "noise_floor": -90.0, "max_range": 50.0, "capacity": 54000000.0}

# Example values for --demo when no server.config is found.
DEMO_RF = {"tx_power": 20.0, "l0": 40.0, "fading_exponent": 2.6, "sigma": 10.0,
           "noise_floor": -90.0, "max_range": 500000.0, "capacity": 1000000.0}

BROKER_TOPIC = "/broker/msgs"
DEFAULT_PACKET_BYTES = 32   # RFComms uses the payload size; replaced by the real one once seen
RX_TOPIC = re.compile(r"^/([^/]+)/rx$")
POSE_TOPIC = re.compile(r"^/world/([^/]+)/pose/info$")
RESCAN_S = 3.0              # how often to look for new endpoints
RATE_WINDOW_S = 5.0         # window for messages per second
SEQ_WINDOW = 20             # recent beacon messages used for the delivered %
LOG_SIZE = 5000             # received messages kept for the Listen panel
BACKLOG = 200               # messages a freshly opened page gets straight away
MAX_PUSH = 400              # most messages sent to the page in one update
MAX_RATE = 200.0            # beacon messages per second, per beacon
CV_WAIT_S = 1.0             # how long the page's request for the next camera frame waits
CV_MAX_BYTES = 20_000_000   # largest camera frame accepted
CV_STALE_S = 3.0            # no frame for this long: the page says the camera stopped


# ---------------------------------------------------------------- helpers

def load_rf(path):
    """RFComms values from server.config, plus a note if something is off."""
    rf = dict(RF_DEFAULTS)
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as e:
        return rf, f"Could not read {path} ({e.__class__.__name__}), so RFComms defaults are shown."
    for plugin in root.iter("plugin"):
        if "rf-comms" in plugin.get("filename", ""):
            for key in rf:
                el = plugin.find(f".//{key}")
                if el is not None and el.text and el.text.strip():
                    try:
                        rf[key] = float(el.text)
                    except ValueError:
                        pass
            return rf, None
    return rf, f"No RFComms plugin in {path}, so RFComms defaults are shown."


def parse_fields(text):
    """ID, BID and SEQ from a payload like 'ID:drone1;BID:3;SEQ:42;MSG:hi'."""
    out = {}
    for part in text.split(";"):
        key, sep, value = part.partition(":")
        if sep and key in ("ID", "BID", "SEQ") and key not in out:
            out[key] = value
    try:
        out["SEQ"] = int(out["SEQ"]) if "SEQ" in out else None
    except ValueError:
        out["SEQ"] = None
    return out


def image_type(data):
    """Image type from the first bytes of a file, or None if it is not JPEG, PNG or WebP."""
    if data[:3] == b"\xff\xd8\xff":
        return "image/jpeg"
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return "image/png"
    if data[:4] == b"RIFF" and data[8:12] == b"WEBP":
        return "image/webp"
    return None


def gz_topics():
    try:
        res = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=10)
        return res.stdout.split()
    except (OSError, subprocess.TimeoutExpired):
        return []


def load_gz():
    """Import Gazebo's Python bindings (Harmonic first, then Ionic)."""
    for transport, msgs in (("transport13", "msgs10"), ("transport14", "msgs11")):
        try:
            node_cls = importlib.import_module(f"gz.{transport}").Node
            dataframe = importlib.import_module(f"gz.{msgs}.dataframe_pb2").Dataframe
            pose_v = importlib.import_module(f"gz.{msgs}.pose_v_pb2").Pose_V
            return node_cls, dataframe, pose_v
        except ModuleNotFoundError:
            continue
        except Exception:
            # Usually the protobuf clash with a newer copy in ~/.local.
            # Restart once with -s so Python ignores ~/.local.
            if not sys.flags.no_user_site:
                os.execv(sys.executable, [sys.executable, "-s", *sys.argv])
            raise
    sys.exit("Gazebo Python bindings not found. Install them with:\n"
             "  sudo apt install python3-gz-transport13 python3-gz-msgs10\n"
             "or try the page without Gazebo:  python3 rf_dashboard.py --demo")


# ---------------------------------------------------------------- shared state

class LinkStats:
    """Messages actually received on one direction of one link."""

    def __init__(self):
        self.count = 0
        self.times = deque(maxlen=2000)
        self.first_t = None
        self.last_t = 0.0
        self.rssi = None
        self.streams = {}       # beacon id -> recent SEQ numbers
        self.stream = None      # the stream heard most recently

    def add(self, now, seq, stream, rssi):
        self.count += 1
        self.times.append(now)
        self.first_t = self.first_t or now
        self.last_t = now
        if rssi is not None:
            self.rssi = rssi
        if seq is not None:
            st = self.streams.get(stream)
            if st is None or (st["seqs"] and seq < st["seqs"][-1]):   # new or restarted
                st = self.streams[stream] = {"first": seq, "seqs": deque(maxlen=SEQ_WINDOW)}
                while len(self.streams) > 32:
                    self.streams.pop(next(iter(self.streams)))
            st["seqs"].append(seq)
            self.stream = stream

    def summary(self, now):
        recent = sum(1 for t in self.times if now - t <= RATE_WINDOW_S)
        span = max(1.0, min(RATE_WINDOW_S, now - self.first_t))
        delivered = None
        st = self.streams.get(self.stream)
        if st and st["seqs"]:
            latest = st["seqs"][-1]
            expected = min(SEQ_WINDOW, latest - st["first"] + 1)
            got = sum(1 for q in st["seqs"] if q > latest - SEQ_WINDOW)
            if expected > 0:
                delivered = 100.0 * got / expected
        return {"count": self.count, "rate": recent / span, "delivered": delivered,
                "rssi": self.rssi, "age": now - self.last_t}


class Hub:
    """Thread-safe store of everything the page shows."""

    def __init__(self, source, rf_path):
        self.source = source
        self.server_id = "%08x" % random.getrandbits(32)
        self.rf_path = Path(rf_path).expanduser()
        self._lock = threading.Lock()
        self._rf_mtime = self._mtime()
        self.rf, self.rf_note = load_rf(self.rf_path)
        self.world = None
        self.endpoints = []
        self.poses = {}
        self.last_pose = 0.0
        self.links = {}
        self.packet_bytes = DEFAULT_PACKET_BYTES
        self.log = deque(maxlen=LOG_SIZE)
        self.log_id = 0
        self.fof = {}                   # receiver -> (fof.Tracker, recent change lines)
        self.cv_frame = None            # newest camera frame (bytes) from the computer-vision program
        self.cv_type = ""               # its image type, e.g. image/jpeg
        self.cv_id = 0                  # counts frames, so the page only downloads new ones
        self.cv_times = deque(maxlen=60)
        self.cv_new = threading.Condition(self._lock)
        if fof:
            threading.Thread(target=self._fof_clock, daemon=True).start()

    # ---- writers
    def _mtime(self):
        try:
            return self.rf_path.stat().st_mtime
        except OSError:
            return None

    def reload_rf_if_changed(self):
        mtime = self._mtime()
        if mtime != self._rf_mtime:
            self._rf_mtime = mtime
            rf, note = load_rf(self.rf_path)
            with self._lock:
                self.rf, self.rf_note = rf, note

    def set_rf(self, rf, note):
        with self._lock:
            self.rf, self.rf_note = dict(rf), note

    def set_world(self, world):
        with self._lock:
            self.world = world

    def set_endpoints(self, names):
        with self._lock:
            self.endpoints = list(names)

    def update_poses(self, items):
        with self._lock:
            for name, x, y, z in items:
                self.poses[name] = (x, y, z)
            self.last_pose = time.time()

    # ---- computer vision: the newest camera frame
    def set_cv_frame(self, frame, ctype):
        with self.cv_new:
            self.cv_frame, self.cv_type = frame, ctype
            self.cv_id += 1
            self.cv_times.append(time.time())
            self.cv_new.notify_all()

    def wait_cv_frame(self, after, timeout):
        """(id, type, bytes) of the newest frame if it is newer than frame `after`,
        waiting up to `timeout` seconds for one to arrive; None if none did."""
        with self.cv_new:
            if after > self.cv_id:              # the page is from before a restart
                after = 0
            self.cv_new.wait_for(lambda: self.cv_id > after, timeout)
            if self.cv_id <= after:
                return None
            return self.cv_id, self.cv_type, self.cv_frame

    def _cv_snapshot(self, now):
        if not self.cv_id:
            return {"id": 0}
        recent = [t for t in self.cv_times if now - t <= 2.0]
        span = recent[-1] - recent[0] if len(recent) > 1 else 0
        return {"id": self.cv_id, "age": round(now - self.cv_times[-1], 2),
                "fps": round((len(recent) - 1) / span, 1) if span > 0 else 0, "stale_after": CV_STALE_S}

    # ---- friend or foe: each receiver checks what it hears, like friend_or_foe.py does
    def _fof_tracker(self, receiver):
        with self._lock:
            if receiver not in self.fof:
                events = deque(maxlen=12)
                self.fof[receiver] = (fof.Tracker(on_change=events.append, position_of=self.position_of),
                                      events)
            return self.fof[receiver][0]

    def _fof_clock(self):
        """Let trackers notice silence even when no messages arrive."""
        while True:
            time.sleep(1.0)
            with self._lock:
                trackers = [t for t, _ in self.fof.values()]
            for tracker in trackers:
                tracker.report_changes()

    def _fof_snapshot(self, now):
        if not fof:
            return {"available": False}
        listeners = {}
        for receiver, (tracker, events) in self.fof.items():
            listeners[receiver] = {
                "events": list(events),
                "tracks": [{
                    "sender": t.sender,
                    "remote_id": t.broadcast.remote_id, "sig_mac": t.broadcast.sig_mac,
                    "status": status, "code": t.verdict.code, "label": t.verdict.label,
                    "reason": t.verdict.reason,
                    "zone_state": t.zone.state, "zone": t.zone.zone, "zone_reason": t.zone.reason,
                    "age": round(age, 1), "messages": t.messages,
                } for t, status, age in tracker.snapshot(now)],
            }
        registry = []
        for rid, drone in fof.REGISTRY.items():
            zone, frame = fof.scheduled_zone(rid, now)
            registry.append({"remote_id": rid, "signature": drone.signature, "schedule": drone.schedule,
                             "zone_now": zone, "frame_now": frame})
        zones = {name: [z.x_min, z.x_max, z.y_min, z.y_max] for name, z in fof.ZONES.items()}
        return {"available": True, "registry": registry, "zones": zones,
                "silent_after": fof.SILENT_AFTER_S, "listeners": listeners}

    def record(self, src, dst, payload, rssi):
        """A message that RFComms delivered to dst."""
        now = time.time()
        text = payload.decode(errors="replace")
        fields = parse_fields(text)
        if rssi is not None and not math.isfinite(rssi):
            rssi = None
        with self._lock:
            if payload:
                self.packet_bytes = len(payload)
            st = self.links.get((src, dst))
            if st is None:
                st = self.links[(src, dst)] = LinkStats()
            st.add(now, fields["SEQ"], fields.get("BID"), rssi)
            pa, pb = self._pos(src), self._pos(dst)
            self.log_id += 1
            self.log.append({
                "id": self.log_id, "t": round(now, 3), "src": src, "dst": dst,
                "text": text if len(text) <= 600 else text[:600] + "\u2026",
                "bytes": len(payload), "rssi": None if rssi is None else round(rssi, 2),
                "d": round(math.dist(pa, pb), 2) if pa and pb else None,
            })
        if fof:
            self._fof_tracker(dst).handle(src, text, now)

    # ---- readers
    def rf_copy(self):
        with self._lock:
            return dict(self.rf)

    def endpoint_names(self):
        with self._lock:
            return list(self.endpoints)

    def _model_of(self, name):
        """Model named NAME, or the single model named NAME_<something> (PX4 adds _0)."""
        if name in self.poses:
            return name
        matches = [m for m in self.poses if m.startswith(name + "_")]
        return matches[0] if len(matches) == 1 else None

    def _pos(self, name):
        model = self._model_of(name)
        return self.poses.get(model) if model else None

    def position_of(self, name):
        with self._lock:
            return self._pos(name)

    def last_log_id(self):
        with self._lock:
            return self.log_id

    def messages_after(self, cursor, limit):
        """Messages with id > cursor (at most `limit`, newest kept), new cursor, number skipped."""
        with self._lock:
            pending = []
            for m in reversed(self.log):
                if m["id"] <= cursor:
                    break
                pending.append(m)
            pending.reverse()
            skipped = max(0, len(pending) - limit)
            if skipped:
                pending = pending[-limit:]
            return pending, (pending[-1]["id"] if pending else cursor), skipped

    def snapshot(self):
        now = time.time()
        with self._lock:
            eps = []
            for name in self.endpoints:
                model = self._model_of(name)
                pos = self.poses.get(model) if model else None
                eps.append({"name": name, "model": model,
                            "pos": [round(v, 3) for v in pos] if pos else None})
            links = []
            for (src, dst), st in self.links.items():
                if now - st.last_t > 60:
                    continue
                s = st.summary(now)
                s.update(src=src, dst=dst)
                links.append(s)
            return {
                "server_id": self.server_id,
                "now": round(now, 3),
                "source": self.source,
                "gazebo": now - self.last_pose < 2.0,
                "world": self.world,
                "rf": dict(self.rf, packet_bytes=self.packet_bytes),
                "rf_note": self.rf_note,
                "rf_path": str(self.rf_path),
                "endpoints": eps,
                "links": links,
                "fof": self._fof_snapshot(now),
                "cv": self._cv_snapshot(now),
            }


# ---------------------------------------------------------------- data sources

class GazeboFeed:
    """Live data from Gazebo, and the publisher used to send messages."""

    def __init__(self, hub, world=None):
        node_cls, self.Dataframe, self.Pose_V = load_gz()
        self.node = node_cls()
        self.hub = hub
        self.world_override = world
        self.pose_world = None
        self.rx_subscribed = set()
        self.empty_scans = 0
        self._pub = self.node.advertise(BROKER_TOPIC, self.Dataframe)
        self._pub_lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def publish(self, src, dst, data):
        msg = self.Dataframe()
        msg.src_address = src
        msg.dst_address = dst
        msg.data = data
        with self._pub_lock:
            self._pub.publish(msg)

    def _loop(self):
        while True:
            try:
                self.hub.reload_rf_if_changed()
                self._scan()
            except Exception as e:          # keep the page alive whatever happens
                print(f"scan error: {e}", file=sys.stderr)
            time.sleep(RESCAN_S)

    def _scan(self):
        topics = gz_topics()
        if not topics:
            self.empty_scans += 1
            if self.empty_scans >= 2:       # Gazebo has gone away
                self.hub.set_endpoints([])
            return
        self.empty_scans = 0

        names = sorted({m.group(1) for t in topics if (m := RX_TOPIC.match(t))})
        worlds = [m.group(1) for t in topics if (m := POSE_TOPIC.match(t))]
        world = self.world_override or (worlds[0] if worlds else None)
        self.hub.set_endpoints(names)

        if world:
            self.hub.set_world(world)
            if world != self.pose_world and self.node.subscribe(
                    self.Pose_V, f"/world/{world}/pose/info", self._on_poses):
                self.pose_world = world

        for name in names:
            if name not in self.rx_subscribed and self.node.subscribe(
                    self.Dataframe, f"/{name}/rx", self._make_rx(name)):
                self.rx_subscribed.add(name)

    def _on_poses(self, msg):
        self.hub.update_poses((p.name, p.position.x, p.position.y, p.position.z)
                              for p in msg.pose)

    def _make_rx(self, receiver):
        def on_msg(msg):
            rssi = None
            for entry in msg.header.data:     # RFComms adds the received power here
                if entry.key == "rssi" and entry.value:
                    try:
                        rssi = float(entry.value[0])
                    except ValueError:
                        pass
            self.hub.record(msg.src_address, receiver, bytes(msg.data), rssi)
        return on_msg


class DemoFeed:
    """Made-up endpoints that move around; messages are delivered or dropped
    with the same formula RFComms uses."""

    MODELS = {"drone1": "drone1", "drone2": "drone2", "drone3": "drone3", "drone4": "drone4",
              "toyota2": "toyota2", "x500": "x500_0"}

    def __init__(self, hub):
        self.hub = hub
        if hub.rf_note:
            hub.set_rf(DEMO_RF, "Demo: server.config not found, so example RF values are used.")
        hub.set_world("demo")
        hub.set_endpoints(sorted(self.MODELS))
        self.pos = self._positions(0.0)
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _positions(t):
        return {
            "drone1": (0.0, 0.0, 3.0),
            "drone2": (30 * math.cos(0.15 * t), 30 * math.sin(0.15 * t), 6.0 + 4 * math.sin(0.3 * t)),
            "drone3": (40 + 330 * (1 - math.cos(2 * math.pi * t / 90)), 25.0, 10.0),
            "drone4": (-35.0 + 8 * math.sin(0.2 * t), 20.0, 15.0),
            "toyota2": (-180 + 60 * math.sin(0.05 * t), 140.0, 0.5),
            "x500_0": (6 + math.sin(0.4 * t), -9.0, 12.0),
        }

    def _loop(self):
        t0 = time.time()
        while True:
            self.pos = self._positions(time.time() - t0)
            self.hub.update_poses((n, *p) for n, p in self.pos.items())
            time.sleep(0.05)

    def publish(self, src, dst, data):
        pos = self.pos
        a, b = pos.get(self.MODELS.get(src, src)), pos.get(self.MODELS.get(dst, dst))
        if a is None or b is None:
            return
        rf = self.hub.rf_copy()
        d = math.dist(a, b)
        if rf["max_range"] > 0 and d > rf["max_range"]:
            return
        mean = rf["tx_power"] - (rf["l0"] + 10 * rf["fading_exponent"] * math.log10(max(d, 1e-3)))
        rssi = random.gauss(mean, rf["sigma"]) if rf["sigma"] > 0 else mean
        ber = math.erfc(math.sqrt(10 ** ((rssi - rf["noise_floor"]) / 10)))
        p_ok = 0.0 if ber >= 1 else math.exp(len(data) * math.log1p(-ber))
        if random.random() < p_ok:
            self.hub.record(src, dst, data, rssi)


# ---------------------------------------------------------------- transmit console

class _Fields(dict):
    def __missing__(self, key):             # unknown {placeholders} stay as typed
        return "{" + key + "}"


class Beacon:
    def __init__(self, bid, src, dst, rate, text, header, pad, count):
        self.id, self.src, self.dst = bid, src, dst
        self.rate, self.period = rate, 1.0 / rate
        self.text, self.header, self.pad, self.count = text, header, pad, count
        self.seq = 0            # one SEQ per round, shared by all receivers (like Rf.py)
        self.rounds = 0
        self.messages = 0
        self.bytes = 0
        self.started = time.time()
        self.next_t = self.started

    def info(self):
        return {"id": self.id, "src": self.src, "dst": self.dst, "rate": self.rate,
                "text": self.text, "header": self.header, "pad": self.pad, "count": self.count,
                "rounds": self.rounds, "messages": self.messages, "bytes": self.bytes,
                "running_s": round(time.time() - self.started, 1)}


class Console:
    """Sends single messages and runs beacons from any endpoint."""

    def __init__(self, hub, feed):
        self.hub, self.feed = hub, feed
        self._lock = threading.Lock()
        self.beacons = {}
        self._next_id = 1
        self._oneshot_seq = {}
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- building messages
    def _targets(self, src, dst):
        names = self.hub.endpoint_names()
        if dst == "all":
            return [n for n in names if n != src]
        return [n for n in dst if n != src]

    def build(self, src, dst, text, header, pad, seq, bid):
        fields = _Fields(src=src, dst=dst, seq=seq, bid=bid if bid is not None else "",
                         t=round(time.time(), 3))
        pos = self.hub.position_of(src)
        if pos:
            fields.update(x=round(pos[0], 2), y=round(pos[1], 2), z=round(pos[2], 2))
        try:
            body = text.format_map(fields)
        except (ValueError, IndexError, KeyError, AttributeError):
            body = text                     # stray braces: send exactly what was typed
        if header:
            bid_part = f"BID:{bid};" if bid is not None else ""
            payload = f"ID:{src};{bid_part}SEQ:{seq};MSG:{body}".encode()
        else:
            payload = body.encode()
        missing = pad - len(payload)
        if missing > 0:
            payload += (b";PAD:" + b"~" * (missing - 5)) if header and missing > 5 else b"~" * missing
        return payload

    def _send_round(self, src, targets, text, header, pad, seq, bid):
        sent = 0
        for dst in targets:
            data = self.build(src, dst, text, header, pad, seq, bid)
            self.feed.publish(src, dst, data)
            sent += len(data)
        return sent

    # ---- actions from the page
    def send_once(self, src, dst, text, header, pad):
        targets = self._targets(src, dst)
        if not targets:
            raise ValueError("There is no other endpoint to send to.")
        with self._lock:
            seq = self._oneshot_seq.get(src, 0)
            self._oneshot_seq[src] = seq + 1
        total = self._send_round(src, targets, text, header, pad, seq, None)
        return {"ok": True, "sent_to": targets, "bytes": total // len(targets)}

    def start_beacon(self, src, dst, text, header, pad, rate, count):
        with self._lock:
            bid = self._next_id
            self._next_id += 1
            b = self.beacons[bid] = Beacon(bid, src, dst, rate, text, header, pad, count)
        return {"ok": True, "beacon": b.info()}

    def stop(self, which):
        with self._lock:
            if which == "all":
                n = len(self.beacons)
                self.beacons.clear()
                return {"ok": True, "stopped": n}
            try:
                bid = int(which)
            except (TypeError, ValueError):
                raise ValueError("Say which beacon to stop.")
            if self.beacons.pop(bid, None) is None:
                raise ValueError(f"Beacon {bid} is not running.")
            return {"ok": True, "stopped": 1}

    def list(self):
        with self._lock:
            return [b.info() for b in self.beacons.values()]

    # ---- the beacon clock
    def _loop(self):
        while True:
            now = time.time()
            due = []
            with self._lock:
                for b in self.beacons.values():
                    if now >= b.next_t:
                        due.append(b)
                        b.next_t += b.period
                        if b.next_t < now:          # fell behind: skip, don't burst
                            b.next_t = now + b.period
            for b in due:
                targets = self._targets(b.src, b.dst)
                sent = self._send_round(b.src, targets, b.text, b.header, b.pad, b.seq, b.id)
                with self._lock:
                    b.seq += 1
                    b.rounds += 1
                    b.messages += len(targets)
                    b.bytes += sent
                    if b.count and b.rounds >= b.count:
                        self.beacons.pop(b.id, None)
            time.sleep(0.002)


def parse_tx(body, names, beacon):
    """Check a send or beacon request from the page."""
    src = str(body.get("src") or "").strip()
    if not src:
        raise ValueError("Choose the endpoint to send from.")
    if src not in names:
        raise ValueError(f"There is no endpoint called {src} right now.")
    dst = body.get("dst")
    if dst != "all":
        if not isinstance(dst, list):
            raise ValueError("Choose at least one receiver, or All endpoints.")
        dst = [str(d) for d in dst if str(d) and str(d) != src]
        if not dst:
            raise ValueError("Choose at least one receiver other than the sender.")
    text = str(body.get("text") or "")
    if len(text) > 8000:
        raise ValueError("The message is longer than 8000 characters.")
    try:
        pad = int(body.get("pad") or 0)
    except (TypeError, ValueError):
        raise ValueError("Padding must be a whole number of bytes.")
    if not 0 <= pad <= 60000:
        raise ValueError("Padding must be between 0 and 60000 bytes.")
    out = {"src": src, "dst": dst, "text": text, "header": bool(body.get("header", True)), "pad": pad}
    if beacon:
        try:
            rate = float(body.get("rate"))
        except (TypeError, ValueError):
            raise ValueError("Set a rate in messages per second.")
        if not (0.05 <= rate <= MAX_RATE):
            raise ValueError(f"The rate must be between 0.05 and {MAX_RATE:g} per second.")
        try:
            count = int(body.get("count") or 0)
        except (TypeError, ValueError):
            raise ValueError("Stop after must be a whole number of messages.")
        if count < 0:
            raise ValueError("Stop after can't be negative.")
        out.update(rate=rate, count=count)
    return out


# ---------------------------------------------------------------- web server

def make_handler(hub, console, page, hz):
    body = page.encode()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        # ---- page and live data
        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", body)
            elif path == "/state":
                snap = hub.snapshot()
                snap["beacons"] = console.list()
                self._json(200, snap)
            elif path == "/events":
                self._stream()
            elif path == "/cv":
                self._cv_frame()
            else:
                self._send(404, "text/plain", b"not found")

        def _cv_frame(self):
            query = parse_qs(urlparse(self.path).query)
            try:
                after = int(query.get("after", ["0"])[0])
            except ValueError:
                after = 0
            got = hub.wait_cv_frame(after, CV_WAIT_S)
            if got is None:                     # nothing new yet: the page simply asks again
                self.send_response(204)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return
            frame_id, ctype, frame = got
            self._send(200, ctype, frame, {"X-Frame-Id": str(frame_id)})

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            period = 1.0 / hz
            cursor = max(0, hub.last_log_id() - BACKLOG)
            try:
                while True:
                    snap = hub.snapshot()
                    snap["beacons"] = console.list()
                    snap["msgs"], cursor, snap["skipped"] = hub.messages_after(cursor, MAX_PUSH)
                    self.wfile.write(f"data: {json.dumps(snap)}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        # ---- actions
        def do_POST(self):
            path = self.path.split("?")[0]
            origin = self.headers.get("Origin")
            if origin and urlparse(origin).netloc != self.headers.get("Host"):
                return self._json(403, {"error": "Requests from other sites are refused."})
            if path == "/cv":
                return self._receive_cv_frame()
            try:
                n = int(self.headers.get("Content-Length") or 0)
                req = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(req, dict):
                    raise ValueError
            except ValueError:
                return self._json(400, {"error": "The request body must be a JSON object."})
            try:
                names = hub.endpoint_names()
                if path == "/api/send":
                    res = console.send_once(**parse_tx(req, names, beacon=False))
                elif path == "/api/beacon":
                    res = console.start_beacon(**parse_tx(req, names, beacon=True))
                elif path == "/api/beacon/stop":
                    res = console.stop(req.get("id"))
                else:
                    return self._json(404, {"error": "Unknown action."})
            except ValueError as e:
                return self._json(400, {"error": str(e)})
            except Exception as e:                       # publishing failed
                return self._json(500, {"error": f"Could not send: {e}"})
            self._json(200, res)

        # ---- plumbing
        def _json(self, code, obj):
            self._send(code, "application/json", json.dumps(obj).encode())

        def _receive_cv_frame(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n > CV_MAX_BYTES:
                self.close_connection = True
                return self._send(413, "text/plain", f"Frames must be under {CV_MAX_BYTES} bytes.".encode())
            frame = self.rfile.read(n)
            ctype = image_type(frame)
            if ctype is None:
                return self._send(415, "text/plain", b"Send the frame as JPEG, PNG or WebP bytes in the request body.")
            hub.set_cv_frame(frame, ctype)
            self._send(200, "text/plain", b"OK")

        def _send(self, code, ctype, data, headers=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

    return Handler


def main():
    ap = argparse.ArgumentParser(description="Live RF map and radio console for Gazebo comms endpoints")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1",
                    help="use 0.0.0.0 to open the page from another computer "
                         "(anyone who can reach it can then send messages)")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                    help="server.config holding the RFComms values")
    ap.add_argument("--world", help="Gazebo world name (found automatically if omitted)")
    ap.add_argument("--hz", type=float, default=5.0, help="page updates per second")
    ap.add_argument("--demo", action="store_true", help="made-up data, no Gazebo needed")
    ap.add_argument("--no-browser", action="store_true", help="don't open the page automatically")
    args = ap.parse_args()

    if fof:
        fof.check_registry()            # stop here if the registry has a typo
    else:
        print("note: friend_or_foe.py not found next to this script; the friend-or-foe panel is off")
    hub = Hub("demo" if args.demo else "gazebo", args.config)
    feed = DemoFeed(hub) if args.demo else GazeboFeed(hub, args.world)
    console = Console(hub, feed)
    if args.demo:                       # some traffic to look at; stop it from the page
        tail = "_{x},{y},{z}_{t}"                                     # position and time: ignored by the check
        demo_identities = {"drone1": "UA-001_dc:a6:32:d1:b7:01" + tail,     # friendly, in its zone
                           "drone2": "UA-002_dc:a6:32:d1:b7:02" + tail,     # friendly, in and out of its zone
                           "drone3": "UA-003_dc:a6:32:d1:b7:04" + tail,     # UA-004's signature: hacking
                           "drone4": "UA-005_dc:a6:32:d1:b7:77" + tail,     # unknown signature: swapped
                           "toyota2": "UA-042_dc:a6:32:d1:b7:42" + tail,    # not in the registry: invader
                           "x500": "UA-004_" + tail}                        # no signature: sensing issue
        for name in hub.endpoint_names():
            console.start_beacon(name, "all", demo_identities.get(name, ""), True, 0, 1.0, 0)

    try:
        server = http.server.ThreadingHTTPServer((args.host, args.port),
                                                 make_handler(hub, console, PAGE, args.hz))
    except OSError as e:
        sys.exit(f"Can't use port {args.port} ({e.strerror}). Try --port {args.port + 1}")
    server.daemon_threads = True

    url = f"http://localhost:{args.port}"
    print(f"RF dashboard running at {url}   (Ctrl+C to stop; beacons stop with it)")
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


# ---------------------------------------------------------------- the page

PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RF links</title>
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'%3E%3Ccircle cx='16' cy='16' r='13' fill='none' stroke='%231D5FA6' stroke-width='2.5'/%3E%3Ccircle cx='16' cy='16' r='7' fill='none' stroke='%231D5FA6' stroke-width='2'/%3E%3Ccircle cx='16' cy='16' r='2.6' fill='%2315212B'/%3E%3C/svg%3E">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wdth,wght@62..125,100..900&display=swap">
<style>
:root{
  --paper:#E5EBEE; --sheet:#F5F8F9; --grid:#C5D0D6; --grid-strong:#8D9FAA;
  --ink:#15212B; --ink-soft:#4F6272;
  --good:#1D5FA6; --fair:#1F8A7A; --mid:#B07A10; --bad:#B3263E;
  --cover:rgba(29,95,166,.085); --cover-edge:#1D5FA6;
  --friend:#237A45; --foe:#C42B3C; --sensing:#A65A00; --zone:#6B4FB0;
  --font:"Archivo", ui-sans-serif, system-ui, "Segoe UI", Ubuntu, Roboto, sans-serif;
  color-scheme: light dark;
  --side-w:390px;     /* right-hand column: endpoint list and friend-or-foe side panel */
}
@media (prefers-color-scheme: dark){
  :root{
    --paper:#131C23; --sheet:#1A252E; --grid:#2D3C47; --grid-strong:#4D616E;
    --ink:#E3EBF0; --ink-soft:#97A9B6;
    --good:#6AA5E8; --fair:#4CC2A8; --mid:#DBA746; --bad:#E86A80;
    --cover:rgba(106,165,232,.11); --cover-edge:#6AA5E8;
    --friend:#5BC983; --foe:#F06A7E; --sensing:#F0A14A; --zone:#A58BEA;
  }
}
*{box-sizing:border-box}
[hidden]{display:none !important;}
html,body{margin:0}
body{
  background:var(--paper); color:var(--ink); font-family:var(--font);
  font-size:15px; line-height:1.45; font-variant-numeric:tabular-nums;
  -webkit-font-smoothing:antialiased;
}
code{font-family:ui-monospace, "SFMono-Regular", "Ubuntu Mono", Menlo, monospace; font-size:.92em;
  background:color-mix(in srgb, var(--grid) 55%, transparent); padding:1px 5px; border-radius:4px;}

/* ---------- header ---------- */
.top{display:flex; align-items:flex-end; justify-content:space-between; gap:12px 28px; flex-wrap:wrap;
  padding:18px 28px 14px; border-bottom:1px solid var(--grid);}
.pick label{display:block; font-size:13px; color:var(--ink-soft); margin-bottom:1px;}
.select-wrap{position:relative; display:inline-block;}
.select-wrap select{
  appearance:none; -webkit-appearance:none; background:transparent; color:var(--ink);
  border:0; border-bottom:2px solid var(--ink); border-radius:0;
  font:inherit; font-size:36px; line-height:1.1; font-weight:720; font-stretch:78%; letter-spacing:-.01em;
  padding:0 40px 3px 0; cursor:pointer; min-width:7ch;
}
.select-wrap select:focus-visible{outline:2px solid var(--good); outline-offset:4px;}
.select-wrap select:disabled{color:var(--ink-soft); border-bottom-color:var(--grid-strong); cursor:default;}
.select-wrap select option{font-size:16px; font-weight:400; font-stretch:100%;}
.select-wrap::after{content:""; position:absolute; right:8px; top:50%; width:11px; height:11px;
  border-right:2.5px solid var(--ink); border-bottom:2.5px solid var(--ink);
  transform:translateY(-75%) rotate(45deg); pointer-events:none;}
.status{display:flex; align-items:center; gap:9px; font-size:14px; color:var(--ink-soft); padding-bottom:6px;}
.status .dot{width:9px; height:9px; border-radius:50%; background:var(--grid-strong); flex:none;}
.status[data-state="live"] .dot{background:var(--good); box-shadow:0 0 0 4px color-mix(in srgb, var(--good) 22%, transparent);}
.status[data-state="wait"] .dot,.status[data-state="demo"] .dot{background:var(--mid);}
.status[data-state="down"] .dot{background:var(--bad);}

/* ---------- layout ---------- */
.layout{display:grid; grid-template-columns:minmax(0,1fr) var(--side-w);}
.plot-panel{padding:14px 28px 18px; border-right:1px solid var(--grid); display:flex; flex-direction:column; gap:10px; min-width:0;}
.tools{display:flex; gap:10px 26px; align-items:center; flex-wrap:wrap; font-size:13.5px; color:var(--ink-soft);}
.seg{display:inline-flex; border:1px solid var(--grid-strong); border-radius:6px; overflow:hidden;}
.seg button{font:inherit; font-size:13px; background:transparent; color:var(--ink-soft); border:0; padding:5px 13px; cursor:pointer;}
.seg button + button{border-left:1px solid var(--grid-strong);}
.seg button[aria-pressed="true"]{background:var(--ink); color:var(--sheet);}
.seg button:focus-visible{outline:2px solid var(--good); outline-offset:-2px;}
.thresh{display:flex; align-items:center; gap:10px;}
.thresh output{color:var(--ink); font-weight:650; min-width:3.2ch;}
.thresh input{accent-color:var(--good); width:150px;}
.legend{display:flex; align-items:center; gap:8px; flex-wrap:wrap;}
.legend span{white-space:nowrap;}
.legend .ramp{width:86px; height:7px; border-radius:4px;
  background:linear-gradient(90deg, var(--bad), var(--mid) 50%, var(--fair) 75%, var(--good));}
.legend .dash{display:inline-block; vertical-align:middle; width:26px; height:0; border-top:2px dashed var(--grid-strong); margin:0 4px 0 8px;}

.plot-wrap{position:relative; width:100%; max-width:min(100%, calc(100vh - 205px)); min-width:280px; aspect-ratio:1/1; margin:2px auto 0;}
#plot{width:100%; height:100%; display:block; overflow:visible; touch-action:manipulation;}
.plot-msg{position:absolute; inset:0; display:grid; place-items:center; text-align:center;
  padding:0 17%; color:var(--ink-soft); font-size:15px; pointer-events:none;}
.plot-msg.low{align-items:end; padding-bottom:16%;}
.plot-msg strong{display:block; color:var(--ink); font-size:18px; font-weight:650; margin-bottom:6px;}
.hint{margin:0; font-size:13px; color:var(--ink-soft); max-width:76ch;}

/* ---------- list ---------- */
.list-panel{background:var(--sheet); padding:8px 0 24px; overflow:auto; max-height:calc(100vh - 82px); position:sticky; top:0;}
.list-panel h2{display:flex; justify-content:space-between; align-items:baseline; gap:12px;
  font-size:14px; font-weight:620; color:var(--ink-soft); margin:16px 24px 8px;}
.list-panel h2 .n{font-weight:500;}
.list-panel ul{list-style:none; margin:0; padding:0;}
.note{margin:6px 24px 4px; font-size:13px; color:var(--ink-soft);}
.empty{padding:10px 24px 12px; font-size:13px; color:var(--ink-soft); border-top:1px solid var(--grid);}
.row{display:grid; grid-template-columns:minmax(0,1fr) auto; column-gap:12px; width:100%; text-align:left;
  font:inherit; color:inherit; background:transparent; border:0; border-top:1px solid var(--grid);
  padding:12px 24px 13px; cursor:pointer;}
.row:hover,.row.hl{background:color-mix(in srgb, var(--good) 8%, transparent);}
.row:focus-visible{outline:2px solid var(--good); outline-offset:-2px;}
.row .nm{font-weight:650; font-size:16px; overflow:hidden; text-overflow:ellipsis; white-space:nowrap;}
.row .meta{grid-column:1; font-size:12.5px; color:var(--ink-soft); min-height:1em;}
.row .dist{grid-column:2; grid-row:1 / span 2; justify-self:end; align-self:center;
  font-size:30px; font-weight:680; font-stretch:72%; line-height:1; letter-spacing:-.01em;}
.row .dist small{font-size:14px; font-weight:500; font-stretch:100%; color:var(--ink-soft); margin-left:3px;}
.row .meter{grid-column:1 / -1; position:relative; height:6px; margin:10px 0 6px; border-radius:3px;
  background:color-mix(in srgb, var(--grid) 75%, transparent);}
.row .meter i{position:absolute; left:0; top:0; bottom:0; border-radius:3px; transition:width .25s;}
.row .meter b{position:absolute; top:-3px; bottom:-3px; width:2px; margin-left:-1px; background:var(--ink); opacity:.7;}
.row .pred{grid-column:1 / -1; font-size:13px; color:var(--ink-soft);}
.row .heard{grid-column:1 / -1; font-size:13px; color:var(--ink); margin-top:2px;}
.row .heard.quiet{color:var(--ink-soft);}
.row .heard[hidden]{display:none;}
#out .row .nm,#out .row .dist{color:var(--ink-soft);}

/* ---------- computer vision ---------- */
.cv-panel{border-top:1px solid var(--grid); padding:18px 28px 24px;}
.cv-head{display:flex; justify-content:space-between; align-items:baseline; gap:8px 16px; flex-wrap:wrap;}
.cv-head h2{font-size:22px; font-weight:700; font-stretch:85%; margin:0;}
.cv-panel .status{padding-bottom:0;}
.cv-panel .help{margin:6px 0 12px;}
.cv-view{position:relative; width:100%; max-width:1100px; margin:0 auto; aspect-ratio:16/9; background:#000;
  border:1px solid var(--grid); border-radius:8px; overflow:hidden; display:grid; place-items:center;}
#cv-image{width:100%; height:100%; object-fit:contain; display:block;}
#cv-image.stale{opacity:.6;}
.cv-wait{position:absolute; inset:0; display:grid; place-content:center; gap:6px; padding:0 12%; text-align:center;
  color:#A9B6BF; font-size:13.5px;}
.cv-wait strong{color:#fff; font-size:18px; font-weight:650;}
.cv-wait code{background:rgba(255,255,255,.12); color:#E3EBF0;}
.cv-badge{position:absolute; left:12px; top:12px; padding:3px 10px; border-radius:999px; font-size:12.5px;
  font-weight:650; color:#fff; background:rgba(179,38,62,.85);}

/* ---------- friend or foe ---------- */
.fof-legend{display:flex; align-items:center; gap:6px; flex-wrap:wrap;}
.fof-legend svg{width:16px; height:16px; overflow:visible; vertical-align:middle;}
.fof-legend span{white-space:nowrap; margin-right:6px;}
.pill{display:inline-block; padding:1px 9px; border:1px solid currentColor; border-radius:999px;
  font-size:12px; font-weight:650; white-space:nowrap; line-height:1.5;}
.pill small{font-size:11px; font-weight:500; margin-left:4px;}
.pill.friend{color:var(--friend); background:color-mix(in srgb, var(--friend) 12%, transparent);}
.pill.foe{color:var(--foe); background:color-mix(in srgb, var(--foe) 12%, transparent);}
.pill.sensing{color:var(--sensing); background:color-mix(in srgb, var(--sensing) 12%, transparent);}
.pill.silent{color:var(--ink-soft);}
.pill + .pill{margin-left:4px;}
.pill.none{color:var(--ink-soft); border-style:dashed;}
.row .fofline{grid-column:1 / -1; font-size:12.5px; color:var(--ink-soft); margin-top:6px;}
.row .fofline[hidden]{display:none;}
.fof{display:grid; grid-template-columns:minmax(0,1fr) var(--side-w); border-top:1px solid var(--grid);}
.fof-main{padding:18px 28px 22px; min-width:0;}
.fof-side{padding:18px 24px 22px; background:var(--sheet); border-left:1px solid var(--grid);}
.fof-head{display:flex; align-items:baseline; justify-content:space-between; gap:8px 16px; flex-wrap:wrap;}
.fof-head h2{margin:0;}
.fof-counts{display:flex; gap:6px; flex-wrap:wrap;}
.fof .help{margin:6px 0 12px;}
.fof-wrap{max-height:380px;}
.fof td.reason{white-space:normal; min-width:220px;}
.fof tr.dim td{color:var(--ink-soft);}
.fof-side h3{font-size:14px; font-weight:620; color:var(--ink-soft); margin:0 0 6px;}
.fof-side h3 + ul{margin-bottom:20px;}
.fof-side ul{list-style:none; margin:0; padding:0; font-size:13px;}
.fof-events{max-height:330px; overflow:auto;}
.fof-events li{padding:6px 0; border-top:1px solid var(--grid);}
.fof-events .tm{color:var(--ink-soft); margin-right:8px;}
.fof-events .friend{color:var(--friend); font-weight:620;}
.fof-events .foe{color:var(--foe); font-weight:620;}
.fof-events .silent{color:var(--ink-soft); font-weight:620;}
.fof-events .sensing{color:var(--sensing); font-weight:620;}
.fof-registry li{display:grid; grid-template-columns:auto 1fr auto; column-gap:10px; row-gap:1px; align-items:baseline;
  padding:6px 0; border-top:1px solid var(--grid);}
.fof-registry .now{grid-column:1 / -1; font-size:12px; color:var(--ink-soft);}
.fof-registry .now b{color:var(--zone); font-weight:650;}
.fof-registry .who.foe{color:var(--foe);}
.fof-zones li{display:flex; justify-content:space-between; gap:10px; padding:5px 0; border-top:1px solid var(--grid);}
.fof-zones .zn{font-weight:650; color:var(--zone);}
.fof-zones .zb{font-size:12.5px; color:var(--ink-soft); font-variant-numeric:tabular-nums;}
.fof-registry .rid{font-weight:650;}
.fof-registry .mac{font-family:ui-monospace, "SFMono-Regular", "Ubuntu Mono", Menlo, monospace; font-size:12px; color:var(--ink-soft);}
.fof-registry .who{font-size:12.5px; color:var(--ink-soft); text-align:right;}
.fof-registry .who.friend{color:var(--friend);}
.fof-off{padding:18px 28px; color:var(--ink-soft); font-size:14px;}

/* ---------- radio console ---------- */
.console{display:grid; grid-template-columns:minmax(0, 0.85fr) minmax(0, 1.15fr); border-top:1px solid var(--grid);}
.console > div{padding:18px 28px 24px; min-width:0;}
.console .tx{border-right:1px solid var(--grid);}
.console .rx{background:var(--sheet);}
.console h2,.fof h2{font-size:22px; font-weight:700; font-stretch:85%; margin:0 0 14px;}
.console h3{display:flex; justify-content:space-between; align-items:center; gap:12px;
  font-size:14px; font-weight:620; color:var(--ink-soft); margin:26px 0 8px;}
.console h3 .n{font-weight:500; margin-left:4px;}
.console fieldset{border:0; padding:0; margin:0 0 14px; min-width:0;}
.console legend,.field > label{display:block; font-size:13px; color:var(--ink-soft); margin:0 0 6px; padding:0;}
.field{margin-bottom:14px;}
.field-row{display:flex; flex-wrap:wrap; align-items:center; gap:10px 20px; margin-bottom:14px; font-size:14px;}
.field-row .inline{display:flex; align-items:center; gap:8px;}
.field-row .grow{flex:1; min-width:180px;}
.field-row .grow input{flex:1; min-width:0;}
.console input[type=text],.console input[type=number],.console input[type=search],.console select{
  font:inherit; font-size:14px; color:var(--ink); background:var(--sheet); border:1px solid var(--grid-strong);
  border-radius:6px; padding:6px 9px;}
.console .rx input,.console .rx select{background:var(--paper);}
.console input:focus-visible,.console select:focus-visible,.btn:focus-visible,.primary:focus-visible,.chip input:focus-visible + span{
  outline:2px solid var(--good); outline-offset:2px;}
#tx-text{width:100%;}
#tx-src{min-width:12ch;}
.console input[type=number]{width:88px;}
.check{display:flex; align-items:center; gap:8px; cursor:pointer;}
.check input{accent-color:var(--good); width:16px; height:16px; margin:0;}
.help{margin:6px 0 0; font-size:12.5px; color:var(--ink-soft); max-width:72ch;}
.tx[data-mode="once"] .beacon-only{display:none;}
.chips{display:flex; flex-wrap:wrap; gap:6px;}
.chip{position:relative; display:inline-flex;}
.chip input{position:absolute; inset:0; opacity:0; margin:0; cursor:pointer;}
.chip span{display:inline-block; padding:4px 12px; border:1px solid var(--grid-strong); border-radius:999px;
  font-size:13.5px; color:var(--ink-soft); user-select:none;}
.chip input:checked + span{background:var(--ink); border-color:var(--ink); color:var(--sheet);}
.chip input:disabled{cursor:default;}
.chip input:disabled + span{opacity:.5;}
.chip.all span{font-weight:620;}
.primary{font:inherit; font-size:14.5px; font-weight:650; background:var(--good); color:var(--sheet);
  border:0; border-radius:6px; padding:8px 20px; cursor:pointer;}
.primary:disabled{opacity:.6; cursor:progress;}
.btn{font:inherit; font-size:13px; background:transparent; color:var(--ink); border:1px solid var(--grid-strong);
  border-radius:6px; padding:5px 12px; cursor:pointer;}
.btn[aria-pressed="true"]{background:var(--ink); border-color:var(--ink); color:var(--sheet);}
.actions{display:flex; align-items:center; flex-wrap:wrap; gap:8px 14px;}
.feedback{font-size:13.5px; color:var(--ink-soft);}
.feedback.err{color:var(--bad);}
.load{font-size:13px; color:var(--ink-soft); margin:0 0 12px;}
.load.warn{color:var(--bad);}
.beacons{list-style:none; margin:0; padding:0;}
.beacons li{display:flex; align-items:center; justify-content:space-between; gap:14px; padding:10px 0; border-top:1px solid var(--grid);}
.beacons .b-main{display:flex; flex-direction:column; min-width:0;}
.beacons .b-route{font-weight:620;}
.beacons .b-stats{font-size:13px;}
.beacons .b-msg{font-size:12.5px; color:var(--ink-soft); white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}
.rx-stat{font-size:13px; color:var(--ink-soft);}
.log-wrap{max-height:460px; overflow:auto; border:1px solid var(--grid); border-radius:6px; background:var(--paper);}
.log{width:100%; border-collapse:collapse; font-size:13px;}
.log th{position:sticky; top:0; background:var(--paper); text-align:left; font-weight:620; color:var(--ink-soft);
  padding:7px 10px; border-bottom:1px solid var(--grid-strong); white-space:nowrap;}
.log td{padding:5px 10px; border-bottom:1px solid var(--grid); white-space:nowrap; vertical-align:top;}
.log .num{text-align:right;}
.log td.t{color:var(--ink-soft);}
.log td.msg{font-family:ui-monospace, "SFMono-Regular", "Ubuntu Mono", Menlo, monospace; font-size:12.5px;
  width:100%; max-width:0; min-width:140px; overflow:hidden; text-overflow:ellipsis;}
.log tr.none td{color:var(--ink-soft); white-space:normal; padding:14px 10px;}

/* ---------- footer ---------- */
.rf{display:flex; flex-wrap:wrap; gap:6px 26px; padding:12px 28px 20px; border-top:1px solid var(--grid);
  font-size:13px; color:var(--ink-soft);}
.rf span b{font-weight:600; color:var(--ink);}
.rf .src{flex-basis:100%;}

/* ---------- plot ---------- */
#plot .disc{fill:var(--sheet);}
#plot .ring{fill:none; stroke:var(--grid);}
#plot .ring.major{stroke:var(--grid-strong);}
#plot .spoke{stroke:var(--grid);}
#plot .axis{stroke:var(--grid-strong);}
#plot .rim{fill:none; stroke:var(--ink);}
#plot .lbl{fill:var(--ink-soft); paint-order:stroke; stroke:var(--sheet); stroke-linejoin:round;}
#plot .axis-lbl{fill:var(--ink-soft); font-weight:600;}
#plot .cover{fill:var(--cover); stroke:var(--cover-edge); stroke-opacity:.7;}
#plot .maxring{fill:none; stroke:var(--bad); stroke-opacity:.75;}
#plot .cover-lbl{fill:var(--good); font-weight:600; paint-order:stroke; stroke:var(--sheet); stroke-linejoin:round;}
#plot .max-lbl{fill:var(--bad); font-weight:600; paint-order:stroke; stroke:var(--sheet); stroke-linejoin:round;}
#plot .link{fill:none; stroke-linecap:round;}
#plot .node{cursor:pointer;}
#plot .node .hit{fill:transparent;}
#plot .node text,#plot .centre text{paint-order:stroke; stroke:var(--sheet); stroke-linejoin:round;}
#plot .node .t-nm{fill:var(--ink); font-weight:620;}
#plot .node .t-d{fill:var(--ink-soft);}
#plot .frame{fill:none;}
#plot .t-rid{font-weight:650;}
#plot .t-rid.friend{fill:var(--friend);}
#plot .t-rid.foe{fill:var(--foe);}
#plot .t-rid.sensing{fill:var(--sensing);}
#plot .t-rid.silent,#plot .t-rid.none{fill:var(--ink-soft);}
#plot .t-out{fill:var(--foe); font-weight:650;}
#plot .zone path{fill:color-mix(in srgb, var(--zone) 6%, transparent); stroke:var(--zone); stroke-opacity:.5;}
#plot .zone.needed path{fill:color-mix(in srgb, var(--zone) 11%, transparent); stroke-opacity:.95;}
#plot .zone text{fill:var(--zone); font-weight:650; paint-order:stroke; stroke:var(--sheet); stroke-linejoin:round;}
#plot .node .t-z,#plot .centre .c-z{fill:var(--ink-soft);}
#plot .node.hl .dotc{stroke:var(--ink) !important;}
#plot .node.hl .t-nm{text-decoration:underline;}
#plot .centre .c-ring{fill:var(--sheet); stroke:var(--ink);}
#plot .centre .c-dot{fill:var(--ink);}
#plot .centre .c-nm{fill:var(--ink); font-weight:720;}
#plot .pulse{fill:var(--ink); stroke:var(--sheet);}

@media (max-width: 920px){
  .layout{grid-template-columns:1fr;}
  .plot-panel{border-right:0; padding:12px 16px 16px;}
  .list-panel{position:static; max-height:none; border-top:1px solid var(--grid);}
  .plot-wrap{max-width:100%;}
  .top{padding:14px 16px 12px;}
  .select-wrap select{font-size:30px;}
  .rf{padding:12px 16px 20px;}
  .console,.fof{grid-template-columns:1fr;}
  .cv-panel{padding:16px;}
  .fof-main,.fof-side{padding:16px;}
  .fof-side{border-left:0; border-top:1px solid var(--grid);}
  .console > div{padding:16px;}
  .console .tx{border-right:0; border-bottom:1px solid var(--grid);}
}
@media (prefers-reduced-motion: reduce){ .row .meter i{transition:none;} }
</style>
</head>
<body>

<header class="top">
  <div class="pick">
    <label for="centre">Centre node</label>
    <span class="select-wrap"><select id="centre" disabled><option>Loading</option></select></span>
  </div>
  <div class="status" id="status" data-state="wait" role="status">
    <span class="dot"></span><span id="status-text">Connecting to the dashboard server</span>
  </div>
</header>

<main class="layout">
  <section class="plot-panel" aria-label="Distance plot">
    <div class="tools">
      <div class="seg" role="group" aria-label="Distance scale">
        <button type="button" data-scale="linear">Linear</button>
        <button type="button" data-scale="log">Log</button>
      </div>
      <label class="thresh">Linked at
        <input type="range" id="thresh" min="10" max="95" step="5" value="50" aria-describedby="thresh-help">
        <output id="thresh-out" for="thresh">50%</output>
      </label>
      <span class="legend" id="thresh-help">
        <span>Predicted delivery</span><span class="ramp" aria-hidden="true"></span><span>0 to 100%</span>
        <span><span class="dash" aria-hidden="true"></span> not linked</span>
      </span>
      <span class="fof-legend" id="fof-legend" hidden>
        <svg viewBox="-9 -9 18 18" aria-hidden="true"><circle r="6.5" fill="none" stroke="var(--friend)" stroke-width="2"/></svg><span>friendly</span>
        <svg viewBox="-9 -9 18 18" aria-hidden="true"><path d="M0 -8L8 0L0 8L-8 0Z" fill="none" stroke="var(--foe)" stroke-width="2"/></svg><span>not friendly</span>
        <svg viewBox="-9 -9 18 18" aria-hidden="true"><path d="M0 -7.5L7.2 5.2L-7.2 5.2Z" fill="none" stroke="var(--sensing)" stroke-width="2"/></svg><span>sensing issue</span>
        <svg viewBox="-9 -9 18 18" aria-hidden="true"><circle r="6.5" fill="none" stroke="var(--ink-soft)" stroke-width="2" stroke-dasharray="3 3"/></svg><span>silent</span>
        <svg viewBox="-9 -9 18 18" aria-hidden="true"><rect x="-8" y="-6" width="16" height="12" fill="none" stroke="var(--zone)" stroke-width="2"/></svg><span>zone</span>
      </span>
    </div>

    <div class="plot-wrap">
      <svg id="plot" viewBox="0 0 1000 1000" role="img" aria-label="Top-down map of the endpoints around the centre node">
        <defs><clipPath id="disc-clip"><circle cx="500" cy="500" r="440"/></clipPath></defs>
        <circle class="disc" cx="500" cy="500" r="440"/>
        <g clip-path="url(#disc-clip)">
          <circle id="cover" class="cover" cx="500" cy="500" r="0"/>
          <circle id="maxring" class="maxring" cx="500" cy="500" r="0"/>
        </g>
        <g id="g-grid"></g>
        <g id="g-zones" clip-path="url(#disc-clip)"></g>
        <text id="cover-lbl" class="cover-lbl"></text>
        <text id="max-lbl" class="max-lbl"></text>
        <g id="g-links"></g>
        <g id="g-pulses"></g>
        <g id="g-nodes"></g>
        <g id="centre-g" class="centre" visibility="hidden">
          <circle class="c-ring" cx="500" cy="500"/>
          <circle class="c-dot" cx="500" cy="500"/>
          <text class="c-nm" x="500" text-anchor="middle"></text>
          <text class="c-z" x="500" text-anchor="middle"></text>
        </g>
      </svg>
      <div class="plot-msg" id="plot-msg" hidden></div>
    </div>
    <p class="hint">Click a node, here or in the list, to make it the centre. Top-down map: each endpoint sits at
      its real x, y offset from the centre, +X to the right, and the rings measure horizontal distance. Height is not
      drawn; each label gives the horizontal distance, then z and the difference from the centre (↑ above, ↓ below).
      Link quality still uses the straight-line 3D distance, as RFComms does. Violet rectangles are the zones from
      friend_or_foe.py; a solid one is where a drone heard by the centre node should be now.</p>
  </section>

  <aside class="list-panel" aria-label="Endpoints">
    <h2><span id="linked-title">Linked</span><span class="n" id="linked-n"></span></h2>
    <p class="note" id="traffic-note" hidden></p>
    <ul id="linked"></ul>
    <h2><span>Not linked</span><span class="n" id="out-n"></span></h2>
    <ul id="out"></ul>
  </aside>
</main>

<section class="cv-panel" aria-label="Computer vision">
  <div class="cv-head">
    <h2>Computer vision</h2>
    <div class="status" id="cv-status" data-state="wait" role="status">
      <span class="dot"></span><span id="cv-status-text">Waiting for the camera</span>
    </div>
  </div>
  <p class="help">The newest frame from your computer-vision program. It sends each frame (JPEG, PNG or WebP bytes)
    as the body of a POST to <code>http://localhost:8765/cv</code>.</p>
  <div class="cv-view">
    <img id="cv-image" alt="Latest computer-vision frame" hidden>
    <div class="cv-wait" id="cv-wait">
      <strong>No camera frames yet</strong>
      <span>Start your computer-vision program; frames appear here as soon as it sends them to
        <code>POST /cv</code>.</span>
    </div>
    <span class="cv-badge" id="cv-badge" hidden></span>
  </div>
</section>

<section class="fof" id="fof" aria-label="Friend or foe">
  <div class="fof-main">
    <div class="fof-head">
      <h2>Friend or foe</h2>
      <div class="fof-counts" id="fof-counts"></div>
    </div>
    <p class="help" id="fof-sub"></p>
    <div class="log-wrap fof-wrap">
      <table class="log">
        <thead><tr><th>Sender</th><th>Remote ID</th><th>Signature</th><th>Verdict</th><th>Zone</th><th class="num">Heard</th>
          <th class="num">Msgs</th><th>Reason</th></tr></thead>
        <tbody id="fof-rows"></tbody>
      </table>
    </div>
  </div>
  <aside class="fof-side">
    <h3>Recent changes</h3>
    <ul class="fof-events" id="fof-events"></ul>
    <h3>Registry</h3>
    <ul class="fof-registry" id="fof-registry"></ul>
    <h3>Zones</h3>
    <ul class="fof-zones" id="fof-zones"></ul>
  </aside>
</section>
<p class="fof-off" id="fof-off" hidden>Friend or foe is off: friend_or_foe.py was not found next to rf_dashboard.py.
  Put both files in the same folder and start the dashboard again.</p>

<section class="console" aria-label="Radio console">
  <div class="tx" data-mode="once">
    <h2>Transmit</h2>
    <form id="tx-form" novalidate>
      <div class="field">
        <label for="tx-src">From</label>
        <select id="tx-src" disabled><option>Loading</option></select>
      </div>
      <fieldset class="field">
        <legend>To</legend>
        <div class="chips" id="tx-dst"></div>
      </fieldset>
      <div class="field">
        <label for="tx-text">Message</label>
        <input id="tx-text" type="text" autocomplete="off" spellcheck="false" placeholder="UA-001_dc:a6:32:d1:b7:01_{x},{y},{z}_{t}">
        <p class="help">Fills in per message: <code>{src}</code> sender, <code>{dst}</code> receiver, <code>{seq}</code> number,
          <code>{t}</code> time, <code>{x}</code> <code>{y}</code> <code>{z}</code> sender’s position. Formats work too, e.g. <code>{seq:05d}</code>.</p>
      </div>
      <div class="field-row">
        <label class="check"><input type="checkbox" id="tx-header"> Add ID and SEQ header</label>
        <label class="inline">Pad to <input type="number" id="tx-pad" min="0" max="60000" step="1" inputmode="numeric"> bytes</label>
      </div>
      <div class="field-row">
        <div class="seg mode" role="group" aria-label="How to send">
          <button type="button" data-mode="once">Send once</button>
          <button type="button" data-mode="beacon">Beacon</button>
        </div>
        <label class="inline beacon-only">Rate <input type="number" id="tx-rate" min="0.05" max="200" step="any" inputmode="decimal"> per second</label>
        <label class="inline beacon-only">Stop after <input type="number" id="tx-count" min="0" step="1" inputmode="numeric" placeholder="never"> sends</label>
      </div>
      <p class="load" id="tx-load"></p>
      <div class="actions">
        <button type="submit" class="primary" id="tx-go">Send</button>
        <span class="feedback" id="tx-feedback" role="status"></span>
      </div>
    </form>

    <h3><span>Running beacons <span class="n" id="beacons-n"></span></span>
      <button type="button" class="btn" id="stop-all" hidden>Stop all</button></h3>
    <p class="help" id="beacons-empty">None. Start one above; it keeps running here until you stop it or close rf_dashboard.py.</p>
    <ul class="beacons" id="beacons"></ul>
  </div>

  <div class="rx">
    <h2>Listen</h2>
    <fieldset class="field">
      <legend>Receivers</legend>
      <div class="chips" id="rx-dst"></div>
    </fieldset>
    <div class="field-row">
      <label class="inline">From <select id="rx-src"><option value="">anyone</option></select></label>
      <label class="inline grow">Containing <input id="rx-filter" type="search" autocomplete="off" placeholder="any text"></label>
    </div>
    <div class="field-row">
      <button type="button" class="btn" id="rx-pause" aria-pressed="false">Pause</button>
      <button type="button" class="btn" id="rx-clear">Clear</button>
      <button type="button" class="btn" id="rx-csv">Export CSV</button>
      <span class="rx-stat" id="rx-stat" role="status"></span>
    </div>
    <div class="log-wrap">
      <table class="log">
        <thead><tr><th>Time</th><th>From</th><th>To</th><th>Message</th><th class="num">Bytes</th><th class="num">RSSI</th><th class="num">3D distance</th></tr></thead>
        <tbody id="rx-log"></tbody>
      </table>
    </div>
    <p class="help">Only messages RFComms delivered show up. A dropped message never reaches its receiver, so the gaps in
      SEQ numbers are the losses. RSSI is the value Gazebo attached; 3D distance is the straight-line distance when it arrived.</p>
  </div>
</section>

<footer class="rf" id="rf"></footer>

<script>
(() => {
"use strict";

const NS = "http://www.w3.org/2000/svg";
const C = 500, R = 440;                       // plot centre and radius in viewBox units
const $ = (s) => document.querySelector(s);
const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

const svg = $("#plot");
const G = { grid: $("#g-grid"), zones: $("#g-zones"), links: $("#g-links"), pulses: $("#g-pulses"), nodes: $("#g-nodes") };
const sel = $("#centre");

// ---------------------------------------------------------------- small utils
function svgEl(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function setA(e, attrs) { for (const k in attrs) e.setAttribute(k, attrs[k]); }
function esc(s) { return String(s).replace(/[&<>"']/g, (c) => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c])); }
const store = {
  get(k, d) { try { const v = localStorage.getItem("rfdash:" + k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem("rfdash:" + k, JSON.stringify(v)); } catch (e) {} },
};
function trimNum(v) { return String(+v.toPrecision(3)); }
function fmtDist(d) {
  if (d < 10) return [d.toFixed(1), "m"];
  if (d < 1000) return [String(Math.round(d)), "m"];
  return [(d / 1000).toFixed(d < 10000 ? 2 : 1), "km"];
}
function fmtDistStr(d) { const [v, u] = fmtDist(d); return v + " " + u; }
function fmtRing(v) { return v < 1000 ? trimNum(v) + " m" : trimNum(v / 1000) + " km"; }
function zText(z, dz) {
  const rel = Math.abs(dz) < 0.05 ? "level" : (dz > 0 ? "\u2191" : "\u2193") + Math.abs(dz).toFixed(1);
  return "z " + z.toFixed(1) + " m (" + rel + ")";
}
function fmtDbm(v) { return (v < 0 ? "\u2212" : "") + Math.abs(v).toFixed(0) + " dBm"; }
function fmtPct(p) {
  if (p <= 0) return "0%";
  if (p < 0.005) return "under 1%";
  if (p > 0.995 && p < 0.9999) return "over 99%";
  return Math.round(p * 100) + "%";
}

// ---------------------------------------------------------------- RF model (same maths as gz-sim RFComms)
function erfc(x) {                              // Numerical Recipes erfcc, error < 1.2e-7
  const z = Math.abs(x), t = 1 / (1 + 0.5 * z);
  const r = t * Math.exp(-z * z - 1.26551223 + t * (1.00002368 + t * (0.37409196 + t * (0.09678418 +
    t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587 + t * (-0.82215223 + t * 0.17087277)))))))));
  return x >= 0 ? r : 2 - r;
}
const FADE = (() => {                           // N(0,1) sampled from -6 to 6 sigma
  const pts = []; let sum = 0;
  for (let i = -60; i <= 60; i++) { const z = i / 10, w = Math.exp(-z * z / 2); pts.push([z, w]); sum += w; }
  return pts.map(([z, w]) => [z, w / sum]);
})();
function meanRssi(d, rf) { return rf.tx_power - (rf.l0 + 10 * rf.fading_exponent * Math.log10(Math.max(d, 0.01))); }
function okProb(power, rf, bytes) {
  const ber = erfc(Math.sqrt(Math.pow(10, (power - rf.noise_floor) / 10)));
  return ber >= 1 ? 0 : Math.exp(bytes * Math.log1p(-ber));
}
function pDeliver(d, rf, bytes) {
  if (rf.max_range > 0 && d > rf.max_range) return 0;
  const m = meanRssi(d, rf);
  if (!(rf.sigma > 0)) return okProb(m, rf, bytes);
  let acc = 0;
  for (const [z, w] of FADE) acc += w * okProb(m + rf.sigma * z, rf, bytes);
  return acc;
}
let coverCache = { key: "", r: 0 };
function coverageRadius(rf, bytes, thr) {      // distance where predicted delivery drops to the threshold
  const key = JSON.stringify([rf, bytes, thr]);
  if (coverCache.key === key) return coverCache.r;
  const hi = rf.max_range > 0 ? rf.max_range : 1e7;
  let r;
  if (pDeliver(hi * 0.999999, rf, bytes) >= thr) r = hi;
  else if (pDeliver(0.01, rf, bytes) < thr) r = 0;
  else {
    let lo = Math.log(0.01), up = Math.log(hi);
    for (let i = 0; i < 48; i++) { const mid = (lo + up) / 2; if (pDeliver(Math.exp(mid), rf, bytes) >= thr) lo = mid; else up = mid; }
    r = Math.exp(lo);
  }
  coverCache = { key, r };
  return r;
}

// ---------------------------------------------------------------- colours
const palette = {};
function readPalette() {
  const cs = getComputedStyle(document.documentElement);
  for (const k of ["good", "fair", "mid", "bad"]) palette[k] = cs.getPropertyValue("--" + k).trim();
}
function rgb(h) { const n = parseInt(h.slice(1), 16); return [n >> 16 & 255, n >> 8 & 255, n & 255]; }
function mix(a, b, t) { const A = rgb(a), B = rgb(b); return "rgb(" + A.map((v, i) => Math.round(v + (B[i] - v) * t)).join(",") + ")"; }
function qColour(p) {
  if (p >= 0.75) return mix(palette.fair, palette.good, (p - 0.75) / 0.25);
  if (p >= 0.5) return mix(palette.mid, palette.fair, (p - 0.5) / 0.25);
  return mix(palette.bad, palette.mid, Math.max(0, p) / 0.5);
}
readPalette();
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { readPalette(); gridKey = ""; refresh(); });

// ---------------------------------------------------------------- state
const ui = { centre: store.get("centre", null), scale: store.get("scale", "linear"), thr: store.get("thr", 0.5), hl: null };
let snap = null, lastMsgAt = 0, sseOk = false, activeCentre = null;
let model = { centre: null, centreEp: null, others: [], unknown: [], rf: null, bytes: 32, cover: 0, heardAny: false };
const counts = new Map();
const view = { max: 50, target: 50 };
const disp = new Map();                         // name -> displayed {x, y}
let fontK = 1.6;                                // viewBox units per CSS pixel

// ---------------------------------------------------------------- scale
function niceCeil(x) {
  const e = Math.pow(10, Math.floor(Math.log10(x)));
  for (const m of [1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10]) if (m * e >= x * 0.9999) return m * e;
  return 10 * e;
}
function isNice(v) {
  const e = Math.pow(10, Math.floor(Math.log10(v) + 1e-9)), m = v / e;
  return [1, 2, 2.5, 5, 10].some((n) => Math.abs(m - n) < 1e-6);
}
function radiusFor(d) {
  if (ui.scale === "log") return R * Math.log10(1 + d) / Math.log10(1 + view.max);
  return R * d / view.max;
}
function retarget(force) {
  const far = model.others.reduce((m, o) => Math.max(m, o.dh), 0);
  let want = niceCeil(Math.max(5, far * 1.12));
  if (ui.scale === "log") want = Math.max(10, want);
  if (force || want > view.target || want < view.target * 0.45) view.target = want;
  if (force && reduceMotion) view.max = view.target;
}

// ---------------------------------------------------------------- model from a snapshot
function computeModel() {
  if (!snap) return;
  const rf = snap.rf, bytes = rf.packet_bytes || 32;
  const byName = new Map(snap.endpoints.map((e) => [e.name, e]));
  const c = byName.get(activeCentre) || null;
  const links = new Map(snap.links.map((L) => [L.src + ">" + L.dst, L]));
  const others = [], unknown = [];
  let heardAny = false;
  // friend or foe, as checked by the centre node (sender -> its track)
  const fof = snap.fof && snap.fof.available ? snap.fof : null;
  const fofHeard = new Map();
  if (fof && fof.listeners[activeCentre]) for (const t of fof.listeners[activeCentre].tracks) fofHeard.set(t.sender, t);
  for (const e of snap.endpoints) {
    if (e.name === activeCentre) continue;
    const out = links.get(activeCentre + ">" + e.name) || null;
    const inn = links.get(e.name + ">" + activeCentre) || null;
    if ((out && out.age < 10) || (inn && inn.age < 10)) heardAny = true;
    const heard = fofHeard.get(e.name) || null;
    if (!c || !c.pos || !e.pos) { unknown.push({ name: e.name, model: e.model, out, inn, fof: heard }); continue; }
    const dx = e.pos[0] - c.pos[0], dy = e.pos[1] - c.pos[1], dz = e.pos[2] - c.pos[2];
    const d = Math.hypot(dx, dy, dz);          // straight-line: what RFComms uses
    const dh = Math.hypot(dx, dy);             // horizontal: what the map shows
    const p = pDeliver(d, rf, bytes);
    others.push({ name: e.name, model: e.model, fof: heard, d, dh, dx, dy, dz, z: e.pos[2], ang: Math.atan2(dy, dx), p,
      rssi: meanRssi(d, rf), linked: p >= ui.thr, beyond: rf.max_range > 0 && d > rf.max_range, out, inn });
  }
  others.sort((a, b) => a.dh - b.dh);
  model = { centre: activeCentre, centreEp: c, others, unknown, rf, bytes, cover: coverageRadius(rf, bytes, ui.thr), heardAny, fof };
}

// ---------------------------------------------------------------- centre selection
function syncSelect(names) {
  const key = names.join("\n");
  if (sel.dataset.key === key) return;
  sel.dataset.key = key;
  sel.textContent = "";
  for (const n of names) { const o = document.createElement("option"); o.value = n; o.textContent = n; sel.appendChild(o); }
  if (!names.length) { const o = document.createElement("option"); o.value = ""; o.textContent = "No endpoints"; sel.appendChild(o); }
  sel.disabled = !names.length;
}
function switchCentre(name) {
  if (name === activeCentre) return;
  if (activeCentre) disp.set(activeCentre, { x: C, y: C });   // old centre slides out from the middle
  disp.delete(name);
  activeCentre = name;
  if (name) sel.value = name;
}
function pickCentre(names) {
  let want = activeCentre;
  if (ui.centre && names.includes(ui.centre)) want = ui.centre;
  else if (!want || !names.includes(want)) want = names[0] || null;
  if (want !== activeCentre) { switchCentre(want); return true; }
  if (want) sel.value = want;
  return false;
}
function setCentre(name) {
  if (!name) return;
  ui.centre = name; store.set("centre", name);
  switchCentre(name);
  refresh(true);
}
sel.addEventListener("change", () => setCentre(sel.value));

// ---------------------------------------------------------------- controls
function syncScaleButtons() {
  for (const b of document.querySelectorAll(".seg button[data-scale]")) b.setAttribute("aria-pressed", String(b.dataset.scale === ui.scale));
}
for (const b of document.querySelectorAll(".seg button[data-scale]")) {
  b.addEventListener("click", () => { ui.scale = b.dataset.scale; store.set("scale", ui.scale); syncScaleButtons(); gridKey = ""; retarget(true); });
}
syncScaleButtons();
const thr = $("#thresh"), thrOut = $("#thresh-out");
thr.value = Math.round(ui.thr * 100); thrOut.textContent = thr.value + "%";
thr.addEventListener("input", () => {
  ui.thr = +thr.value / 100; store.set("thr", ui.thr); thrOut.textContent = thr.value + "%"; refresh();
});

// ---------------------------------------------------------------- live data
function onSnapshot(s) {
  snap = s; lastMsgAt = performance.now(); sseOk = true;
  const names = s.endpoints.map((e) => e.name);
  syncSelect(names);
  const changed = pickCentre(names);
  for (const L of s.links) {                     // a count that went up = messages actually received
    const key = L.src + ">" + L.dst, prev = counts.get(key);
    if (prev !== undefined && L.count > prev && activeCentre && (L.src === activeCentre || L.dst === activeCentre))
      spawnPulse(L.src, L.dst, Math.min(3, L.count - prev));
    counts.set(key, L.count);
  }
  refresh(changed);
  consoleUpdate(s);
  cvUpdate(s);
}
function refresh(forceScale) {
  computeModel();
  retarget(!!forceScale);
  renderList(); renderStatus(); renderPlotMsg(); renderFooter(); renderFof();
}
function connect() {
  const es = new EventSource("/events");
  es.onmessage = (e) => { try { onSnapshot(JSON.parse(e.data)); } catch (err) { console.error(err); } };
  es.onerror = () => { sseOk = false; renderStatus(); renderPlotMsg(); };
}
setInterval(() => {
  if (sseOk && performance.now() - lastMsgAt > 3000) { sseOk = false; renderStatus(); renderPlotMsg(); }
}, 1000);

// ---------------------------------------------------------------- status and messages
function renderStatus() {
  const st = $("#status"), tx = $("#status-text");
  if (!sseOk) { st.dataset.state = "down"; tx.textContent = "No contact with rf_dashboard.py"; return; }
  if (snap.source === "demo") { st.dataset.state = "demo"; tx.textContent = "Demo data, not connected to Gazebo"; return; }
  if (!snap.gazebo) { st.dataset.state = "wait"; tx.textContent = "Waiting for Gazebo"; return; }
  st.dataset.state = "live"; tx.textContent = "Live from world \u201c" + snap.world + "\u201d";
}
function renderPlotMsg() {
  const m = $("#plot-msg"); let html = "", low = false;
  if (!sseOk) html = "<strong>No contact with the dashboard server</strong>Start it again with <code>python3 rf_dashboard.py</code>. This page reconnects by itself.";
  else if (!snap.endpoints.length && snap.source !== "demo" && !snap.gazebo) html = "<strong>Waiting for Gazebo</strong>Start the simulation, for example <code>make px4_sitl gz_x500</code>. This page fills in by itself.";
  else if (!snap.endpoints.length) html = "<strong>No radio endpoints found</strong>Give each model a CommsEndpoint plugin with <code>&lt;topic&gt;/NAME/rx&lt;/topic&gt;</code>.";
  else if (!model.centreEp || !model.centreEp.pos) html = "<strong>Waiting for the position of " + esc(activeCentre) + "</strong>If it never appears, check that the model name matches its address.";
  else if (snap.endpoints.length === 1) { html = "<strong>" + esc(activeCentre) + " is the only endpoint</strong>Add another model with a CommsEndpoint to see a link."; low = true; }
  m.hidden = !html; m.innerHTML = html; m.classList.toggle("low", low);
}
let footKey = "";
function renderFooter() {
  const rf = snap.rf;
  const items = [
    ["Transmit power", (rf.tx_power < 0 ? "\u2212" : "") + trimNum(Math.abs(rf.tx_power)) + " dBm"],
    ["Path loss at 1 m", trimNum(rf.l0) + " dB"],
    ["Path-loss exponent", trimNum(rf.fading_exponent)],
    ["Fading spread", trimNum(rf.sigma) + " dB"],
    ["Noise floor", fmtDbm(rf.noise_floor)],
    ["Max range", rf.max_range > 0 ? fmtDistStr(rf.max_range) : "none"],
    ["Packet size in predictions", rf.packet_bytes + " bytes"],
  ];
  let html = items.map(([k, v]) => "<span>" + esc(k) + " <b>" + esc(v) + "</b></span>").join("");
  html += "<span class=\"src\">" + esc(snap.rf_note || ("Radio settings read from " + snap.rf_path)) + "</span>";
  if (html !== footKey) { footKey = html; $("#rf").innerHTML = html; }
}

// ---------------------------------------------------------------- list
const rows = new Map();
function ensureRow(name) {
  let r = rows.get(name);
  if (r) return r;
  const li = document.createElement("li"); li.dataset.name = name;
  const b = document.createElement("button"); b.type = "button"; b.className = "row";
  b.innerHTML = '<span class="nm"></span><span class="dist"></span><span class="meta"></span><span class="fofline" hidden></span>' +
    '<span class="meter" aria-hidden="true"><i></i><b></b></span><span class="pred"></span>' +
    '<span class="heard h-out" hidden></span><span class="heard h-in" hidden></span>';
  li.appendChild(b);
  b.addEventListener("click", () => setCentre(name));
  b.addEventListener("pointerenter", () => setHl(name));
  b.addEventListener("pointerleave", () => setHl(null));
  b.addEventListener("focus", () => setHl(name));
  b.addEventListener("blur", () => setHl(null));
  r = { li, b, nm: b.querySelector(".nm"), dist: b.querySelector(".dist"), meta: b.querySelector(".meta"),
    fofline: b.querySelector(".fofline"),
    bar: b.querySelector(".meter i"), tick: b.querySelector(".meter b"), pred: b.querySelector(".pred"),
    hout: b.querySelector(".h-out"), hin: b.querySelector(".h-in") };
  r.nm.textContent = name;
  rows.set(name, r);
  return r;
}
function trafficLine(elm, L, from, to) {
  if (!L) { elm.hidden = true; return; }
  elm.hidden = false;
  if (L.age > 5) {
    elm.classList.add("quiet");
    elm.textContent = from + " \u2192 " + to + ": nothing received for " + Math.round(L.age) + " s";
    return;
  }
  elm.classList.remove("quiet");
  const parts = [L.rate.toFixed(1) + "/s"];
  if (L.delivered !== null && L.delivered !== undefined) parts.push(Math.round(L.delivered) + "% delivered");
  if (L.rssi !== null && L.rssi !== undefined) parts.push(fmtDbm(L.rssi));
  elm.textContent = from + " \u2192 " + to + ": " + parts.join(", ");
}
function fillRow(r, o) {
  r.b.setAttribute("aria-label", o.name + ", " + (o.d !== undefined ? fmtDistStr(o.dh) + " from " + activeCentre + " horizontally" : "position unknown") + ". Make centre.");
  const meta = [];
  if (o.model && o.model !== o.name) meta.push("model " + o.model);
  if (o.z !== undefined) meta.push("z " + o.z.toFixed(1) + " m");
  if (o.dz !== undefined && Math.abs(o.dz) >= 0.05) meta.push(Math.abs(o.dz).toFixed(1) + " m " + (o.dz > 0 ? "higher" : "lower"));
  r.meta.textContent = meta.join(", ");
  if (o.d === undefined) {
    r.dist.textContent = "";
    r.bar.style.width = "0"; r.tick.style.left = (ui.thr * 100) + "%";
    r.pred.textContent = "Position unknown. Its model name should match its address.";
  } else {
    const [v, u] = fmtDist(o.dh);
    r.dist.innerHTML = esc(v) + "<small>" + u + "</small>";
    r.bar.style.width = (o.p * 100).toFixed(1) + "%";
    r.bar.style.background = qColour(o.p);
    r.tick.style.left = (ui.thr * 100) + "%";
    r.pred.textContent = o.beyond
      ? "Beyond the max range of " + fmtDistStr(model.rf.max_range) + " (" + fmtDistStr(o.d) + " straight-line)"
      : "Predicted " + fmtPct(o.p) + " delivery, about " + fmtDbm(o.rssi) + " over " + fmtDistStr(o.d) + " straight-line";
  }
  fofLine(r.fofline, o.fof);
  trafficLine(r.hout, o.out, activeCentre, o.name);
  trafficLine(r.hin, o.inn, o.name, activeCentre);
}
function place(ul, items) {
  ul.querySelectorAll("li.empty").forEach((x) => x.remove());
  const want = items.map((o) => o.name);
  const have = [...ul.children].map((li) => li.dataset.name);
  if (want.join("\n") !== have.join("\n")) for (const n of want) ul.appendChild(rows.get(n).li);
}
function emptyLi(ul, text) {
  if (ul.querySelector("li.empty")) { ul.querySelector("li.empty").textContent = text; return; }
  const li = document.createElement("li"); li.className = "empty"; li.textContent = text; ul.appendChild(li);
}
function renderList() {
  const linked = model.others.filter((o) => o.linked);
  const out = model.others.filter((o) => !o.linked).concat(model.unknown);
  const keep = new Set(linked.concat(out).map((o) => o.name));
  for (const [n, r] of rows) if (!keep.has(n)) { r.li.remove(); rows.delete(n); }
  for (const o of linked.concat(out)) fillRow(ensureRow(o.name), o);
  place($("#linked"), linked);
  place($("#out"), out);
  $("#linked-title").textContent = activeCentre ? "Linked to " + activeCentre : "Linked";
  $("#linked-n").textContent = linked.length ? String(linked.length) : "";
  $("#out-n").textContent = out.length ? String(out.length) : "";
  if (!linked.length) emptyLi($("#linked"), activeCentre && model.others.length
    ? "Nothing reaches the threshold. Move endpoints closer or lower the threshold."
    : "No other endpoints yet.");
  if (!out.length) emptyLi($("#out"), "Every endpoint is linked.");
  const note = $("#traffic-note");
  if (activeCentre && model.others.length + model.unknown.length && !model.heardAny) {
    note.hidden = false;
    note.innerHTML = "No messages to or from " + esc(activeCentre) + " yet. To see real traffic, run <code>python3 Rf.py beacon " + esc(activeCentre) + " all</code>";
  } else note.hidden = true;
  for (const [n, r] of rows) r.b.classList.toggle("hl", n === ui.hl);
}
function setHl(name) {
  ui.hl = name;
  for (const [n, r] of rows) r.b.classList.toggle("hl", n === name);
}

// ---------------------------------------------------------------- plot: grid
let gridKey = "";
function linearRings(max) {
  let major = max / 4;
  for (const n of [5, 4, 3, 6, 2]) if (isNice(max / n)) { major = max / n; break; }
  const minor = major / 5, out = [];
  const count = Math.round(max / minor);
  for (let i = 1; i <= count; i++) { const isMajor = i % 5 === 0; out.push({ v: i * minor, major: isMajor, label: isMajor }); }
  return out;
}
function logRings(max) {
  const out = [];
  for (let e = 0; Math.pow(10, e) <= max; e++)
    for (let m = 1; m <= 9; m++) { const v = m * Math.pow(10, e); if (v > max * 1.0001) break; out.push({ v, major: m === 1, label: m === 1 || m === 2 || m === 5 }); }
  if (!out.length || Math.abs(out[out.length - 1].v - max) > 1e-6) out.push({ v: max, major: true, label: true });
  return out;
}
function drawGrid() {
  const key = ui.scale + "|" + view.target + "|" + view.max.toPrecision(5) + "|" + fontK.toFixed(3);
  if (key === gridKey) return;
  gridKey = key;
  const k = fontK; let h = "";
  for (let a = 0; a < 360; a += 30) {
    const rad = a * Math.PI / 180, axis = a % 90 === 0;
    h += '<line class="' + (axis ? "axis" : "spoke") + '" x1="' + C + '" y1="' + C + '" x2="' + (C + R * Math.cos(rad)).toFixed(1) +
      '" y2="' + (C - R * Math.sin(rad)).toFixed(1) + '" stroke-width="' + ((axis ? 1 : 0.6) * k).toFixed(2) + '"/>';
  }
  const rings = ui.scale === "log" ? logRings(view.target) : linearRings(view.target);
  let lastR = 1e9, labels = "";
  for (let i = rings.length - 1; i >= 0; i--) {
    const g = rings[i], r = radiusFor(g.v);
    if (r > R + 0.5 || r < 2) continue;
    h += '<circle class="ring' + (g.major ? " major" : "") + '" cx="' + C + '" cy="' + C + '" r="' + r.toFixed(1) +
      '" stroke-width="' + ((g.major ? 1 : 0.5) * k).toFixed(2) + '"/>';
    if (g.label && lastR - r > 30 * k) {
      labels += '<text class="lbl" x="' + (C + 5 * k).toFixed(1) + '" y="' + (C - r - 4 * k).toFixed(1) + '" font-size="' + (11 * k).toFixed(1) +
        '" stroke-width="' + (3 * k).toFixed(1) + '">' + fmtRing(g.v) + "</text>";
      lastR = r;
    }
  }
  h += '<circle class="rim" cx="' + C + '" cy="' + C + '" r="' + R + '" stroke-width="' + (1.5 * k).toFixed(2) + '"/>';
  h += labels;
  h += '<text class="axis-lbl" x="' + (C + R + 8 * k).toFixed(1) + '" y="' + (C + 4 * k).toFixed(1) + '" font-size="' + (12 * k).toFixed(1) + '">+X</text>';
  h += '<text class="axis-lbl" x="' + (C - 6 * k).toFixed(1) + '" y="' + (C - R - 6 * k).toFixed(1) + '" font-size="' + (12 * k).toFixed(1) + '" text-anchor="end">+Y</text>';
  G.grid.innerHTML = h;
}

// ---------------------------------------------------------------- plot: coverage
const coverEl = $("#cover"), maxEl = $("#maxring"), coverLbl = $("#cover-lbl"), maxLbl = $("#max-lbl");
function drawCover() {
  const k = fontK, show = model.rf && model.centreEp && model.centreEp.pos;
  if (!show || model.cover <= 0) { coverEl.setAttribute("r", 0); coverLbl.textContent = ""; }
  else {
    const rr = radiusFor(model.cover), pct = Math.round(ui.thr * 100) + "%";
    setA(coverEl, { r: Math.min(rr, R + 4).toFixed(1), "stroke-width": (1.5 * k).toFixed(2) });
    setA(coverLbl, { "font-size": (12 * k).toFixed(1), "stroke-width": (3.5 * k).toFixed(1) });
    if (rr >= R) {
      setA(coverLbl, { x: C, y: (C + R - 22 * k).toFixed(1), "text-anchor": "middle" });
      coverLbl.textContent = "All of this view is within the " + pct + " range (" + fmtDistStr(model.cover) + ") at equal height";
    } else {
      const inside = rr > 60 * k;
      setA(coverLbl, { x: C, y: (inside ? C + rr - 10 * k : C + rr + 18 * k).toFixed(1), "text-anchor": "middle" });
      coverLbl.textContent = pct + " delivery range at equal height " + fmtDistStr(model.cover);
    }
  }
  const mr = model.rf && model.rf.max_range > 0 && show ? radiusFor(model.rf.max_range) : Infinity;
  if (mr <= R) {
    setA(maxEl, { r: mr.toFixed(1), "stroke-width": (1.4 * k).toFixed(2), "stroke-dasharray": (7 * k).toFixed(1) + " " + (5 * k).toFixed(1) });
    setA(maxLbl, { x: (C - 6 * k).toFixed(1), y: (C - mr + 18 * k).toFixed(1), "text-anchor": "end",
      "font-size": (12 * k).toFixed(1), "stroke-width": (3.5 * k).toFixed(1) });
    maxLbl.textContent = "Max range " + fmtDistStr(model.rf.max_range);
  } else { maxEl.setAttribute("r", 0); maxLbl.textContent = ""; }
}

// ---------------------------------------------------------------- plot: nodes and links
const nodes = new Map();
function ensureNode(name) {
  let n = nodes.get(name);
  if (n) return n;
  const line = svgEl("line", { class: "link" }, G.links);
  const g = svgEl("g", { class: "node", "data-name": name }, G.nodes);
  const hit = svgEl("circle", { class: "hit" }, g);
  const dot = svgEl("circle", { class: "dotc" }, g);
  const frame = svgEl("path", { class: "frame", d: "M0 0" }, g);      // friend-or-foe outline
  const t1 = svgEl("text", { class: "t-nm" }, g);
  t1.appendChild(document.createTextNode(name));
  const rid = svgEl("tspan", { class: "t-rid none" }, t1);           // remote ID after the name
  const out = svgEl("tspan", { class: "t-out" }, t1);                // "out of zone" after that
  const t2 = svgEl("text", { class: "t-d" }, g);
  const t3 = svgEl("text", { class: "t-z" }, g);
  n = { line, g, hit, dot, frame, t1, rid, out, t2, t3 };
  nodes.set(name, n);
  return n;
}
G.nodes.addEventListener("click", (e) => { const g = e.target.closest(".node"); if (g) setCentre(g.dataset.name); });
G.nodes.addEventListener("pointerover", (e) => { const g = e.target.closest(".node"); setHl(g ? g.dataset.name : null); });
G.nodes.addEventListener("pointerleave", () => setHl(null));

const centreG = $("#centre-g");
function drawNodes() {
  const k = fontK, seen = new Set(), has = model.centreEp && model.centreEp.pos, boxes = [];
  if (has) {
    const [ring, dot, nm, cz] = centreG.children;
    setA(ring, { r: (11 * k).toFixed(1), "stroke-width": (2 * k).toFixed(2) });
    setA(dot, { r: (4.5 * k).toFixed(1) });
    setA(nm, { y: (C + 30 * k).toFixed(1), "font-size": (15 * k).toFixed(1), "stroke-width": (4 * k).toFixed(1) });
    setA(cz, { y: (C + 45 * k).toFixed(1), "font-size": (12.5 * k).toFixed(1), "stroke-width": (4 * k).toFixed(1) });
    nm.textContent = model.centre;
    cz.textContent = "z " + model.centreEp.pos[2].toFixed(1) + " m";
    centreG.setAttribute("visibility", "visible");
    const cw = Math.max(nm.getComputedTextLength(), cz.getComputedTextLength());
    boxes.push({ fixed: true, x0: C - cw / 2, x1: C + cw / 2, y0: C + 17 * k, y1: C + 49 * k });
    for (const o of model.others) {
      const n = ensureNode(o.name); seen.add(o.name);
      const r = Math.min(radiusFor(o.dh), R * 1.03);
      const tx = C + r * Math.cos(o.ang), ty = C - r * Math.sin(o.ang);
      let p = disp.get(o.name);
      if (!p) { p = { x: tx, y: ty }; disp.set(o.name, p); }
      if (reduceMotion) { p.x = tx; p.y = ty; } else { p.x += (tx - p.x) * 0.2; p.y += (ty - p.y) * 0.2; }
      const col = qColour(o.p);
      setA(n.line, { x1: C, y1: C, x2: p.x.toFixed(1), y2: p.y.toFixed(1), stroke: col,
        "stroke-width": ((o.linked ? 2.4 : 1.4) * k).toFixed(2), "stroke-opacity": o.linked ? 0.9 : 0.6,
        "stroke-dasharray": o.linked ? "none" : (6 * k).toFixed(1) + " " + (6 * k).toFixed(1) });
      n.g.setAttribute("transform", "translate(" + p.x.toFixed(1) + " " + p.y.toFixed(1) + ")");
      setA(n.hit, { r: (18 * k).toFixed(1) });
      setA(n.dot, { r: (7 * k).toFixed(1), fill: o.linked ? col : "var(--sheet)", stroke: col, "stroke-width": (2.4 * k).toFixed(2) });
      drawFrame(n, o.fof, k);
      // put the label on the side facing away from the centre
      const ox = p.x - C, oy = p.y - C, len = Math.hypot(ox, oy) || 1, ux = ox / len, uy = oy / len;
      let anchor = "middle", nx = 0, ny;
      if (ux > 0.4) { anchor = "start"; nx = 13 * k; ny = -1 * k; }
      else if (ux < -0.4) { anchor = "end"; nx = -13 * k; ny = -1 * k; }
      else ny = uy < 0 ? -44 * k : 25 * k;
      setA(n.t1, { x: nx.toFixed(1), y: ny.toFixed(1), "text-anchor": anchor, "font-size": (14 * k).toFixed(1), "stroke-width": (4 * k).toFixed(1) });
      setA(n.t2, { x: nx.toFixed(1), y: (ny + 15 * k).toFixed(1), "text-anchor": anchor, "font-size": (12.5 * k).toFixed(1), "stroke-width": (4 * k).toFixed(1) });
      setA(n.t3, { x: nx.toFixed(1), y: (ny + 29 * k).toFixed(1), "text-anchor": anchor, "font-size": (12.5 * k).toFixed(1), "stroke-width": (4 * k).toFixed(1) });
      n.t2.textContent = fmtDistStr(o.dh);
      n.t3.textContent = zText(o.z, o.dz);
      if (anchor !== "middle") {
        const w = Math.max(n.t1.getComputedTextLength(), n.t2.getComputedTextLength(), n.t3.getComputedTextLength());
        if (anchor === "start" && p.x + nx + w > 995) { anchor = "end"; nx = -13 * k; }
        else if (anchor === "end" && p.x + nx - w < 5) { anchor = "start"; nx = 13 * k; }
        if (anchor !== n.t1.getAttribute("text-anchor")) {
          setA(n.t1, { x: nx.toFixed(1), "text-anchor": anchor });
          setA(n.t2, { x: nx.toFixed(1), "text-anchor": anchor });
          setA(n.t3, { x: nx.toFixed(1), "text-anchor": anchor });
        }
      }
      n.g.classList.toggle("hl", o.name === ui.hl);
      const w = Math.max(n.t1.getComputedTextLength(), n.t2.getComputedTextLength(), n.t3.getComputedTextLength());
      const ax = p.x + nx, x0 = anchor === "start" ? ax : anchor === "end" ? ax - w : ax - w / 2;
      boxes.push({ n, ny, x0, x1: x0 + w, y0: p.y + ny - 12 * k, y1: p.y + ny + 33 * k });
    }
    // nudge labels down until no two overlap (nearest to the centre keep their place)
    const movable = boxes.filter((b) => !b.fixed).sort((a, b) => a.y0 - b.y0);
    const placed = boxes.filter((b) => b.fixed);
    for (const b of movable) {
      let shift = 0;
      for (let guard = 0; guard < 12; guard++) {
        const hit = placed.find((q) => b.x0 < q.x1 + 4 * k && b.x1 > q.x0 - 4 * k && b.y0 + shift < q.y1 && b.y1 + shift > q.y0);
        if (!hit) break;
        shift += hit.y1 - (b.y0 + shift) + 2 * k;
      }
      b.y0 += shift; b.y1 += shift;
      placed.push(b);
      b.n.t1.setAttribute("y", (b.ny + shift).toFixed(1));
      b.n.t2.setAttribute("y", (b.ny + 15 * k + shift).toFixed(1));
      b.n.t3.setAttribute("y", (b.ny + 29 * k + shift).toFixed(1));
    }
  } else centreG.setAttribute("visibility", "hidden");
  for (const [name, n] of nodes) if (!seen.has(name)) { n.g.remove(); n.line.remove(); nodes.delete(name); }
}

// ---------------------------------------------------------------- plot: message pulses
const pulses = [];
function spawnPulse(src, dst, n) {
  if (reduceMotion) return;
  const t = performance.now();
  for (let i = 0; i < n; i++) pulses.push({ src, dst, t0: t + i * 140 });
  if (pulses.length > 90) pulses.splice(0, pulses.length - 90);
}
function posOf(name) { return name === model.centre ? { x: C, y: C } : disp.get(name) || null; }
function drawPulses(now) {
  let h = "";
  for (let i = pulses.length - 1; i >= 0; i--) {
    const P = pulses[i], t = (now - P.t0) / 700;
    if (t > 1) { pulses.splice(i, 1); continue; }
    if (t < 0) continue;
    const a = posOf(P.src), b = posOf(P.dst);
    if (!a || !b || !nodes.has(P.src === model.centre ? P.dst : P.src)) continue;
    const e = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
    h += '<circle class="pulse" cx="' + (a.x + (b.x - a.x) * e).toFixed(1) + '" cy="' + (a.y + (b.y - a.y) * e).toFixed(1) +
      '" r="' + (4 * fontK).toFixed(1) + '" stroke-width="' + (1.5 * fontK).toFixed(1) + '"/>';
  }
  G.pulses.innerHTML = h;
}



// ---------------------------------------------------------------- friend or foe
// Verdicts come from friend_or_foe.py running inside rf_dashboard.py, for the
// centre node: what it has heard from each sender and how that was judged.
function fofClass(t) {
  if (!t) return "none";
  if (t.status === "SILENT") return "silent";
  return { "FRIENDLY": "friend", "NOT FRIENDLY": "foe", "SENSING ISSUE": "sensing" }[t.status] || "foe";
}
function fofPill(t) {
  const c = fofClass(t);
  const text = c === "silent" ? "Silent" : c === "none" ? "Not heard" : t.label;
  return '<span class="pill ' + c + '">' + esc(text) + "</span>";
}
function zonePill(t) {
  if (!t || !t.zone_state) return "";
  return t.zone_state === "IN ZONE" ? '<span class="pill friend">In ' + esc(t.zone) + "</span>"
                                    : '<span class="pill foe">Out of zone</span>';
}
function fofLine(elm, t) {
  if (!model.fof) { elm.hidden = true; return; }
  elm.hidden = false;
  const c = fofClass(t);
  let text;
  if (c === "none") text = "no identity broadcast received by " + activeCentre;
  else if (c === "friend") text = t.zone_reason || t.remote_id;
  else if (c === "silent") text = "last: " + t.label + (t.zone_reason ? "; " + t.zone_reason : "");
  else text = t.reason;
  elm.innerHTML = fofPill(t) + zonePill(t) + " " + esc(text);
}
function drawFrame(n, t, k) {
  const c = model.fof ? fofClass(t) : "none";
  n.rid.setAttribute("class", "t-rid " + c);
  n.rid.textContent = t ? " " + (t.remote_id || "no ID") : "";
  n.out.textContent = t && t.zone_state === "OUT OF ZONE" ? " out of zone" : "";
  if (c === "none") { n.frame.setAttribute("d", "M0 0"); return; }
  const r = 12.5 * k, s = 15 * k, f = (v) => v.toFixed(1);
  let d;
  if (c === "foe") d = "M0 " + f(-s) + "L" + f(s) + " 0L0 " + f(s) + "L" + f(-s) + " 0Z";
  else if (c === "sensing") d = "M0 " + f(-s * 1.05) + "L" + f(s * 1.05) + " " + f(s * 0.75) + "L" + f(-s * 1.05) + " " + f(s * 0.75) + "Z";
  else d = "M" + f(-r) + " 0A" + f(r) + " " + f(r) + " 0 1 0 " + f(r) + " 0A" + f(r) + " " + f(r) + " 0 1 0 " + f(-r) + " 0";
  const colour = { friend: "var(--friend)", foe: "var(--foe)", sensing: "var(--sensing)" }[c] || "var(--ink-soft)";
  setA(n.frame, { d, stroke: colour, "stroke-width": (2.2 * k).toFixed(2),
    "stroke-dasharray": c === "silent" ? f(3 * k) + " " + f(3 * k) : "none" });
}

// zones: rectangles in world x, y, drawn around the centre node with the map's own scale
const zoneEls = new Map();
function drawZones() {
  const zones = model.fof ? model.fof.zones : null;
  const c = model.centreEp && model.centreEp.pos;
  const names = zones && c ? Object.keys(zones) : [];
  for (const [name, el] of zoneEls) if (!names.includes(name)) { el.g.remove(); zoneEls.delete(name); }
  if (!names.length) return;
  const k = fontK, steps = ui.scale === "log" ? 32 : 1;      // log scale bends straight edges
  const toScreen = (wx, wy) => {
    const dx = wx - c[0], dy = wy - c[1], dh = Math.hypot(dx, dy);
    if (dh < 1e-9) return [C, C];
    const r = radiusFor(dh), a = Math.atan2(dy, dx);
    return [C + r * Math.cos(a), C - r * Math.sin(a)];
  };
  const needed = new Map();          // zone -> remote IDs heard by the centre that should be in it now
  const L = model.fof.listeners[model.centre];
  if (L) for (const t of L.tracks) if (t.zone) {
    if (!needed.has(t.zone)) needed.set(t.zone, []);
    needed.get(t.zone).push(t.remote_id);
  }
  const placed = [];                 // label boxes already used, so labels don't overlap
  for (const name of names) {
    const [x0, x1, y0, y1] = zones[name];
    let el = zoneEls.get(name);
    if (!el) {
      const g = svgEl("g", { class: "zone" }, G.zones);
      el = { g, path: svgEl("path", {}, g), lbl: svgEl("text", {}, g) };
      zoneEls.set(name, el);
    }
    const corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
    let d = "", minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (let i = 0; i < 4; i++) {
      const [ax, ay] = corners[i], [bx, by] = corners[(i + 1) % 4];
      for (let s = 0; s < steps; s++) {
        const [px, py] = toScreen(ax + (bx - ax) * s / steps, ay + (by - ay) * s / steps);
        d += (d ? "L" : "M") + px.toFixed(1) + " " + py.toFixed(1);
        minX = Math.min(minX, px); maxX = Math.max(maxX, px); minY = Math.min(minY, py); maxY = Math.max(maxY, py);
      }
    }
    const isNeeded = needed.has(name);
    el.g.classList.toggle("needed", isNeeded);
    setA(el.path, { d: d + "Z", "stroke-width": ((isNeeded ? 1.8 : 1.2) * k).toFixed(2),
      "stroke-dasharray": isNeeded ? "none" : (7 * k).toFixed(1) + " " + (5 * k).toFixed(1) });
    // label at the top-left corner of the part of the zone inside the view
    const m = view.max * 0.68, lx0 = Math.max(x0, c[0] - m), ly1 = Math.min(y1, c[1] + m);
    const visible = lx0 < Math.min(x1, c[0] + m) && ly1 > Math.max(y0, c[1] - m);
    const [lx, ly] = toScreen(lx0, ly1);
    el.lbl.textContent = name + (isNeeded ? " · " + needed.get(name).join(", ") : "");
    setA(el.lbl, { "font-size": (12 * k).toFixed(1), "stroke-width": (3.5 * k).toFixed(1) });
    const w = el.lbl.getComputedTextLength(), h = 15 * k;
    let bx = lx + 6 * k, by = ly + 15 * k;
    for (let i = 0; i < 6 && placed.some((q) => bx < q[0] + q[2] && bx + w > q[0] && by - h < q[1] && by > q[1] - h); i++)
      by += h + 2 * k;
    setA(el.lbl, { x: bx.toFixed(1), y: by.toFixed(1) });
    // only label a zone that is drawn big enough to hold its label
    const fits = visible && maxX - minX > el.lbl.getComputedTextLength() + 14 * k && maxY - minY > 24 * k;
    el.lbl.setAttribute("visibility", fits ? "visible" : "hidden");
    if (fits) placed.push([bx, by, w, h]);
  }
}
let fofKey = "";
function renderFof() {
  const f = snap && snap.fof;
  $("#fof").hidden = !f || !f.available;
  $("#fof-off").hidden = !f || f.available;
  $("#fof-legend").hidden = !f || !f.available;
  if (!f || !f.available) return;

  const listener = activeCentre;
  const L = (listener && f.listeners[listener]) || { tracks: [], events: [] };
  const byName = new Map(L.tracks.map((t) => [t.sender, t]));
  const senders = new Set(L.tracks.map((t) => t.sender));
  for (const e of snap.endpoints) if (e.name !== listener) senders.add(e.name);
  const names = [...senders].sort();

  const count = { friend: 0, foe: 0, sensing: 0, silent: 0, none: 0 };
  let outOfZone = 0, rows = "";
  for (const name of names) {
    const t = byName.get(name), c = fofClass(t);
    count[c]++;
    if (!t) {
      rows += '<tr class="dim"><td>' + esc(name) + "</td><td>-</td><td>-</td><td>" + fofPill(null) +
        '</td><td>-</td><td class="num"></td><td class="num">0</td><td class="reason">no identity broadcast received by ' +
        esc(listener) + "</td></tr>";
      continue;
    }
    if (t.zone_state === "OUT OF ZONE") outOfZone++;
    let reason = c === "silent" ? "nothing for " + Math.round(t.age) + " s; last: " + t.reason : t.reason;
    if (t.zone_state === "OUT OF ZONE" || t.zone_reason === "position unknown") reason += "; " + t.zone_reason;
    rows += "<tr><td>" + esc(t.sender) + "</td><td>" + esc(t.remote_id || "-") + "</td><td>" + esc(t.sig_mac || "-") +
      "</td><td>" + fofPill(t) + "</td><td>" + (zonePill(t) || "-") + '</td><td class="num">' + t.age.toFixed(1) +
      ' s</td><td class="num">' + t.messages + '</td><td class="reason">' + esc(reason) + "</td></tr>";
  }
  if (!names.length) rows = '<tr class="none"><td colspan="8">No other endpoints yet.</td></tr>';

  const counts = [["friend", "friendly"], ["foe", "not friendly"], ["sensing", "sensing issue"], ["silent", "silent"],
                  ["none", "not heard"]]
    .filter(([c]) => count[c]).map(([c, w]) => '<span class="pill ' + c + '">' + count[c] + " " + w + "</span>").join("") +
    (outOfZone ? '<span class="pill foe">' + outOfZone + " out of zone</span>" : "");

  const users = new Map();          // remote ID -> senders currently accepted as that drone
  for (const t of L.tracks) if (t.code === "FRIENDLY" && t.remote_id) {
    if (!users.has(t.remote_id)) users.set(t.remote_id, []);
    users.get(t.remote_id).push(t);
  }
  const registry = f.registry.map((r) => {
    const who = users.get(r.remote_id) || [];
    const out = who.some((t) => t.zone_state === "OUT OF ZONE");
    const whoText = who.map((t) => t.sender + (t.status === "SILENT" ? " (silent)" : "") +
      (t.zone_state === "OUT OF ZONE" ? " (out of zone)" : "")).join(", ");
    const plan = Object.entries(r.schedule).map(([frame, zone]) => frame + " " + zone).join(", ");
    const now = r.zone_now ? "Now in <b>" + esc(r.zone_now) + "</b> (" + esc(r.frame_now) + ")" : "Now: no zone, must not fly";
    return '<li title="' + esc(plan) + '"><span class="rid">' + esc(r.remote_id) + '</span><span class="mac">' +
      esc(r.signature) + '</span><span class="who' + (who.length ? (out ? " foe" : " friend") : "") + '">' +
      (who.length ? esc(whoText) : "not heard") + '</span><span class="now">' + now + "</span></li>";
  }).join("");
  const fmtM = (v) => (v < 0 ? "−" : "") + Math.abs(v);
  const zoneList = Object.entries(f.zones).map(([name, [x0, x1, y0, y1]]) =>
    '<li><span class="zn">' + esc(name) + '</span><span class="zb">x ' + fmtM(x0) + " to " + fmtM(x1) + ", y " +
    fmtM(y0) + " to " + fmtM(y1) + " m</span></li>").join("");

  const STATUS_CLASS = { "NOT FRIENDLY": "foe", "FRIENDLY": "friend", "SENSING ISSUE": "sensing", "SILENT": "silent",
                         "OUT OF ZONE": "foe", "IN ZONE": "friend" };
  const events = L.events.slice().reverse().map((line) => {
    const tm = line.slice(0, 8);
    const rest = esc(line.slice(8).trim()).replace(/NOT FRIENDLY|FRIENDLY|SENSING ISSUE|SILENT|OUT OF ZONE|IN ZONE/g,
      (w) => '<span class="' + STATUS_CLASS[w] + '">' + w + "</span>");
    return '<li><span class="tm">' + esc(tm) + "</span>" + rest + "</li>";
  }).join("") || "<li>Nothing yet.</li>";

  const sub = listener
    ? "Checked by " + listener + ", the centre node: every identity broadcast it receives is compared with the " +
      "registry, and friendly drones with their zone for this time of day. Pick another centre node to see what " +
      "that drone has verified. Hover a registry entry for its full schedule."
    : "";
  const key = [rows, counts, registry, zoneList, events, sub].join("|");
  if (key === fofKey) return;
  fofKey = key;
  $("#fof-rows").innerHTML = rows;
  $("#fof-counts").innerHTML = counts;
  $("#fof-registry").innerHTML = registry;
  $("#fof-zones").innerHTML = zoneList;
  $("#fof-events").innerHTML = events;
  $("#fof-sub").textContent = sub;
}

// ---------------------------------------------------------------- radio console: transmit
const tx = Object.assign({ src: null, dst: [], all: true, text: "", header: true, pad: 0, mode: "once", rate: 5, count: "" },
  store.get("tx", {}));
const rx = Object.assign({ all: true, dst: [], src: "", filter: "" }, store.get("rx", {}));
const saveTx = () => store.set("tx", tx);
const saveRx = () => store.set("rx", rx);
const txForm = $("#tx-form"), txSrc = $("#tx-src"), txDst = $("#tx-dst"), txText = $("#tx-text"), txHeader = $("#tx-header"),
  txPad = $("#tx-pad"), txRate = $("#tx-rate"), txCount = $("#tx-count"), txGo = $("#tx-go"), txFb = $("#tx-feedback"),
  txLoad = $("#tx-load"), modeBtns = document.querySelectorAll(".mode button");
let epNames = null;

function chip(box, value, label, checked, disabled, cls) {
  const l = document.createElement("label"); l.className = "chip" + (cls ? " " + cls : "");
  const i = document.createElement("input"); i.type = "checkbox"; i.value = value; i.checked = checked; i.disabled = disabled;
  const s = document.createElement("span"); s.textContent = label;
  l.append(i, s); box.appendChild(l);
}
function fillSelect(selEl, names, first) {
  selEl.textContent = "";
  if (first) { const o = document.createElement("option"); o.value = first[0]; o.textContent = first[1]; selEl.appendChild(o); }
  for (const n of names) { const o = document.createElement("option"); o.value = n; o.textContent = n; selEl.appendChild(o); }
}
function renderTxChips() {
  const focused = document.activeElement && txDst.contains(document.activeElement) ? document.activeElement.value : null;
  txDst.textContent = "";
  chip(txDst, "__all", "All endpoints", tx.all, false, "all");
  for (const n of epNames) if (n !== tx.src) chip(txDst, n, n, tx.all || tx.dst.includes(n), tx.all);
  if (focused) { const i = [...txDst.querySelectorAll("input")].find((x) => x.value === focused); if (i) i.focus(); }
}
function renderRxChips() {
  const focused = document.activeElement && rxDst.contains(document.activeElement) ? document.activeElement.value : null;
  rxDst.textContent = "";
  chip(rxDst, "__all", "All receivers", rx.all, false, "all");
  for (const n of epNames) chip(rxDst, n, n, rx.all || rx.dst.includes(n), rx.all);
  if (focused) { const i = [...rxDst.querySelectorAll("input")].find((x) => x.value === focused); if (i) i.focus(); }
}
function consoleEndpoints(names) {
  if (epNames && names.join("\n") === epNames.join("\n")) return;
  epNames = names.slice();
  fillSelect(txSrc, names, names.length ? null : ["", "No endpoints"]);
  txSrc.disabled = !names.length;
  tx.src = names.includes(tx.src) ? tx.src : names.includes(activeCentre) ? activeCentre : names[0] || null;
  if (tx.src) txSrc.value = tx.src;
  renderTxChips();
  fillSelect(rxSrc, names, ["", "anyone"]);
  rxSrc.value = names.includes(rx.src) ? rx.src : "";
  renderRxChips();
}
txSrc.addEventListener("change", () => { tx.src = txSrc.value; saveTx(); renderTxChips(); updateLoad(); });
txDst.addEventListener("change", (e) => {
  const i = e.target;
  if (i.value === "__all") tx.all = i.checked;
  else { const set = new Set(tx.dst); if (i.checked) set.add(i.value); else set.delete(i.value); tx.dst = [...set]; }
  saveTx(); renderTxChips(); updateLoad();
});

txText.value = tx.text; txHeader.checked = tx.header; txPad.value = tx.pad || 0;
txRate.value = tx.rate; txCount.value = tx.count || "";
txText.addEventListener("input", () => { tx.text = txText.value; saveTx(); updateLoad(); });
txHeader.addEventListener("change", () => { tx.header = txHeader.checked; saveTx(); updateLoad(); });
txPad.addEventListener("input", () => { tx.pad = Math.max(0, Math.floor(+txPad.value || 0)); saveTx(); updateLoad(); });
txRate.addEventListener("input", () => { tx.rate = txRate.value; saveTx(); updateLoad(); });
txCount.addEventListener("input", () => { tx.count = txCount.value; saveTx(); });
function syncMode() {
  for (const b of modeBtns) b.setAttribute("aria-pressed", String(b.dataset.mode === tx.mode));
  $(".tx").dataset.mode = tx.mode;
  txGo.textContent = tx.mode === "beacon" ? "Start beacon" : "Send";
  updateLoad();
}
for (const b of modeBtns) b.addEventListener("click", () => { tx.mode = b.dataset.mode; saveTx(); syncMode(); });

function receiverCount() {
  if (!epNames) return 0;
  return tx.all ? epNames.filter((n) => n !== tx.src).length : tx.dst.filter((n) => n !== tx.src && epNames.includes(n)).length;
}
function estimateBytes() {
  const src = tx.src || "", longest = Math.max(4, ...(epNames || []).map((n) => n.length));
  const body = tx.text.replace(/\{src\}/g, src).replace(/\{dst\}/g, "x".repeat(longest))
    .replace(/\{seq(:[^}]*)?\}/g, "123").replace(/\{bid\}/g, "1").replace(/\{t(:[^}]*)?\}/g, "1700000000.123")
    .replace(/\{[xyz](:[^}]*)?\}/g, "-123.45");
  let n = new TextEncoder().encode(body).length;
  if (tx.header) n += ("ID:" + src + ";SEQ:123;MSG:").length + (tx.mode === "beacon" ? 6 : 0);
  return Math.max(n, tx.pad || 0);
}
function fmtBits(bps) {
  if (bps >= 1e6) return trimNum(bps / 1e6) + " Mbit/s";
  if (bps >= 1e3) return trimNum(bps / 1e3) + " kbit/s";
  return Math.round(bps) + " bit/s";
}
function updateLoad() {
  if (!snap || !epNames) { txLoad.textContent = ""; return; }
  const bytes = estimateBytes(), k = receiverCount();
  let s = "About " + bytes + " bytes per message";
  s += k ? ", to " + k + (k === 1 ? " receiver" : " receivers") : ". No receivers are selected";
  let warn = !k;
  const rate = +tx.rate;
  if (tx.mode === "beacon" && k && rate > 0) {
    const bits = rate * k * bytes * 8, cap = snap.rf.capacity;
    s += ", which is " + fmtBits(bits) + " out of " + (tx.src || "the sender") + "\u2019s " + fmtBits(cap) + " radio capacity";
    if (bits > cap) { warn = true; s += ". RFComms refuses anything over capacity, so expect missing messages"; }
  }
  txLoad.textContent = s + ".";
  txLoad.classList.toggle("warn", warn);
}

async function post(url, body) {
  let r;
  try {
    r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  } catch (e) { throw new Error("Could not reach rf_dashboard.py. Is it still running?"); }
  let data = {};
  try { data = await r.json(); } catch (e) {}
  if (!r.ok) throw new Error(data.error || "The request failed (HTTP " + r.status + ").");
  return data;
}
let fbTimer = 0;
function feedback(msg, isErr) {
  txFb.textContent = msg; txFb.classList.toggle("err", !!isErr);
  clearTimeout(fbTimer);
  if (!isErr) fbTimer = setTimeout(() => { txFb.textContent = ""; }, 7000);
}
function routeText(b) { return b.src + " \u2192 " + (b.dst === "all" ? "all endpoints" : b.dst.join(", ")); }
txForm.addEventListener("submit", async (e) => {
  e.preventDefault();
  const body = { src: tx.src, dst: tx.all ? "all" : tx.dst, text: tx.text, header: tx.header, pad: tx.pad || 0 };
  const beacon = tx.mode === "beacon";
  if (beacon) { body.rate = +tx.rate; body.count = Math.floor(+tx.count || 0); }
  if (!tx.src) { feedback("Choose the endpoint to send from.", true); return; }
  if (!receiverCount()) { feedback("Choose at least one receiver other than the sender, or All endpoints.", true); return; }
  txGo.disabled = true;
  try {
    const res = await post(beacon ? "/api/beacon" : "/api/send", body);
    if (beacon) feedback("Beacon started: " + routeText(res.beacon) + ", " + trimNum(res.beacon.rate) + " per second.");
    else feedback("Sent to " + res.sent_to.join(", ") + " (" + res.bytes + " bytes each). Listen shows which ones arrived.");
  } catch (err) { feedback(err.message, true); }
  finally { txGo.disabled = false; }
});

// running beacons
const beaconEls = new Map();
function describePayload(b) {
  const parts = [b.text ? "\u201c" + b.text + "\u201d" : "no text"];
  if (b.header) parts.push("ID and SEQ header");
  if (b.pad) parts.push("padded to " + b.pad + " bytes");
  return parts.join(", ");
}
async function stopBeacon(id) {
  try { await post("/api/beacon/stop", { id }); } catch (e) { feedback(e.message, true); }
}
function renderBeacons(list) {
  const ul = $("#beacons"), ids = new Set(list.map((b) => b.id));
  for (const [id, el] of beaconEls) if (!ids.has(id)) { el.li.remove(); beaconEls.delete(id); }
  for (const b of list) {
    let el = beaconEls.get(b.id);
    if (!el) {
      const li = document.createElement("li");
      li.innerHTML = '<div class="b-main"><span class="b-route"></span><span class="b-stats"></span><span class="b-msg"></span></div>' +
        '<button type="button" class="btn">Stop</button>';
      li.querySelector("button").addEventListener("click", () => stopBeacon(b.id));
      ul.appendChild(li);
      el = { li, route: li.querySelector(".b-route"), stats: li.querySelector(".b-stats"), msg: li.querySelector(".b-msg") };
      el.route.textContent = routeText(b);
      el.msg.textContent = describePayload(b);
      el.msg.title = el.msg.textContent;
      el.li.querySelector("button").setAttribute("aria-label", "Stop beacon " + routeText(b));
      beaconEls.set(b.id, el);
    }
    let s = trimNum(b.rate) + " per second, " + b.rounds + (b.count ? " of " + b.count : "") + " sent (" +
      b.messages + (b.messages === 1 ? " message" : " messages") + ")";
    el.stats.textContent = s;
  }
  $("#beacons-empty").hidden = list.length > 0;
  $("#stop-all").hidden = list.length === 0;
  $("#beacons-n").textContent = list.length ? String(list.length) : "";
}
$("#stop-all").addEventListener("click", () => stopBeacon("all"));

// ---------------------------------------------------------------- radio console: listen
const rxDst = $("#rx-dst"), rxSrc = $("#rx-src"), rxFilter = $("#rx-filter"), rxLog = $("#rx-log"), rxStat = $("#rx-stat"),
  rxPause = $("#rx-pause");
const msgs = [];
let lastMsgId = 0, serverId = null, skipped = 0, paused = false, pausedAt = 0, logDirty = true;
rxFilter.value = rx.filter;
rxDst.addEventListener("change", (e) => {
  const i = e.target;
  if (i.value === "__all") rx.all = i.checked;
  else { const set = new Set(rx.dst); if (i.checked) set.add(i.value); else set.delete(i.value); rx.dst = [...set]; }
  saveRx(); renderRxChips(); logDirty = true; renderLog();
});
rxSrc.addEventListener("change", () => { rx.src = rxSrc.value; saveRx(); logDirty = true; renderLog(); });
rxFilter.addEventListener("input", () => { rx.filter = rxFilter.value; saveRx(); logDirty = true; renderLog(); });
rxPause.addEventListener("click", () => {
  paused = !paused; pausedAt = lastMsgId;
  rxPause.setAttribute("aria-pressed", String(paused)); rxPause.textContent = paused ? "Resume" : "Pause";
  logDirty = true; renderLog();
});
$("#rx-clear").addEventListener("click", () => { msgs.length = 0; skipped = 0; pausedAt = lastMsgId; logDirty = true; renderLog(); });
function csvCell(v) { const s = v === null || v === undefined ? "" : String(v); return /[",\n\r]/.test(s) ? '"' + s.replace(/"/g, '""') + '"' : s; }
$("#rx-csv").addEventListener("click", () => {
  const lines = ["time_iso,time_unix,from,to,bytes,rssi_dbm,distance_m,message"];
  for (const m of msgs) if (matches(m))
    lines.push([new Date(m.t * 1000).toISOString(), m.t, m.src, m.dst, m.bytes, m.rssi, m.d, m.text].map(csvCell).join(","));
  const url = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], { type: "text/csv" }));
  const a = document.createElement("a");
  a.href = url; a.download = "rf_messages_" + new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-") + ".csv";
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
});
function matches(m) {
  if (!rx.all && !rx.dst.includes(m.dst)) return false;
  if (rx.src && m.src !== rx.src) return false;
  if (rx.filter && !m.text.toLowerCase().includes(rx.filter.toLowerCase())) return false;
  return true;
}
function takeMessages(s) {
  if (s.server_id !== serverId) { serverId = s.server_id; lastMsgId = 0; pausedAt = 0; }
  if (s.skipped) skipped += s.skipped;
  for (const m of s.msgs || []) if (m.id > lastMsgId) { msgs.push(m); lastMsgId = m.id; logDirty = true; }
  if (msgs.length > 5000) msgs.splice(0, msgs.length - 5000);
}
function fmtTime(t) {
  const d = new Date(t * 1000), two = (n) => String(n).padStart(2, "0");
  return two(d.getHours()) + ":" + two(d.getMinutes()) + ":" + two(d.getSeconds()) + "." + String(d.getMilliseconds()).padStart(3, "0");
}
function renderLog() {
  let total = 0, recent = 0, waiting = 0;
  const now = snap ? snap.now : Date.now() / 1000;
  for (const m of msgs) if (matches(m)) { total++; if (now - m.t <= 5) recent++; if (paused && m.id > pausedAt) waiting++; }
  let stat = total.toLocaleString() + (total === 1 ? " message" : " messages") + ", " + (recent / 5).toFixed(1) + " per second";
  if (paused) stat = "Paused. " + waiting.toLocaleString() + " new since pausing; " + stat;
  else if (total > 300) stat += ", newest 300 shown";
  if (skipped) stat += ". " + skipped.toLocaleString() + " arrived too fast to show";
  rxStat.textContent = stat + ".";
  if (!logDirty || paused) return;
  logDirty = false;
  const out = [];
  for (let i = msgs.length - 1; i >= 0 && out.length < 300; i--) {
    const m = msgs[i];
    if (!matches(m)) continue;
    out.push('<tr><td class="t">' + fmtTime(m.t) + "</td><td>" + esc(m.src) + "</td><td>" + esc(m.dst) +
      '</td><td class="msg" title="' + esc(m.text) + '">' + esc(m.text) + '</td><td class="num">' + m.bytes +
      '</td><td class="num">' + (m.rssi === null ? "" : fmtDbm(m.rssi)) + '</td><td class="num">' +
      (m.d === null ? "" : fmtDistStr(m.d)) + "</td></tr>");
  }
  rxLog.innerHTML = out.join("") || '<tr class="none"><td colspan="7">' + (msgs.length
    ? "No messages match these filters."
    : "Nothing received yet. Start a beacon under Transmit, or run Rf.py.") + "</td></tr>";
}
function consoleUpdate(s) {
  consoleEndpoints(s.endpoints.map((e) => e.name));
  takeMessages(s);
  renderBeacons(s.beacons || []);
  renderLog();
  updateLoad();
}
syncMode();

// ---------------------------------------------------------------- computer vision
// The page asks for "a frame newer than the one I have". The server answers as soon
// as one arrives (or after 1 s with "nothing new"), so every frame is shown once,
// as fast as your computer-vision program sends them, and nothing is downloaded twice.
const cvImg = $("#cv-image"), cvWait = $("#cv-wait"), cvBadge = $("#cv-badge");
const cvStatus = $("#cv-status"), cvStatusText = $("#cv-status-text");
let cvId = 0, cvServer = null, cvUrl = null;
const sleep = (ms) => new Promise((ok) => setTimeout(ok, ms));
async function cvLoop() {
  for (;;) {
    try {
      const r = await fetch("/cv?after=" + cvId, { cache: "no-store" });
      if (r.status === 204) continue;                      // nothing new within 1 s: ask again
      if (!r.ok) throw new Error("HTTP " + r.status);
      const id = +r.headers.get("X-Frame-Id");
      const url = URL.createObjectURL(await r.blob());
      const next = new Image();
      next.src = url;
      try { await next.decode(); } catch (e) { URL.revokeObjectURL(url); cvId = id || cvId + 1; continue; }
      cvImg.src = url;                                      // swap only once decoded: no flicker
      if (cvUrl) URL.revokeObjectURL(cvUrl);
      cvUrl = url;
      cvId = id || cvId + 1;
      cvImg.hidden = false;
      cvWait.hidden = true;
    } catch (e) {
      await sleep(1000);                                    // dashboard stopped: try again shortly
    }
  }
}
function cvUpdate(s) {
  if (s.server_id !== cvServer) { cvServer = s.server_id; cvId = 0; }   // the dashboard restarted
  const cv = s.cv || { id: 0 };
  if (!cv.id) {
    cvStatus.dataset.state = "wait"; cvStatusText.textContent = "Waiting for the camera";
    cvBadge.hidden = true; cvImg.classList.remove("stale");
    return;
  }
  const stale = cv.age > cv.stale_after;
  const size = cvImg.naturalWidth ? ", " + cvImg.naturalWidth + "×" + cvImg.naturalHeight : "";
  if (stale) {
    const ago = cv.age < 120 ? Math.round(cv.age) + " s" : Math.round(cv.age / 60) + " min";
    cvStatus.dataset.state = "down";
    cvStatusText.textContent = "No new frame for " + ago;
    cvBadge.textContent = "Last frame " + ago + " ago";
  } else {
    cvStatus.dataset.state = "live";
    cvStatusText.textContent = "Live, " + cv.fps.toFixed(1) + " frames/s" + size;
  }
  cvBadge.hidden = !stale;
  cvImg.classList.toggle("stale", stale);
}
cvLoop();

// ---------------------------------------------------------------- animation loop
function measure() { const w = svg.getBoundingClientRect().width; if (w > 0) fontK = 1000 / w; }
new ResizeObserver(measure).observe(svg);
measure();
function frame(now) {
  if (reduceMotion) view.max = view.target;
  else view.max += (view.target - view.max) * 0.12;
  if (Math.abs(view.max - view.target) < view.target * 1e-4) view.max = view.target;
  drawGrid(); drawCover(); drawZones(); drawNodes(); drawPulses(now);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
connect();
})();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
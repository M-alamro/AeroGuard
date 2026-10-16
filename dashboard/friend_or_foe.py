#!/usr/bin/env python3
"""
friend_or_foe.py - decide whether each broadcasting drone is friendly,
and whether friendly drones are in the zone they should be in right now.

Every friendly drone broadcasts once per second:

    REMOTEID_SIGMAC[_anything]    for example   UA-001_dc:a6:32:d1:b7:01_12.0,-3.5,10.0_1759850000

    REMOTEID   who the drone says it is          UA-001 ... UA-999
    SIGMAC     its physical signature            dc:a6:32:d1:b7:01 ... :99
               (stands in for heat map, frequency profile, ...)
    anything   after the second "_" is ignored (position, time, ...)

This program listens as one endpoint (the drone doing the checking) and, for
every sender it hears, looks the two values up in the registry and decides
with the decision table in section 5:

                              REMOTE ID
                    missing    in registry          not in registry
    SIGNATURE
    not in registry Invader    Swapped or hacking   Invader
    in registry     Invader    Friendly if they     Invader
                               match, else Hacking
    missing         Sensing    Sensing issue        Sensing issue
                    issue

For a FRIENDLY drone it also checks its position against the zone the
registry gives it for the current time of day: IN ZONE or OUT OF ZONE.

RUN
    python3 friend_or_foe.py x500_0            listen as endpoint x500_0
    python3 friend_or_foe.py x500_0 --plain    one line per change, no table
    python3 friend_or_foe.py --selftest        check the logic, no Gazebo needed

Messages work with or without the dashboard / Rf.py header
(ID:drone1;SEQ:4;MSG:UA-001_dc:a6:32:d1:b7:01_...). A message without any "_"
(ordinary text) is not an identity broadcast and is ignored.

HOW THE FILE IS LAID OUT
    1. Zones             rectangles drones may fly in          <- edit here
    2. Registry          our drones: signature + time frames   <- edit here
    3. Settings          timings and formats
    4. Reading           message text  ->  remote ID + signature
    5. Checking          the decision table  ->  verdict
    6. Zone check        is a friendly drone where it should be now?
    7. Tracking          verdicts per sender, identity changes, silence, zone
    8. Showing           table or plain lines in the terminal
    9. Gazebo            the listener's /NAME/rx topic and model positions
   10. Self-test         worked examples of every case
   11. Main
"""
import argparse
import importlib
import math
import os
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass


# =============================================================================
# 1. ZONES: rectangles drones may fly in                          <- edit here
# =============================================================================
# x and y in metres, in Gazebo's world frame (the same axes as the dashboard
# map: +X right, +Y up). Height is not checked.
@dataclass(frozen=True)
class Zone:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def distance_outside(self, x, y):
        """0 when (x, y) is inside the rectangle, otherwise metres to its nearest edge."""
        dx = max(self.x_min - x, 0.0, x - self.x_max)
        dy = max(self.y_min - y, 0.0, y - self.y_max)
        return math.hypot(dx, dy)


ZONES = {
    # "BASE":  Zone(x_min=-15, x_max=15, y_min=-15, y_max=15),     # around the origin
    # "NORTH": Zone(x_min=-40, x_max=40, y_min=15,  y_max=60),
    # "EAST":  Zone(x_min=15,  x_max=60, y_min=-40, y_max=40),
    # "WEST":  Zone(x_min=-60, x_max=-15, y_min=-40, y_max=40),

    "zone1": Zone(x_min=0, x_max=25, y_min=-25, y_max=25),
    "zone2": Zone(x_min=25, x_max=50, y_min=-25, y_max=25),




}


# =============================================================================
# 2. REGISTRY: the drones we know are ours                         <- edit here
# =============================================================================
# For each remote ID: the signature MAC that must come with it, and which zone
# it must be in during which time frame.
#
# Time frames are "HH:MM-HH:MM" in this computer's local clock time. The start
# is included, the end is not, "24:00" is the end of the day, and a frame may
# run past midnight ("22:00-06:00"). Outside all of its time frames a drone is
# not allowed anywhere, so it counts as OUT OF ZONE.
@dataclass(frozen=True)
class Drone:
    signature: str      # signature MAC registered to this remote ID
    schedule: dict      # "HH:MM-HH:MM" -> zone name


REGISTRY = {
    "UA-001": Drone(signature="dc:a6:32:d1:b7:01", schedule={"12:00-14:00": "zone1"}),
    "UA-002": Drone(signature="dc:a6:32:d1:b7:02", schedule={"18:00-21:00": "zone2",
                                                             "12:00-24:00": "zone1"}),
    "UA-003": Drone(signature="dc:a6:32:d1:b7:03", schedule={"06:00-18:00": "zone1"}),
    "UA-004": Drone(signature="dc:a6:32:d1:b7:04", schedule={"00:00-24:00": "zone1"}),
    # "UA-005": Drone(signature="dc:a6:32:d1:b7:05", schedule={"18:00-06:00": "NORTH"}),
}


# =============================================================================
# 3. SETTINGS
# =============================================================================
SILENT_AFTER_S = 3.0      # friendlies broadcast every 1 s; 3 s without one = silent
REFRESH_S = 1.0           # how often the table is redrawn

REMOTE_ID_FORMAT = re.compile(r"^UA-(\d{3})$")                 # UA-001 ... UA-999
SIG_MAC_FORMAT = re.compile(r"^dc:a6:32:d1:b7:(\d{2})$")       # ...:01 ... :99


# =============================================================================
# 4. READING: message text -> remote ID + signature
# =============================================================================
@dataclass(frozen=True)
class Broadcast:
    remote_id: str      # e.g. "UA-001", or "" when missing
    sig_mac: str        # e.g. "dc:a6:32:d1:b7:01", or "" when missing


def identity_text(payload):
    """The part of a message that carries the identity.

    Dashboard / Rf.py messages:  ID:drone1;BID:2;SEQ:4;MSG:UA-001_dc:..._x,y,z_t;PAD:~~~
    Plain messages:              UA-001_dc:..._x,y,z_t
    """
    if "MSG:" in payload:
        payload = payload.split("MSG:", 1)[1]
        payload = payload.split(";PAD:", 1)[0]
    return payload.strip()


def read_broadcast(payload):
    """The remote ID and signature in a message, or None if it is not an identity broadcast.

    Only the first two "_"-separated fields count; anything after them is ignored.
    Either field may be empty, which means missing:
        _dc:a6:32:d1:b7:01        remote ID missing
        UA-001_                   signature missing
    """
    text = identity_text(payload)
    if "_" not in text:
        return None                 # ordinary text, an empty beacon, ...
    fields = text.split("_")
    return Broadcast(remote_id=fields[0].strip().upper(), sig_mac=fields[1].strip().lower())


# =============================================================================
# 5. CHECKING: the decision table -> verdict
# =============================================================================
MISSING, IN_REGISTRY, NOT_IN_REGISTRY = "missing", "in registry", "not in registry"

# The decision table, as drawn: rows are the signature, columns the remote ID.
#                           remote ID:  MISSING              IN_REGISTRY         NOT_IN_REGISTRY
DECISION_TABLE = {
    NOT_IN_REGISTRY: {MISSING: "INVADER", IN_REGISTRY: "SWAPPED",    NOT_IN_REGISTRY: "INVADER"},
    IN_REGISTRY:     {MISSING: "INVADER", IN_REGISTRY: "MATCHING?",  NOT_IN_REGISTRY: "INVADER"},
    MISSING:         {MISSING: "SENSING", IN_REGISTRY: "SENSING",    NOT_IN_REGISTRY: "SENSING"},
}
# "MATCHING?": FRIENDLY if the signature is the one registered for that remote
# ID, HACKING if it is registered to a different drone.

# What each outcome is called, and which overall status it belongs to.
#             code        status            label
OUTCOMES = {"FRIENDLY": ("FRIENDLY",      "Friendly"),
            "HACKING":  ("NOT FRIENDLY",  "Hacking suspected"),
            "SWAPPED":  ("NOT FRIENDLY",  "Swapped drone or hacking"),
            "INVADER":  ("NOT FRIENDLY",  "Invader"),
            "CHANGED":  ("NOT FRIENDLY",  "Changed identity"),     # section 7
            "SENSING":  ("SENSING ISSUE", "Issue in our sensing system")}


@dataclass(frozen=True)
class Verdict:
    code: str           # a key of OUTCOMES
    reason: str

    @property
    def status(self):
        return OUTCOMES[self.code][0]

    @property
    def label(self):
        return OUTCOMES[self.code][1]

    @property
    def friendly(self):
        return self.code == "FRIENDLY"


def owner_of(sig_mac):
    """The remote ID a signature is registered to, or None."""
    for remote_id, drone in REGISTRY.items():
        if drone.signature == sig_mac:
            return remote_id
    return None


def remote_id_state(remote_id):
    if not remote_id:
        return MISSING
    return IN_REGISTRY if remote_id in REGISTRY else NOT_IN_REGISTRY


def signature_state(sig_mac):
    if not sig_mac:
        return MISSING
    return IN_REGISTRY if owner_of(sig_mac) else NOT_IN_REGISTRY


def check_broadcast(b):
    """Look the remote ID and signature up, then read the outcome from the decision table."""
    code = DECISION_TABLE[signature_state(b.sig_mac)][remote_id_state(b.remote_id)]
    if code == "MATCHING?":
        code = "FRIENDLY" if REGISTRY[b.remote_id].signature == b.sig_mac else "HACKING"
    return Verdict(code, explain(code, b))


def explain(code, b):
    """One line saying why, in plain words."""
    rid, sig, owner = b.remote_id, b.sig_mac, owner_of(b.sig_mac)
    if rid and rid not in REGISTRY and not REMOTE_ID_FORMAT.match(rid):
        rid += " (not a valid remote ID)"
    if code == "FRIENDLY":
        return f"{rid} with its registered signature"
    if code == "HACKING":
        return f"{rid} came with the signature of {owner}: hacking suspected"
    if code == "SWAPPED":
        return f"{rid} is registered, but signature {sig} is not: illegally swapped drone, or hacking"
    if code == "SENSING":
        return f"no signature sensed for {rid or 'a drone with no remote ID'}: issue in our sensing system"
    # INVADER
    if not rid:
        return f"no remote ID, with the signature of {owner}" if owner else f"no remote ID, unknown signature {sig}"
    if owner:
        return f"{rid} is not registered, but has the signature of {owner}"
    return f"{rid} and signature {sig} are not in the registry"


# =============================================================================
# 6. ZONE CHECK: is a friendly drone where it should be right now?
# =============================================================================
@dataclass(frozen=True)
class ZoneCheck:
    state: str          # "IN ZONE", "OUT OF ZONE", or "" when it cannot be checked
    zone: str           # the zone it should be in now ("" if none)
    reason: str


NOT_CHECKED = ZoneCheck("", "", "")


def minutes(hhmm):
    """'08:30' -> 510 minutes after midnight."""
    hours, mins = hhmm.strip().split(":")
    return int(hours) * 60 + int(mins)


def scheduled_zone(remote_id, now):
    """(zone, time frame) the drone should be in at time `now`, or (None, None)."""
    t = time.localtime(now)
    m = t.tm_hour * 60 + t.tm_min + t.tm_sec / 60
    for frame, zone in REGISTRY[remote_id].schedule.items():
        start, end = (minutes(part) for part in frame.split("-"))
        if start < end:
            inside = start <= m < end
        else:                                   # runs past midnight
            inside = m >= start or m < end
        if inside:
            return zone, frame
    return None, None


def check_zone(remote_id, position, now):
    """Compare a registered drone's position (x, y, ...) with its zone for this time of day."""
    if remote_id not in REGISTRY:
        return NOT_CHECKED
    zone, frame = scheduled_zone(remote_id, now)
    if position is None:
        return ZoneCheck("", zone or "", "position unknown")
    if zone is None:
        return ZoneCheck("OUT OF ZONE", "", f"no zone scheduled for {remote_id} at {clock(now, seconds=False)}")
    outside = ZONES[zone].distance_outside(position[0], position[1])
    if outside == 0:
        return ZoneCheck("IN ZONE", zone, f"in {zone} ({frame})")
    metres = "under 0.1" if outside < 0.1 else f"{outside:.1f}" if outside < 10 else f"{outside:.0f}"
    return ZoneCheck("OUT OF ZONE", zone, f"should be in {zone} ({frame}), {metres} m outside")


# =============================================================================
# 7. TRACKING: verdicts per sender, over time
# =============================================================================
# On top of the decision table, per sender over time:
#   CHANGED   a sender used one remote ID or signature, and later a different one
#             (sticky: NOT FRIENDLY until the program restarts). A missing value
#             is a sensing problem, not a change.
#   SILENT    no identity broadcast for SILENT_AFTER_S; the last verdict is kept
#   zone      for friendly senders, checked against their time frames

@dataclass
class Track:
    sender: str                      # RF address the messages come from (e.g. drone1)
    broadcast: Broadcast = None      # latest identity it sent
    first_id: str = ""               # first remote ID it sent
    first_sig: str = ""              # first signature it sent
    change: str = ""                 # set once it changes identity
    verdict: Verdict = None
    zone: ZoneCheck = NOT_CHECKED
    last_heard: float = 0.0
    messages: int = 0

    def status(self, now):
        if now - self.last_heard > SILENT_AFTER_S:
            return "SILENT"
        return self.verdict.status


class Tracker:
    def __init__(self, on_change=None, position_of=None):
        self.tracks = {}                    # sender -> Track
        self._shown = {}                    # sender -> state last reported
        self._lock = threading.Lock()
        self.on_change = on_change or (lambda line: None)
        self.position_of = position_of or (lambda sender: None)     # sender -> (x, y, z) or None

    # ---- called for every received message
    def handle(self, sender, payload, now=None):
        now = time.time() if now is None else now
        b = read_broadcast(payload)
        if b is None:
            return                          # not an identity broadcast
        verdict = check_broadcast(b)

        with self._lock:
            track = self.tracks.setdefault(sender, Track(sender))
            track.messages += 1
            track.last_heard = now
            track.broadcast = b

            if b.remote_id:
                if not track.first_id:
                    track.first_id = b.remote_id
                elif b.remote_id != track.first_id and not track.change:
                    track.change = f"first said {track.first_id}, then {b.remote_id}"
            if b.sig_mac:
                if not track.first_sig:
                    track.first_sig = b.sig_mac
                elif b.sig_mac != track.first_sig and not track.change:
                    track.change = f"signature changed from {track.first_sig} to {b.sig_mac}"

            track.verdict = Verdict("CHANGED", track.change) if track.change else verdict
        self.report_changes(now)

    # ---- called every second, and after every message
    def report_changes(self, now=None):
        now = time.time() if now is None else now
        with self._lock:
            friendly = [s for s, t in self.tracks.items() if t.verdict.friendly]
        positions = {s: self.position_of(s) for s in friendly}     # outside the lock

        lines = []
        with self._lock:
            for sender, track in self.tracks.items():
                if track.verdict.friendly:
                    track.zone = check_zone(track.broadcast.remote_id, positions.get(sender), now)
                else:
                    track.zone = NOT_CHECKED
                state = (track.status(now), track.verdict.code, track.zone.state, track.zone.zone)
                if self._shown.get(sender) != state:
                    self._shown[sender] = state
                    lines.append(event_line(track, now))
        for line in lines:
            self.on_change(line)

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        with self._lock:
            return [(t, t.status(now), now - t.last_heard)
                    for t in sorted(self.tracks.values(), key=lambda t: t.sender)]


# =============================================================================
# 8. SHOWING: table or plain lines
# =============================================================================
USE_COLOUR = sys.stdout.isatty() and "NO_COLOR" not in os.environ
COLOURS = {"FRIENDLY": "\033[32m", "NOT FRIENDLY": "\033[31m", "SENSING ISSUE": "\033[35m",
           "SILENT": "\033[33m", "IN ZONE": "\033[32m", "OUT OF ZONE": "\033[31m"}


def paint(key, text):
    return f"{COLOURS[key]}{text}\033[0m" if USE_COLOUR and key in COLOURS else text


def clock(t, seconds=True):
    return time.strftime("%H:%M:%S" if seconds else "%H:%M", time.localtime(t))


def event_line(track, now):
    status = track.status(now)
    who = track.broadcast.remote_id or "no remote ID"
    if status == "SILENT":
        text = f"{paint(status, status)}, nothing heard for {now - track.last_heard:.0f} s"
    elif status == "NOT FRIENDLY":
        text = f"{paint(status, status)} ({track.verdict.label}), {track.verdict.reason}"
    else:
        text = f"{paint(status, status)}, {track.verdict.reason}"
    if track.zone.state:
        text += f"; {paint(track.zone.state, track.zone.state)}, {track.zone.reason}"
    return f"{clock(now)}  {track.sender} ({who}): {text}"


def draw_table(tracker, listener, events):
    now = time.time()
    rows = tracker.snapshot(now)
    plan = []
    for rid in REGISTRY:
        zone, _ = scheduled_zone(rid, now)
        plan.append(f"{rid} {zone or '(none)'}")
    out = ["\033[2J\033[H" if sys.stdout.isatty() else "",
           f"Friend or foe, listening as {listener}    {clock(now)}",
           f"Zones now: {', '.join(plan)}",
           ""]
    header = (f"{'SENDER':<10} {'REMOTE ID':<9} {'SIGNATURE':<17} {'VERDICT':<27} "
              f"{'ZONE':<12} {'HEARD':>6} {'MSGS':>5}  REASON")
    out += [header, "-" * len(header)]
    if not rows:
        out.append(f"(no identity broadcast heard yet on /{listener}/rx)")
    for t, status, age in rows:
        label = "Silent" if status == "SILENT" else t.verdict.label
        reason = t.verdict.reason if status != "SILENT" else f"silent; last: {t.verdict.reason}"
        if t.zone.state:
            reason += f"; {t.zone.reason}"
        zone = t.zone.state or "-"
        out.append(f"{t.sender:<10} {t.broadcast.remote_id or '-':<9} {t.broadcast.sig_mac or '-':<17} "
                   f"{paint(status, f'{label:<27}')} {paint(t.zone.state, f'{zone:<12}')} "
                   f"{age:5.1f}s {t.messages:>5}  {reason}")
    out += ["", "Recent changes:"] + (list(events) or ["(none yet)"])
    print("\n".join(out), flush=True)


# =============================================================================
# 9. GAZEBO: the listener's receive topic, and where every model is
# =============================================================================
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
            # usually the protobuf clash with a newer copy in ~/.local:
            # restart once with -s so Python ignores ~/.local
            if not sys.flags.no_user_site:
                os.execv(sys.executable, [sys.executable, "-s", *sys.argv])
            raise
    sys.exit("Gazebo Python bindings not found. Install them with:\n"
             "  sudo apt install python3-gz-transport13 python3-gz-msgs10")


class Positions:
    """Latest position of every model, from /world/<world>/pose/info."""

    def __init__(self):
        self._pos = {}
        self._lock = threading.Lock()

    def on_poses(self, msg):
        with self._lock:
            for p in msg.pose:
                self._pos[p.name] = (p.position.x, p.position.y, p.position.z)

    def of(self, name):
        """Model NAME, or the single model NAME_<n> (PX4 adds _0 to drone names)."""
        with self._lock:
            if name in self._pos:
                return self._pos[name]
            matches = [m for m in self._pos if m.startswith(name + "_")]
            return self._pos[matches[0]] if len(matches) == 1 else None


def start_listening(listener, world, tracker, positions):
    node_cls, Dataframe, Pose_V = load_gz()
    node = node_cls()

    def on_message(msg):
        tracker.handle(msg.src_address, bytes(msg.data).decode(errors="replace"))

    for msg_type, topic, callback in ((Dataframe, f"/{listener}/rx", on_message),
                                      (Pose_V, f"/world/{world}/pose/info", positions.on_poses)):
        if not node.subscribe(msg_type, topic, callback):
            sys.exit(f"Could not subscribe to {topic}")
    return node                 # keep a reference, or the subscriptions stop


# =============================================================================
# 10. SELF-TEST: worked examples of every case
# =============================================================================
# Uses its own small registry, so it still passes after you edit yours.
TEST_ZONES = {"HOME": Zone(x_min=-10, x_max=10, y_min=-10, y_max=10),
              "FAR":  Zone(x_min=100, x_max=120, y_min=-10, y_max=10)}
TEST_REGISTRY = {"UA-001": Drone("dc:a6:32:d1:b7:01", {"08:00-12:00": "HOME", "12:00-14:00": "FAR"}),
                 "UA-002": Drone("dc:a6:32:d1:b7:02", {"22:00-02:00": "HOME"})}


def selftest():
    global REGISTRY, ZONES
    saved = REGISTRY, ZONES
    REGISTRY, ZONES = TEST_REGISTRY, TEST_ZONES
    try:
        return run_selftest()
    finally:
        REGISTRY, ZONES = saved


def run_selftest():
    failures = 0

    def expect(label, got, want):
        nonlocal failures
        ok = got == want
        failures += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {label:<62} -> {got}" + ("" if ok else f" (wanted {want})"))

    def code(payload):
        b = read_broadcast(payload)
        return "ignored" if b is None else check_broadcast(b).code

    print("The decision table (rows: signature, columns: remote ID):")
    print(f"  {'':<17}" + "".join(f"{c:<19}" for c in (MISSING, IN_REGISTRY, NOT_IN_REGISTRY)))
    for row, cells in DECISION_TABLE.items():
        print(f"  {row:<17}" + "".join(f"{cells[c]:<19}" for c in (MISSING, IN_REGISTRY, NOT_IN_REGISTRY)))

    print("\nEvery cell of the table (registry: UA-001 = :01, UA-002 = :02):")
    for payload, want in [
        ("_",                           "SENSING"),    # signature missing, remote ID missing
        ("UA-001_",                     "SENSING"),    # signature missing, remote ID in registry
        ("UA-042_",                     "SENSING"),    # signature missing, remote ID not in registry
        ("_dc:a6:32:d1:b7:01",          "INVADER"),    # signature in registry, remote ID missing
        ("UA-001_dc:a6:32:d1:b7:01",    "FRIENDLY"),   # both in registry, matching
        ("UA-001_dc:a6:32:d1:b7:02",    "HACKING"),    # both in registry, not matching
        ("UA-042_dc:a6:32:d1:b7:01",    "INVADER"),    # signature in registry, remote ID not
        ("_dc:a6:32:d1:b7:42",          "INVADER"),    # signature not in registry, remote ID missing
        ("UA-001_dc:a6:32:d1:b7:42",    "SWAPPED"),    # signature not in registry, remote ID in it
        ("UA-042_dc:a6:32:d1:b7:42",    "INVADER"),    # neither in registry
    ]:
        expect(payload, code(payload), want)

    print("\nReading messages:")
    for payload, want in [
        ("UA-001_dc:a6:32:d1:b7:01_12.0,-3.5,10.0_1759850000.2",               "FRIENDLY"),
        ("ID:drone1;BID:2;SEQ:5;MSG:UA-001_dc:a6:32:d1:b7:01_1,2,3_99;PAD:~~", "FRIENDLY"),
        ("ua-001_DC:A6:32:D1:B7:01",                                            "FRIENDLY"),
        ("UA-1000_dc:a6:32:d1:b7:01",                                           "INVADER"),
        ("hello from drone1",                                                   "ignored"),
        ("ID:toyota2;SEQ:3;MSG:",                                               "ignored"),
    ]:
        expect(payload, code(payload), want)

    print("\nOver time:")
    t = Tracker()
    t.handle("drone2", "UA-002_dc:a6:32:d1:b7:02", now=0)
    t.handle("drone2", "UA-001_dc:a6:32:d1:b7:01", now=1)
    expect("drone2 switches from UA-002 to UA-001", t.tracks["drone2"].verdict.code, "CHANGED")
    t.handle("drone2", "UA-002_dc:a6:32:d1:b7:02", now=2)
    expect("...and stays flagged after switching back", t.tracks["drone2"].verdict.code, "CHANGED")

    t = Tracker()
    t.handle("drone1", "UA-001_dc:a6:32:d1:b7:01", now=0)
    t.handle("drone1", "UA-001_", now=1)
    expect("signature not sensed for one message", t.tracks["drone1"].verdict.code, "SENSING")
    t.handle("drone1", "UA-001_dc:a6:32:d1:b7:01", now=2)
    expect("...is not a change of identity", t.tracks["drone1"].verdict.code, "FRIENDLY")
    t.handle("drone1", "hello", now=3)
    expect("ordinary text does not count as a broadcast", t.tracks["drone1"].messages, 3)
    expect("drone1 heard 2 s ago", t.tracks["drone1"].status(4.0), "FRIENDLY")
    expect("drone1 heard 4 s ago", t.tracks["drone1"].status(6.0), "SILENT")

    print("\nZones (UA-001: HOME 08:00-12:00, FAR 12:00-14:00; UA-002: HOME 22:00-02:00):")

    def at(hhmm):
        h, m = map(int, hhmm.split(":"))
        return time.mktime((2026, 10, 7, h, m, 0, 0, 0, -1))

    for rid, hhmm, pos, want in [
        ("UA-001", "09:00", (0, 0),   "IN ZONE"),
        ("UA-001", "09:00", (30, 0),  "OUT OF ZONE"),
        ("UA-001", "13:00", (110, 0), "IN ZONE"),
        ("UA-001", "13:00", (0, 0),   "OUT OF ZONE"),
        ("UA-001", "15:00", (0, 0),   "OUT OF ZONE"),      # no time frame covers 15:00
        ("UA-002", "23:30", (0, 0),   "IN ZONE"),
        ("UA-002", "01:00", (0, 0),   "IN ZONE"),          # frame runs past midnight
        ("UA-002", "03:00", (0, 0),   "OUT OF ZONE"),
        ("UA-001", "09:00", None,     ""),                 # position unknown
    ]:
        z = check_zone(rid, pos, at(hhmm))
        expect(f"{rid} at {hhmm}, position {pos}: {z.reason}", z.state, want)

    t = Tracker(position_of=lambda sender: (30.0, 0.0, 5.0))
    t.handle("drone1", "UA-001_dc:a6:32:d1:b7:01", now=at("09:00"))
    expect("a friendly sender 20 m outside its zone", t.tracks["drone1"].zone.state, "OUT OF ZONE")
    t.handle("drone9", "UA-042_dc:a6:32:d1:b7:42", now=at("09:00"))
    expect("zones are not checked for non-friendly senders", t.tracks["drone9"].zone.state, "")

    print(f"\n{'All checks passed.' if not failures else f'{failures} check(s) failed.'}")
    return 1 if failures else 0


# =============================================================================
# 11. MAIN
# =============================================================================
def check_registry():
    """Catch typos in ZONES and REGISTRY before anything else runs."""
    for name, z in ZONES.items():
        if not (z.x_min < z.x_max and z.y_min < z.y_max):
            sys.exit(f"Zone {name}: x_min must be below x_max and y_min below y_max")
    seen = {}
    for rid, drone in REGISTRY.items():
        if not REMOTE_ID_FORMAT.match(rid) or rid == "UA-000":
            sys.exit(f"Registry entry {rid}: remote ID must be UA-001 ... UA-999")
        if not SIG_MAC_FORMAT.match(drone.signature) or drone.signature.endswith(":00"):
            sys.exit(f"Registry entry {rid}: signature must be dc:a6:32:d1:b7:01 ... :99")
        if drone.signature in seen:
            sys.exit(f"Registry: {rid} and {seen[drone.signature]} have the same signature {drone.signature}")
        seen[drone.signature] = rid
        for frame, zone in drone.schedule.items():
            try:
                start, end = (minutes(part) for part in frame.split("-"))
                assert 0 <= start <= 1440 and 0 <= end <= 1440
            except (ValueError, AssertionError):
                sys.exit(f"Registry entry {rid}: time frame '{frame}' must look like 08:00-12:00")
            if zone not in ZONES:
                sys.exit(f"Registry entry {rid}: zone '{zone}' is not defined in ZONES")


def main():
    ap = argparse.ArgumentParser(description="Decide whether each broadcasting drone is friendly.")
    ap.add_argument("listener", nargs="?", help="endpoint that does the checking, e.g. x500_0")
    ap.add_argument("--world", default="default", help="Gazebo world name (default: default)")
    ap.add_argument("--plain", action="store_true", help="print one line per change instead of a table")
    ap.add_argument("--selftest", action="store_true", help="run the worked examples, no Gazebo needed")
    args = ap.parse_args()

    check_registry()
    if args.selftest:
        sys.exit(selftest())
    if not args.listener:
        ap.error("say which endpoint listens, e.g.  python3 friend_or_foe.py x500_0")

    events = deque(maxlen=8)

    def on_change(line):
        events.append(line)
        if args.plain:
            print(line, flush=True)

    positions = Positions()
    tracker = Tracker(on_change, position_of=positions.of)
    if args.plain:
        print(f"Listening as {args.listener} on /{args.listener}/rx. Registry: {', '.join(REGISTRY)}")
    node = start_listening(args.listener, args.world, tracker, positions)    # noqa: F841 (must stay alive)

    try:
        while True:
            time.sleep(REFRESH_S)
            tracker.report_changes()
            if not args.plain:
                draw_table(tracker, args.listener, events)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
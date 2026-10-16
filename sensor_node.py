#!/usr/bin/env python3
"""
TRIPOD SENSOR NODE  —  runs on the Raspberry Pi

Its only job is to sense and publish. It holds no registry, makes no
decisions and knows nothing about who is authorised. That separation is
deliberate: when the SDR and camera replace the Wi-Fi scan in Phase 2,
only this file changes. The engine on the laptop does not.

    senses   Wi-Fi beacon frames  ->  (ssid, mac, rssi) per scan
    publishes UDP broadcast on port 47809, as JSON

USAGE
    sudo python3 sensor_node.py                     # normal operation
    sudo python3 sensor_node.py --dest 10.68.132.218  # send straight to the laptop
    sudo python3 sensor_node.py --calibrate         # RSSI reference at 1 m

WHERE OBSERVATIONS GO
    By default they are broadcast to 255.255.255.255. Some networks —
    phone hotspots especially — drop that. If the engine sees nothing,
    give --dest the laptop's address and the observations are sent
    straight to it. Nothing else changes: the engine binds the same port
    either way.

CALIBRATION
    Run --calibrate with one emitter exactly 1 metre from the antenna.
    It prints the mean RSSI. Put that number into site.json as
    rssi_at_1m. Without it, every range estimate is wrong by a constant
    factor and the gate will never behave.
"""

import argparse
import json
import re
import socket
import statistics
import subprocess
import sys
import time

OBS_PORT = 47809
PREFIX   = "AIRCRAFT-"          # only report emitters, not the venue Wi-Fi


def scan(iface):
    """
    One sweep. Returns a list of observations.

    This is the SENSING boundary. Everything downstream consumes this
    shape — ssid, mac, rssi — and nothing else. In Phase 2 the SDR
    produces the same shape from real RF energy and the camera adds a
    bearing field; the engine is unaffected.
    """
    try:
        out = subprocess.run(["iw", "dev", iface, "scan"],
                             capture_output=True, text=True, timeout=25).stdout
    except FileNotFoundError:
        print("  ! 'iw' not installed:  sudo apt install iw", file=sys.stderr)
        return []
    except subprocess.TimeoutExpired:
        return []

    seen, mac, ssid, rssi, chan = [], None, None, None, None

    def flush():
        if mac and ssid and ssid.startswith(PREFIX):
            seen.append({"ssid": ssid, "mac": mac, "rssi": rssi,
                         "channel": chan})

    for line in out.splitlines():
        line = line.strip()
        if line.startswith("BSS "):
            flush()
            mac = line.split()[1].split("(")[0].lower()
            ssid, rssi, chan = None, None, None
        elif line.startswith("SSID:"):
            ssid = line[5:].strip()
        elif line.startswith("signal:"):
            m = re.search(r"(-?\d+\.?\d*)", line)
            rssi = float(m.group(1)) if m else None
        elif "primary channel:" in line or line.startswith("DS Parameter set:"):
            m = re.search(r"(\d+)", line)
            chan = int(m.group(1)) if m else None
    flush()
    return seen


def calibrate(iface, samples=12):
    print("  Place ONE emitter exactly 1.0 m from the Pi's antenna.")
    print(f"  Taking {samples} scans. Keep everything still.\n")
    per_mac = {}
    for i in range(samples):
        for o in scan(iface):
            per_mac.setdefault(o["mac"], []).append(o["rssi"])
        print(f"    scan {i+1}/{samples}")
        time.sleep(1)
    print()
    for mac, vals in per_mac.items():
        vals = [v for v in vals if v is not None]
        if not vals:
            continue
        mean = statistics.mean(vals)
        sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        print(f"  {mac}   mean RSSI {mean:+.1f} dBm   sd {sd:.1f} dB  "
              f"(n={len(vals)})")
        print(f"     -> site.json:  \"rssi_at_1m\": {mean:.0f}, "
              f"\"rssi_sigma\": {max(2.0, sd):.1f}")
    if not per_mac:
        print("  nothing found. Is the emitter running and named AIRCRAFT-*?")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--iface", default="wlan0")
    p.add_argument("--node-id", default="NODE-1")
    p.add_argument("--dest", default="255.255.255.255",
                   help="where to send observations; the engine's address, "
                        "or a subnet broadcast such as 10.68.132.255")
    p.add_argument("--calibrate", action="store_true")
    p.add_argument("--quiet", action="store_true")
    a = p.parse_args()

    if a.calibrate:
        calibrate(a.iface)
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"  sensor node {a.node_id} on {a.iface}")
    print(f"  publishing observations to {a.dest}:{OBS_PORT}\n")

    seq = 0
    while True:
        obs = scan(a.iface)
        seq += 1
        msg = {"magic": "SLICE1", "node": a.node_id, "seq": seq,
               "t": time.time(), "obs": obs}
        sock.sendto(json.dumps(msg).encode(), (a.dest, OBS_PORT))
        if not a.quiet:
            names = ", ".join(f"{o['ssid']}@{o['rssi']:.0f}" for o in obs)
            print(f"  seq {seq:<4} {len(obs)} observed   {names}")
        time.sleep(0.5)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Reports every hands-off stretch on a recorded drive and what ended it.

Companion to hands_on_logger.py. Instead of watching live, this goes through
the rlogs of a drive already stored on the device and tests the hands-on
timer theory against the whole drive: the stock camera asks for hands on the
wheel (0x11A DAW_WrnMsgSta == 2, then SCC_CONTROL 0x1A0 byte 9 == 0x60) after
~60 s with no column torque above ~240 while cruise is engaged.

For every stretch with cruise engaged and no torque above the threshold it
records how long it lasted, what ended it (lane change, openpilot's own torque
on a curve, a driver nudge, cruise turning off, or the car's warning) and the
conditions during it. Long stretches that ended without a warning are the ones
that contradict the theory.

Read-only. Run it over SSH while the car is OFF (it refuses to run onroad so
it can't take CPU from openpilot):

    cd /data/openpilot
    python frogpilot/system/hands_on_route_report.py --list          # pick the drive
    python frogpilot/system/hands_on_route_report.py 000001e5--abc123  # analyse it

A 3-4 hour drive takes several minutes. The summary is printed and also
saved to /data/media/0/hands_on_route_<route>.txt.
"""
import argparse
import bz2
import os
import re
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

from cereal import log

REALDATA = "/data/media/0/realdata"
OUT_DIR = "/data/media/0"

TORQUE_THRESHOLD = 240  # column torque that resets the stock timer (224 measured not to, 243 to)
OP_CURVE_TORQUE = 200   # openpilot torque command above this means a reset came from openpilot steering a curve
CONTEXT_S = 3.0         # a lane change or blinker this recent explains a reset
MIN_REPORT_S = 40.0     # stretches at least this long are listed individually
GAP_S = 5.0             # a hole in the logs longer than this ends a stretch

WARNING_BUS = 2
SCC_CONTROL_ADDR = 0x1A0
SCC_WARNING_BYTE = 9
SCC_WARNING_VALUE = 0x60
DAW_ADDR = 0x11A


def daw_warning(dat):
  return (int.from_bytes(dat, "little") >> 59) & 0x7  # DAW_WrnMsgSta, 2 = hands-off call request


def segment_dirs(root):
  segs = defaultdict(dict)
  for d in Path(root).iterdir():
    m = re.match(r"^(.+)--(\d+)$", d.name)
    if m and d.is_dir():
      segs[m.group(1)][int(m.group(2))] = d
  return segs


def find_log(seg_dir):
  for name in ("rlog", "rlog.zst", "rlog.bz2", "qlog", "qlog.zst", "qlog.bz2"):
    p = seg_dir / name
    if p.is_file():
      return p
  return None


def read_events(path):
  dat = path.read_bytes()
  if path.suffix == ".bz2" or dat[:3] == b"BZh":
    dat = bz2.decompress(dat)
  elif path.suffix == ".zst" or dat[:4] == b"\x28\xb5\x2f\xfd":
    import zstandard
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(dat)
  return log.Event.read_multiple_bytes(dat), path.name.startswith("qlog")


def list_routes(root):
  routes = segment_dirs(root)
  rows = []
  for route, segs in routes.items():
    first = segs[min(segs)]
    rows.append((first.stat().st_mtime, route, len(segs)))
  print(f"{'started':<17} {'segments':>8} {'~hours':>6}  route")
  for mtime, route, n in sorted(rows)[-25:]:
    print(f"{time.strftime('%Y-%m-%d %H:%M', time.localtime(mtime)):<17} {n:>8} {n / 60:>6.1f}  {route}")


class Stretch:
  def __init__(self, t, seg, seg_t):
    self.start = t
    self.seg = seg
    self.seg_t = seg_t
    self.n = 0
    self.speed = 0.0
    self.lead = 0
    self.lat = 0
    self.standstill = 0
    self.max_op = 0
    self.max_angle = 0.0
    self.max_tq = 0.0
    self.lane_changes = 0
    self.warn_at = None

  def add(self, cs, op_cmd, lat_active, lead):
    self.n += 1
    self.speed += cs.vEgo
    self.lead += lead
    self.lat += lat_active
    self.standstill += cs.standstill
    self.max_op = max(self.max_op, op_cmd)
    self.max_angle = max(self.max_angle, abs(cs.steeringAngleDeg))
    self.max_tq = max(self.max_tq, abs(cs.steeringTorque))

  def row(self, end_t, reason):
    n = max(self.n, 1)
    warn = f"{self.warn_at:5.1f}s" if self.warn_at is not None else "  -  "
    return (f"seg {self.seg:>3} +{self.seg_t:4.1f}s  {end_t - self.start:6.1f}s  {reason:<22} warn@{warn}  "
            f"{self.speed / n * 2.23694:5.1f}mph  tq_max {self.max_tq:4.0f}  op_max {self.max_op:4d}  "
            f"angle_max {self.max_angle:5.1f}  lead {self.lead / n:4.2f}  lat {self.lat / n:4.2f}  "
            f"lane_chg {self.lane_changes}  stopped {self.standstill / n:4.2f}")


def analyse(route, segs, threshold):
  out = []
  stretches = []   # (duration, warned, reason, row)
  warnings = []
  reset_causes = Counter()
  qlog_segments = 0

  stretch = None
  last_t = None
  op_cmd = 0
  lat_active = False
  lead = False
  last_lane_change_t = -1e9
  last_blinker_t = -1e9
  lane_change_active = False
  daw = 0
  scc_warn = False
  warning_prev = False
  above = False  # torque currently above the threshold; one push counts as one reset
  engaged_time = 0.0

  def close(t, reason):
    nonlocal stretch
    if stretch is not None:
      dur = t - stretch.start
      stretches.append((dur, stretch.warn_at is not None, reason, stretch.row(t, reason)))
      stretch = None

  for idx in sorted(segs):
    path = find_log(segs[idx])
    if path is None:
      print(f"  seg {idx}: no log, skipped", file=sys.stderr)
      continue
    try:
      events, is_qlog = read_events(path)
    except Exception as e:
      print(f"  seg {idx}: unreadable ({e}), skipped", file=sys.stderr)
      continue
    qlog_segments += is_qlog
    seg_start = None
    print(f"  seg {idx}/{max(segs)} ({path.name})", file=sys.stderr, end="\r")

    for ev in events:
      try:
        which = ev.which()
      except Exception:
        continue
      t = ev.logMonoTime / 1e9

      if which == "can":
        for m in ev.can:
          if m.src == WARNING_BUS:
            if m.address == DAW_ADDR and len(m.dat) >= 8:
              daw = daw_warning(m.dat)
            elif m.address == SCC_CONTROL_ADDR and len(m.dat) > SCC_WARNING_BYTE:
              scc_warn = m.dat[SCC_WARNING_BYTE] == SCC_WARNING_VALUE
      elif which == "sendcan":
        for m in ev.sendcan:
          if m.address == 0x12A:
            raw = int.from_bytes(m.dat, "little")
            op_cmd = abs(((raw >> 41) & 0x7ff) - 1024)
      elif which == "carControl":
        lat_active = ev.carControl.latActive
      elif which == "radarState":
        lead = ev.radarState.leadOne.status
      elif which == "modelV2":
        active = ev.modelV2.meta.laneChangeState != log.LaneChangeState.off
        if active:
          last_lane_change_t = t
          if not lane_change_active and stretch is not None:
            stretch.lane_changes += 1
        lane_change_active = active
      elif which == "carState":
        cs = ev.carState
        if seg_start is None:
          seg_start = t
        if cs.leftBlinker or cs.rightBlinker:
          last_blinker_t = t

        if last_t is not None and t - last_t > GAP_S:
          close(last_t, "log gap")
        if last_t is not None and cs.cruiseState.enabled:
          engaged_time += min(t - last_t, 0.1)
        last_t = t

        if not cs.cruiseState.enabled:
          close(t, "cruise off")
          warning_prev = False
          above = False
          continue

        warning = daw == 2 or scc_warn
        if abs(cs.steeringTorque) > threshold:
          if not above:
            if warning:
              cause = "nudge during warning"
            elif t - last_lane_change_t < CONTEXT_S or t - last_blinker_t < CONTEXT_S:
              cause = "lane change"
            elif op_cmd >= OP_CURVE_TORQUE:
              cause = "openpilot curve torque"
            else:
              cause = "driver nudge"
            reset_causes[cause] += 1
            close(t, cause)
          above = True
          warning_prev = warning
          continue
        above = False

        # a new stretch starts once the torque drops back below the threshold
        if stretch is None:
          stretch = Stretch(t, idx, t - seg_start)
        stretch.add(cs, op_cmd, lat_active, lead)

        if warning and not warning_prev:
          quiet = t - stretch.start
          if stretch.warn_at is None:
            stretch.warn_at = quiet
          n = max(stretch.n, 1)
          warnings.append(f"seg {idx:>3} +{t - seg_start:4.1f}s  quiet {quiet:6.1f}s  source {'DAW' if daw == 2 else 'SCC'}  "
                          f"{cs.vEgo * 2.23694:5.1f}mph  op_max {stretch.max_op:4d}  tq_max {stretch.max_tq:4.0f}  "
                          f"angle_max {stretch.max_angle:5.1f}  lead {stretch.lead / n:4.2f}")
        warning_prev = warning

  if last_t is not None:
    close(last_t, "end of drive")
  print(file=sys.stderr)

  # ---------- report ----------
  out.append(f"Route {route}: {len(segs)} segments, cruise engaged {engaged_time / 3600:.2f} h"
             + (f" ({qlog_segments} segments only had qlogs, 10 Hz)" if qlog_segments else ""))
  out.append(f"Reset threshold {threshold}; stretch = cruise engaged with no column torque above it\n")

  out.append(f"Car warnings (DAW hands-off request or SCC byte 9): {len(warnings)}")
  out.extend("  " + w for w in warnings)

  out.append("\nWhat ended each stretch:")
  for cause, n in reset_causes.most_common():
    out.append(f"  {cause:<24} {n}")
  other = Counter(r for _, _, r, _ in stretches if r not in reset_causes)
  for cause, n in other.most_common():
    out.append(f"  {cause:<24} {n}")

  bins = [(0, 20), (20, 40), (40, 55), (55, 65), (65, 90), (90, 180), (180, 1e9)]
  out.append("\nStretch length   count   with warning")
  for lo, hi in bins:
    inbin = [s for s in stretches if lo <= s[0] < hi]
    label = f"{lo:.0f}-{hi:.0f}s" if hi < 1e9 else f">{lo:.0f}s"
    out.append(f"  {label:<14} {len(inbin):>5}   {sum(s[1] for s in inbin):>5}")

  long_quiet = sorted((s for s in stretches if s[0] >= 65 and not s[1]), key=lambda s: -s[0])
  out.append(f"\nStretches >= 65 s with NO warning (these contradict the 60 s timer): {len(long_quiet)}")
  out.extend("  " + s[3] for s in long_quiet[:20])

  listed = sorted((s for s in stretches if s[0] >= MIN_REPORT_S), key=lambda s: s[3])
  out.append(f"\nAll stretches >= {MIN_REPORT_S:.0f} s in drive order: {len(listed)}")
  out.extend("  " + s[3] for s in listed)
  return "\n".join(out)


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("route", nargs="?", help="route name, e.g. 000001e5--abc123 (default: most recent)")
  parser.add_argument("--list", action="store_true", help="list recent drives and exit")
  parser.add_argument("--root", default=REALDATA, help="log directory")
  parser.add_argument("--threshold", type=float, default=TORQUE_THRESHOLD)
  parser.add_argument("--force", action="store_true", help="run even while onroad")
  args = parser.parse_args()

  if args.list:
    list_routes(args.root)
    return

  if not args.force:
    try:
      from openpilot.common.params import Params
      if Params().get_bool("IsOnroad"):
        sys.exit("The car is on. Run this with the car off so it doesn't compete with openpilot for CPU.")
    except ImportError:
      pass
  try:
    os.nice(19)
  except OSError:
    pass

  routes = segment_dirs(args.root)
  if not routes:
    sys.exit(f"No drives found in {args.root}")
  route = args.route or max(routes, key=lambda r: routes[r][min(routes[r])].stat().st_mtime)
  if route not in routes:
    sys.exit(f"Route {route} not found. Use --list to see available drives.")

  print(f"Analysing {route} ({len(routes[route])} segments)...", file=sys.stderr)
  report = analyse(route, routes[route], args.threshold)
  print(report)

  out_path = Path(OUT_DIR) / f"hands_on_route_{route}.txt"
  try:
    out_path.write_text(report + "\n")
    print(f"\nSaved to {out_path}")
  except OSError:
    pass


if __name__ == "__main__":
  main()

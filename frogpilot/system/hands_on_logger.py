#!/usr/bin/env python3
"""Records predictions and outcomes for the stock hands-on-wheel speed limiter.

Genesis/Hyundai/Kia vehicles run a hands-on-wheel timer in the stock camera. When
it expires the cluster shows a warning and the car limits speed until the driver
applies steering torque. On a 2022 EV6 the warning is carried in SCC_CONTROL
(0x1A0) byte 9 == 0x60, alongside bytes 20/22, and openpilot only receives it.

This process predicts the warning from the "quiet time" since the driver last
applied real column torque, then records whether the warning actually followed.
The resulting CSV gives labeled hits and misses so the trigger condition can be
identified without hunting through full logs.

Writes /data/media/0/hands_on_events.csv. Diagnostic only: it reads state and
never sends anything to the car.
"""
import os
from collections import deque

import cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog


LOG_PATH = "/data/media/0/hands_on_events.csv"

WARNING_ADDR = 0x1A0
WARNING_BUS = 2
WARNING_BYTE = 9
WARNING_VALUE = 0x60

TORQUE_THRESHOLD = 240  # measured: ~243 resets the stock timer, ~156 does not
PREDICT_AFTER = 45.0    # quiet seconds before we predict a warning
WATCH_WINDOW = 60.0     # seconds to watch for the warning after predicting
WINDOW_S = 60.0         # rolling window the logged conditions describe

HEADER = ("event,quiet_time_s,time_to_warning_s,v_ego_mph,set_speed_mph,"
          "angle_mean,angle_max,driver_tq_mean,driver_tq_max,eps_tq_mean,eps_tq_max,"
          "lat_active_frac\n")


class Window:
  """Rolling stats over the most recent WINDOW_S of driving.

  A rolling window keeps hit and miss rows comparable: both describe the
  conditions leading into the decision point, rather than everything since the
  driver last touched the wheel (which can be many minutes on a hands-off drive).
  """
  def __init__(self):
    self.samples = deque(maxlen=int(WINDOW_S / DT_CTRL))

  def reset(self):
    self.samples.clear()

  def add(self, angle, tq, eps, lat_active):
    self.samples.append((abs(angle), abs(tq), abs(eps), 1 if lat_active else 0))

  def fields(self):
    if not self.samples:
      return "0,0,0,0,0,0,0"
    n = len(self.samples)
    ang = [s[0] for s in self.samples]
    tq = [s[1] for s in self.samples]
    eps = [s[2] for s in self.samples]
    lat = sum(s[3] for s in self.samples)
    return (f"{sum(ang)/n:.2f},{max(ang):.2f},"
            f"{sum(tq)/n:.0f},{max(tq):.0f},"
            f"{sum(eps)/n:.0f},{max(eps):.0f},"
            f"{lat/n:.2f}")


def write_row(row):
  try:
    new = not os.path.exists(LOG_PATH)
    with open(LOG_PATH, "a") as f:
      if new:
        f.write(HEADER)
      f.write(row + "\n")
  except OSError:
    cloudlog.exception("hands_on_logger: failed to write")


def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  sm = messaging.SubMaster(['carState', 'carControl'])
  can_sock = messaging.sub_sock('can', timeout=20)

  quiet_time = 0.0
  window = Window()
  predicted = False
  predicted_quiet = 0.0
  watch_time = 0.0
  warning_time = 0.0
  warning_prev = False

  while True:
    can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
    sm.update(0)

    if not sm.updated['carState']:
      continue

    cs = sm['carState']
    cc = sm['carControl']

    # current state of the car's warning bit
    warning = False
    for evt in can_msgs:
      for m in evt.can:
        if m.src == WARNING_BUS and m.address == WARNING_ADDR and len(m.dat) > WARNING_BYTE:
          warning = m.dat[WARNING_BYTE] == WARNING_VALUE

    mph = cs.vEgo * 2.23694
    set_mph = cs.cruiseState.speed * 2.23694
    window.add(cs.steeringAngleDeg, cs.steeringTorque, cs.steeringTorqueEps, cc.latActive)

    # a real nudge resets the car's timer (and ours)
    if abs(cs.steeringTorque) > TORQUE_THRESHOLD:
      if predicted:
        write_row(f"reset,{predicted_quiet:.1f},,{mph:.1f},{set_mph:.1f},{window.fields()}")
      quiet_time = 0.0
      predicted = False
      watch_time = 0.0
    else:
      quiet_time += DT_CTRL

    # log every warning onset, whether or not we predicted it
    if warning and not warning_prev:
      ttw = f"{watch_time:.1f}" if predicted else ""
      label = "hit" if predicted else "unpredicted"
      write_row(f"{label},{quiet_time:.1f},{ttw},{mph:.1f},{set_mph:.1f},{window.fields()}")
      predicted = False
      watch_time = 0.0
      warning_time = 0.0
    elif warning:
      warning_time += DT_CTRL
    elif warning_prev:
      write_row(f"cleared,{quiet_time:.1f},{warning_time:.1f},{mph:.1f},{set_mph:.1f},{window.fields()}")
      warning_time = 0.0
    warning_prev = warning

    # open a prediction once the car has been quiet long enough
    if not predicted and quiet_time >= PREDICT_AFTER and cc.latActive and not cs.standstill:
      predicted = True
      predicted_quiet = quiet_time
      watch_time = 0.0

    # close it out if the warning never came
    if predicted:
      watch_time += DT_CTRL
      if watch_time >= WATCH_WINDOW:
        write_row(f"miss,{quiet_time:.1f},,{mph:.1f},{set_mph:.1f},{window.fields()}")
        predicted = False
        watch_time = 0.0


if __name__ == "__main__":
  main()

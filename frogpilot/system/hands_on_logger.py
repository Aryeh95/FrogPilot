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

HEADER = ("event,quiet_time_s,time_to_warning_s,v_ego_mph,set_speed_mph,"
          "angle_mean,angle_max,driver_tq_mean,driver_tq_max,eps_tq_mean,eps_tq_max,"
          "lat_active_frac\n")


class Window:
  """Running stats over the current quiet period."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.n = 0
    self.angle_sum = self.angle_max = 0.0
    self.tq_sum = self.tq_max = 0.0
    self.eps_sum = self.eps_max = 0.0
    self.lat_active = 0

  def add(self, angle, tq, eps, lat_active):
    self.n += 1
    a, t, e = abs(angle), abs(tq), abs(eps)
    self.angle_sum += a; self.angle_max = max(self.angle_max, a)
    self.tq_sum += t; self.tq_max = max(self.tq_max, t)
    self.eps_sum += e; self.eps_max = max(self.eps_max, e)
    self.lat_active += 1 if lat_active else 0

  def fields(self):
    n = max(self.n, 1)
    return (f"{self.angle_sum/n:.2f},{self.angle_max:.2f},"
            f"{self.tq_sum/n:.0f},{self.tq_max:.0f},"
            f"{self.eps_sum/n:.0f},{self.eps_max:.0f},"
            f"{self.lat_active/n:.2f}")


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
      window.reset()
    else:
      quiet_time += DT_CTRL

    # log every warning onset, whether or not we predicted it
    if warning and not warning_prev:
      ttw = f"{watch_time:.1f}" if predicted else ""
      label = "hit" if predicted else "unpredicted"
      write_row(f"{label},{quiet_time:.1f},{ttw},{mph:.1f},{set_mph:.1f},{window.fields()}")
      predicted = False
      watch_time = 0.0
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

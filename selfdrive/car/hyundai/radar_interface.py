import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.common.params import Params
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.hyundai.values import DBC

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# CAN-FD platforms broadcast their front radar's tracked objects natively (no UDS
# reconfiguration required). Message layout reverse-engineered by sunnypilot:
# https://github.com/sunnypilot/sunnypilot/tree/hyundai-radar-tracks
# Non-HDA2 cars carry the stream on A-CAN mapped to bus 1; HDA2 cars map it to bus 0.
CANFD_RADAR_START_ADDR = 0x210
CANFD_RADAR_MSG_COUNT = 16
CANFD_RADAR_BUS = 1
CANFD_RADAR_DBC = "hyundai_canfd_radar_210_21f_generated"

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP):
  if DBC[CP.carFingerprint]['radar'] is None:
    return None

  if DBC[CP.carFingerprint]['radar'] == CANFD_RADAR_DBC:
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(CANFD_RADAR_START_ADDR, CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT)]
    return CANParser(CANFD_RADAR_DBC, messages, CANFD_RADAR_BUS)

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.canfd_radar = DBC[CP.carFingerprint]['radar'] == CANFD_RADAR_DBC
    if self.canfd_radar:
      self.trigger_msg = CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT - 1
      # when tracks feed lead fusion, exclude unknown-motion tracks (matches sunnypilot's filter)
      self.motion_filter = Params().get_bool("HyundaiRadarTracksFusion")
    else:
      self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
      self.motion_filter = False
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update_canfd() if self.canfd_radar else self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = car.RadarData.RadarPoint.new_message()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].measured = True
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']
        self.pts[addr].aRel = msg['REL_ACCEL']
        self.pts[addr].yvRel = float('nan')

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_canfd(self):
    ret = car.RadarData.new_message()

    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for addr in range(CANFD_RADAR_START_ADDR, CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      for slot, prefix in enumerate(("T1_", "T2_")):
        key = (addr - CANFD_RADAR_START_ADDR) * 2 + slot

        state = int(msg[f"{prefix}STATE"])
        if state == 0:
          # compact-dialect radars leave STATE zero and use the compressed lifecycle field
          state = {0: 0, 1: 1, 2: 3, 3: 4}[int(msg[f"{prefix}STATE_ALT"])]

        # 3 = measured, 4 = coasted/predicted
        valid = state in (3, 4)
        if valid and self.motion_filter:
          # 1 = stationary, 2 = moving; 0 = unknown is excluded from fusion
          valid = int(msg[f"{prefix}MOTION_STATE"]) in (1, 2)

        if valid:
          if key not in self.pts:
            self.pts[key] = car.RadarData.RadarPoint.new_message()
            self.pts[key].trackId = self.track_id
            self.track_id += 1

          self.pts[key].measured = True
          self.pts[key].dRel = msg[f"{prefix}LONG_DIST"]
          self.pts[key].yRel = msg[f"{prefix}LAT_DIST"]
          self.pts[key].vRel = msg[f"{prefix}REL_SPEED"]
          self.pts[key].aRel = msg[f"{prefix}REL_ACCEL"]
          self.pts[key].yvRel = msg[f"{prefix}REL_LAT_SPEED"]
        elif key in self.pts:
          del self.pts[key]

    ret.points = list(self.pts.values())
    return ret

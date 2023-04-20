# hard-forked from https://github.com/commaai/openpilot/tree/05b37552f3a38f914af41f44ccc7c633ad152a15/selfdrive/car/hyundai/values.py
from cereal import car
from selfdrive.car import dbc_dict
Ecu = car.CarParams.Ecu

# Steer torque limits
class CarControllerParams:
  ACCEL_MIN = -3.5 # m/s
  ACCEL_MAX = 2.0 # m/s

  def __init__(self, CP):
    # To determine the limit for your car, find the maximum value that the stock LKAS will request.
    # If the max stock LKAS request is <384, add your car to this list.
    if CP.carFingerprint in (CAR.IONIQ):
      self.STEER_MAX = 255
    else:
      self.STEER_MAX = 384
    self.STEER_DELTA_UP = 3
    self.STEER_DELTA_DOWN = 7
    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1

class CAR:
  # Hyundai
  IONIQ = "HYUNDAI IONIQ HYBRID 2017-2019"


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4


FW_VERSIONS = {
  CAR.IONIQ: {
    (Ecu.fwdRadar, 0x7d0, None): [
      b'\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ',
    ],
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00AE  MDPS C 1.00 1.07 56310/G2301 4AEHC107',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00AEH MFC  AT EUR LHD 1.00 1.00 95740-G2400 180222',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x816H6F2051\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x816U3H1051\x00\x00\xf1\x006U3H0_C2\x00\x006U3H1051\x00\x00HAE0G16US2\x00\x00\x00\x00',
    ],
  }
}

HYBRID_CAR = CAR.IONIQ  # these cars use a different gas signal

# these cars require a special panda safety mode due to missing counters and checksums in the messages
LEGACY_SAFETY_MODE_CAR = CAR.IONIQ

# If 0x500 is present on bus 1 it probably has a Mando radar outputting radar points.
# If no points are outputted by default it might be possible to turn it on using  selfdrive/debug/hyundai_enable_radar_points.py
DBC = {
  CAR.IONIQ: dbc_dict('hyundai_kia_generic', None),
}

STEER_THRESHOLD = 150

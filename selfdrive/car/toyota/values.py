# hard-forked from https://github.com/commaai/openpilot/tree/05b37552f3a38f914af41f44ccc7c633ad152a15/selfdrive/car/toyota/values.py
from collections import defaultdict
from enum import IntFlag

from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car import dbc_dict

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS
PEDAL_TRANSITION = 10. * CV.MPH_TO_MS


class CarControllerParams:
  ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
  ACCEL_MIN = -3.5  # m/s2

  STEER_MAX = 1500
  STEER_DELTA_UP = 10       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor


class ToyotaFlags(IntFlag):
  HYBRID = 1


class CAR:
  # Toyota
  SEDONA = "KIA SEDONA 2004"

# (addr, cars, bus, 1/freq*100, vl)
TSS2_RADAR_INIT = [
  (0x128, 0,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, 0,   2, b'\x00\x00\x00\x46'),
  (0x160, 0,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, 0,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x283, 0,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x344, 0,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, 0,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, 0,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]

FW_VERSIONS = {
  CAR.SEDONA: {
    (Ecu.engine, 0x7e0, None): [
      b'\x0230ZC2000\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230ZC2100\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230ZC2200\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230ZC2300\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230ZC3000\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230ZC3100\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230ZC3200\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0230ZC3300\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x0330ZC1200\x00\x00\x00\x00\x00\x00\x00\x0050212000\x00\x00\x00\x00\x00\x00\x00\x00895231203202\x00\x00\x00\x00',
    ],
    (Ecu.dsu, 0x791, None): [
      b'881510201100\x00\x00\x00\x00',
      b'881510201200\x00\x00\x00\x00',
    ],
    (Ecu.esp, 0x7b0, None): [
      b'F152602190\x00\x00\x00\x00\x00\x00',
      b'F152602191\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x7a1, None): [
      b'8965B02181\x00\x00\x00\x00\x00\x00',
      b'8965B02191\x00\x00\x00\x00\x00\x00',
      b'8965B48150\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x750, 0xf): [
      b'8821F4702100\x00\x00\x00\x00',
      b'8821F4702300\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'8646F0201101\x00\x00\x00\x00',
      b'8646F0201200\x00\x00\x00\x00',
    ],
  }
}

STEER_THRESHOLD = 100

DBC = {
  CAR.SEDONA: dbc_dict('toyota_new_mc_pt_generated', 'toyota_adas')
}

# These cars have non-standard EPS torque scale factors. All others are 73
EPS_SCALE = defaultdict(lambda: 73, {CAR.SEDONA: 73})

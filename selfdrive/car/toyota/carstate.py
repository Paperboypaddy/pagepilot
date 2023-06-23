# hard-forked from https://github.com/commaai/openpilot/tree/05b37552f3a38f914af41f44ccc7c633ad152a15/selfdrive/car/toyota/carstate.py
from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import mean
from common.filter_simple import FirstOrderFilter
import cereal.messaging as messaging
from common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, EPS_SCALE


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    poller = messaging.Poller()

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.low_speed_lockout = False
    self.acc_type = 1
    self.joystick_sock = messaging.sub_sock('testJoystick', conflate=True, poller=poller)

  def update(self, cp):
    ret = car.CarState.new_message()

    self.joystick = messaging.recv_one(self.joystick_sock)

    ret.doorOpen = False
    ret.seatbeltUnlatched = False
    ret.parkingBrake = False

    ret.brakePressed = False
    ret.brakeHoldActive = False

    # TODO: find a new, common signal
    ret.gas = 0
    ret.gasPressed = False

    ret.wheelSpeeds = self.get_wheel_speeds(
      72,
      72,
      72,
      72,
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.001

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    # On some cars, the angle measurement is non-zero while initializing

    if self.accurate_steer_angle_seen:
      # Offset seems to be invalid for large steering angles
      if abs(ret.steeringAngleDeg) < 90 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    ret.gearShifter = self.parse_gear_shifter('D')
    ret.leftBlinker = False
    ret.rightBlinker = False

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerFaultTemporary = False


    ret.cruiseState.available = True
    ret.cruiseState.speed = 25

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1

    self.pcm_acc_status = 8

    ret.cruiseState.standstill = self.pcm_acc_status == 7
    if  self.joystick.testJoystick.buttons[1]:
      ret.cruiseState.enabled = True
    ret.cruiseState.nonAdaptive = 8 in (1, 2, 3, 4, 5, 6)

    ret.genericToggle = False
    ret.stockAeb = False

    ret.espDisabled = False
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]["LKA_STATE"]

    if self.CP.enableBsm:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR"),
      ("STEER_RATE", "STEER_ANGLE_SENSOR"),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR"),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR"),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR"),
      ("STEER_ANGLE_INITIALIZING", "STEER_TORQUE_SENSOR"),
      ("LKA_STATE", "EPS_STATUS"),

    ]

    checks = [
      ("EPS_STATUS", 25),
      ("STEER_ANGLE_SENSOR", 80),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    # add gas interceptor reading if we are using it

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

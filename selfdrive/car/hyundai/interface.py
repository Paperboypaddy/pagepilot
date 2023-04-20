# hard-forked from https://github.com/commaai/openpilot/tree/05b37552f3a38f914af41f44ccc7c633ad152a15/selfdrive/car/hyundai/interface.py
from cereal import car
from panda import Panda
from common.params import Params
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, Buttons, CarControllerParams
from selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.disable_ecu import disable_ecu

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]
    ret.radarOffCan = RADAR_START_ADDR not in fingerprint[1]

    # WARNING: disabling radar also disables AEB (and we show the same warning on the instrument cluster as if you manually disabled AEB)
    ret.openpilotLongitudinalControl = Params().get_bool("DisableRadar") and (candidate not in LEGACY_SAFETY_MODE_CAR)

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.4
    tire_stiffness_factor = 1.

    ret.stoppingControl = True
    ret.vEgoStopping = 1.0

    ret.longitudinalTuning.kpV = [0.1]
    ret.longitudinalTuning.kiV = [0.0]
    ret.stopAccel = 0.0

    ret.longitudinalActuatorDelayUpperBound = 1.0 # s

    
    if candidate in (CAR.IONIQ,):
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1490. + STD_CARGO_KG  # weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    else:
      raise ValueError(f"Unsupported car: {candidate}")

    # these cars require a special panda safety mode due to missing counters and checksums in the messages
    if candidate in LEGACY_SAFETY_MODE_CAR:
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]

    # set appropriate safety param for gas signal
    if candidate in HYBRID_CAR:
      ret.safetyConfigs[0].safetyParam = 2

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableBsm = 0x58b in fingerprint[0]

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_LONG

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl:
      disable_ecu(logcan, sendcan, addr=0x7d0, com_cont_req=b'\x28\x83\x01')

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise)

    if self.CS.brake_error:
      events.add(EventName.brakeUnavailable)

    if self.CS.CP.openpilotLongitudinalControl:
      buttonEvents = []

      if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
        be = car.CarState.ButtonEvent.new_message()
        be.type = ButtonType.unknown
        if self.CS.cruise_buttons != 0:
          be.pressed = True
          but = self.CS.cruise_buttons
        else:
          be.pressed = False
          but = self.CS.prev_cruise_buttons
        if but == Buttons.RES_ACCEL:
          be.type = ButtonType.accelCruise
        elif but == Buttons.SET_DECEL:
          be.type = ButtonType.decelCruise
        elif but == Buttons.GAP_DIST:
          be.type = ButtonType.gapAdjustCruise
        elif but == Buttons.CANCEL:
          be.type = ButtonType.cancel
        buttonEvents.append(be)

        ret.buttonEvents = buttonEvents

        for b in ret.buttonEvents:
          # do enable on both accel and decel buttons
          if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
            events.add(EventName.buttonEnable)
          # do disable on button down
          if b.type == ButtonType.cancel and b.pressed:
            events.add(EventName.buttonCancel)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c, self.CS, self.frame, c.actuators, c.cruiseControl.cancel, hud_control.visualAlert,
                         hud_control.setSpeed, hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                         hud_control.leftLaneDepart, hud_control.rightLaneDepart)
    self.frame += 1
    return ret

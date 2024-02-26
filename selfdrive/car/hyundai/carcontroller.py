# hard-forked from https://github.com/commaai/openpilot/tree/05b37552f3a38f914af41f44ccc7c633ad152a15/selfdrive/car/hyundai/carcontroller.py
import datetime
from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from common.conversions import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfahda_mfc, create_acc_commands, create_acc_opt, create_frt_radar_opt
from selfdrive.car.hyundai.values import Buttons, CarControllerParams, CAR
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

def debug(message):
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    now_milliseconds = int(now[-6:]) // 1000  # extract microseconds and convert to milliseconds
    debug_message = f"[DEBUG] {now[:-6]}{now_milliseconds:03d}: {message}"
    print(debug_message)

def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.p = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)

    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.last_resume_frame = 0
    self.accel = 0

  def update(self, c, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed,
             left_lane, right_lane, left_lane_depart, right_lane_depart):
    # Steering Torque
    new_steer = int(round(actuators.steer * self.p.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    if not c.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning = \
      process_hud_alert(c.enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    #if CS.CP.openpilotLongitudinalControl:
    #  if (frame % 100) == 0:
    #    can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    
    # can_sends.append(create_lkas11(self.packer, frame, apply_steer, c.latActive,))


    if not CS.CP.openpilotLongitudinalControl:
     if pcm_cancel_cmd:
       can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))
     elif CS.out.cruiseState.standstill:
       # send resume at a max freq of 10Hz
       if (frame - self.last_resume_frame) * DT_CTRL > 0.1:
         # send 25 messages at a time to increases the likelihood of resume being accepted
         can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)] * 25)
         self.last_resume_frame = frame

    if frame % 2 == 0 and CS.CP.openpilotLongitudinalControl:
     lead_visible = False
     accel = actuators.accel if c.longActive else 0

     jerk = clip(2.0 * (accel - CS.out.aEgo), -12.7, 12.7)

     if accel < 0:
       accel = interp(accel - CS.out.aEgo, [-1.0, -0.5], [2 * accel, accel])

     accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

     stopping = (actuators.longControlState == LongCtrlState.stopping)
     set_speed_in_units = hud_speed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
     can_sends.extend(create_acc_commands(self.packer, c.enabled, accel, jerk, int(frame / 2), lead_visible, set_speed_in_units, stopping))
     self.accel = accel

    # 20 Hz LFA MFA message
    ##if frame % 5 == 0 and self.car_fingerprint in (CAR.IONIQ,):
    ##  can_sends.append(create_lfahda_mfc(self.packer, c.enabled))

    # 5 Hz ACC options
    ##if frame % 20 == 0 and CS.CP.openpilotLongitudinalControl:
    ##  can_sends.extend(create_acc_opt(self.packer))

    # 2 Hz front radar options
    #if frame % 50 == 0 and CS.CP.openpilotLongitudinalControl:
    #  can_sends.append(create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    # new_actuators.steer = apply_steer / self.p.STEER_MAX
    new_actuators.accel = self.accel

    return new_actuators, can_sends

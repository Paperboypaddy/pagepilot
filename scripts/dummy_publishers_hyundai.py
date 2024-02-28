import cereal.messaging as messaging
from cereal import log
import threading
from opendbc.can.packer import CANPacker
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car import crc8_pedal
from common.realtime import DT_DMON
from selfdrive.car.hyundai.values import Buttons
import time

packer = CANPacker("hyundai_kia_generic")

def can_function(pm, speed, angle, idx, cruise_button, is_engaged):

  msg = []

  # *** powertrain bus ***

  speed = speed * 3.6 # convert m/s to kph
  msg.append(packer.make_can_msg("WHL_SPD11", 2, {
    "WHL_SPD_FL": speed,
    "WHL_SPD_FR": speed,
    "WHL_SPD_RL": speed,
    "WHL_SPD_RR": speed
  }))

  msg.append(packer.make_can_msg("CLU11", 2, {"CF_Clu_CruiseSwState": cruise_button}))

  msg.append(packer.make_can_msg("E_EMS11", 2, {"CR_Vcu_AccPedDep_Pos": 0}))

  msg.append(packer.make_can_msg("CLU15", 2, {"CF_Clu_Gear": 8}))
  msg.append(packer.make_can_msg("CGW1", 2, {"CF_Gway_DrvSeatBeltSw": 1}))
  msg.append(packer.make_can_msg("SAS11", 2, {"SAS_Angle": angle}))
  msg.append(packer.make_can_msg("TCS13", 2, {"StandStill": 1 if speed >= 1.0 else 0, "ACCEnable": 0,"ACC_REQ": 1, "DriverBraking": 0, "PBRAKE_ACT": 0}))
  msg.append(packer.make_can_msg("TCS15", 2, {"AVH_LAMP": 0}))
  msg.append(packer.make_can_msg("MDPS12", 2, {"CF_Mdps_ToiUnavail": 0, "CF_Mdps_ToiFlt": 0}))
  msg.append(packer.make_can_msg("ESP12", 2, {})) # Blank for now
  msg.append(packer.make_can_msg("CGW2", 2, {})) # Blank
  msg.append(packer.make_can_msg("CGW4", 2, {})) # Blank

  # *** cam bus ***

  pm.send('can', can_list_to_can_capnp(msg))

pm = messaging.PubMaster(["can"])

def can_function_runner(exit_event: threading.Event):
  i = 0
  while not exit_event.is_set():
    cruise_button = Buttons.RES_ACCEL
    if i % 500 == 0:
        cruise_button = 0
    can_function(pm, 20, 0.1, i, cruise_button, True)
    time.sleep(0.009)
    i += 1

def panda_state_function(exit_event: threading.Event):
  pm = messaging.PubMaster(['pandaStates'])
  while not exit_event.is_set():
    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': True,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'hyundai',
    }
    pm.send('pandaStates', dat)
    time.sleep(0.5)

def peripheral_state_function(exit_event: threading.Event):
  pm = messaging.PubMaster(['peripheralState'])
  while not exit_event.is_set():
    dat = messaging.new_message('peripheralState')
    dat.valid = True
    # fake peripheral state data
    dat.peripheralState = {
      'pandaType': log.PandaState.PandaType.blackPanda,
      'voltage': 12000,
      'current': 5678,
      'fanSpeedRpm': 1000
    }
    pm.send('peripheralState', dat)
    time.sleep(0.5)

def fake_driver_monitoring(exit_event: threading.Event):
  pm = messaging.PubMaster(['driverState', 'driverMonitoringState'])
  while not exit_event.is_set():
    # dmonitoringmodeld output
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    pm.send('driverState', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "faceDetected": True,
      "isDistracted": False,
      "awarenessStatus": 1.,
    }
    pm.send('driverMonitoringState', dat)

    time.sleep(DT_DMON)


if __name__ == "__main__":
    threads = []
    exit_event = threading.Event()
    threads.append(threading.Thread(target=panda_state_function, args=(exit_event,)))
    threads.append(threading.Thread(target=peripheral_state_function, args=(exit_event,)))
    threads.append(threading.Thread(target=fake_driver_monitoring, args=(exit_event,)))
    threads.append(threading.Thread(target=can_function_runner, args=(exit_event,)))

    for t in threads:
      t.start()

    for t in reversed(threads):
        t.join()


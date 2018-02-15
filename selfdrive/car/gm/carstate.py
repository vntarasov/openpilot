import common.numpy_fast as np
from common.realtime import sec_since_boot

import selfdrive.messaging as messaging
from selfdrive.config import Conversions as CV

from selfdrive.can.parser import CANParser

class VoltCanBus:
  volt_can_forwarding = True
  if volt_can_forwarding:
    # Object/obstacle detection CAN is all we got.
    # For carstate, powertrain messages are
    # forwarded to it.
    # for carcontrol, messages that openpilot sends
    # on it are forwarded to either chassis or
    # powertrain based on message address
    powertrain = 0
    obstacle = 0
    chassis = 0

    voltboard = True
    if voltboard:
      sw_gmlan = 1
    else:
      sw_gmlan = 3
  else:
    powertrain = 0
    obstacle = 1
    chassis = 2
    sw_gmlan = 3

# Car button codes
class CruiseButtons:
  UNPRESS     = 2
  RES_ACCEL   = 4
  DECEL_SET   = 6
  CANCEL      = 12
  MAIN        = 10

def get_powertrain_can_parser():
  # this function generates lists for signal, messages and initial values
  dbc_f = 'gm_global_a_powertrain'
  signals = [
    # sig_name, sig_address, default
    ("RegenPaddle", 189, 0),
    ("BrakePedalPosition", 241, 0),
    ("FrontLeftDoor", 298, 0),
    ("FrontRightDoor", 298, 0),
    ("RearLeftDoor", 298, 0),
    ("RearRightDoor", 298, 0),
    ("LeftSeatBelt", 298, 0),
    ("RightSeatBelt", 298, 0),
    ("TurnSignals", 320, 0),
    ("SteeringWheelAngle", 485, 0),
    ("FLWheelSpd", 840, 0),
    ("FRWheelSpd", 840, 0),
    ("RLWheelSpd", 842, 0),
    ("RRWheelSpd", 842, 0),
    ("PRNDL", 309, 0),
    ("LKADriverAppldTrq", 388, 0),
    ("LKATorqueDeliveredStatus", 388, 0)
  ]

  return CANParser(dbc_f, signals, [], VoltCanBus.powertrain)

def get_lowspeed_can_parser():
  # this function generates lists for signal, messages and initial values
  dbc_f = 'gm_global_a_lowspeed'
  signals = [
    ("CruiseButtons", 276135936, 2),
    ("LKAGapButton", 276127744, 0),
    ("GasPedal", 271360000, 0)
  ]

  return CANParser(dbc_f, signals, [], VoltCanBus.sw_gmlan)

class CarState(object):
  def __init__(self, CP):
    self.brake_only = CP.enableCruise

    # initialize can parsers
    self.powertrain_cp = get_powertrain_can_parser()
    self.lowspeed_cp = get_lowspeed_can_parser()

    self.car_gas = 0

    self.cruise_buttons = CruiseButtons.UNPRESS
    self.lkas_gap_buttons = 0

    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False

  def update(self):
    self.powertrain_cp.update(int(sec_since_boot() * 1e9), False)
    powertrain_cp = self.powertrain_cp
    self.lowspeed_cp.update(int(sec_since_boot() * 1e9), False)
    lowspeed_cp = self.lowspeed_cp

    self.can_valid = powertrain_cp.can_valid
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = lowspeed_cp.vl[276135936]['CruiseButtons']
    self.lkas_gap_buttons = lowspeed_cp.vl[276127744]['LKAGapButton']

    # CAN bus reading is a bit below dashboard speedometer
    cv = 1.02 / CV.MS_TO_KPH

    self.v_wheel_fl = powertrain_cp.vl[840]['FLWheelSpd'] * cv
    self.v_wheel_fr = powertrain_cp.vl[840]['FRWheelSpd'] * cv
    self.v_wheel_rl = powertrain_cp.vl[842]['RLWheelSpd'] * cv
    self.v_wheel_rr = powertrain_cp.vl[842]['RRWheelSpd'] * cv
    speed_estimate = (self.v_wheel_fl + self.v_wheel_fr +
      self.v_wheel_rl + self.v_wheel_rr) / 4.0

    self.v_ego_raw = self.v_ego = speed_estimate
    self.standstill = self.v_ego == 0.

    self.angle_steers = powertrain_cp.vl[485]['SteeringWheelAngle']
    self.gear_shifter = powertrain_cp.vl[309]['PRNDL']

    self.user_brake = powertrain_cp.vl[241]['BrakePedalPosition']
    # Brake pedal's potentiometer returns near-zero reading
    # even when pedal is not pressed. It seems to be fixed
    # in latest OEM firmware, but it's pretty harmless to leave
    # this compatibility margin in place.
    if self.user_brake <= 5:
      self.user_brake = 0
    self.brake_pressed = self.user_brake > 0

    self.regen_pressed = bool(powertrain_cp.vl[189]['RegenPaddle'])
    # Regen braking is braking
    self.brake_pressed = self.brake_pressed or self.regen_pressed

    self.pedal_gas = lowspeed_cp.vl[271360000]['GasPedal']
    self.user_gas = self.pedal_gas
    self.user_gas_pressed = self.user_gas > 0

    self.steer_torque_driver = powertrain_cp.vl[388]['LKADriverAppldTrq']
    self.steer_override = abs(self.steer_torque_driver) > 1.0

    # 0 - inactive, 1 - active, 2 - temporary limited, 3 - failed
    self.lkas_status = powertrain_cp.vl[388]['LKATorqueDeliveredStatus']

    # 1 - open, 0 - closed
    self.door_all_closed = (powertrain_cp.vl[298]['FrontLeftDoor'] == 0 and
      powertrain_cp.vl[298]['FrontRightDoor'] == 0 and
      powertrain_cp.vl[298]['RearLeftDoor'] == 0 and
      powertrain_cp.vl[298]['RearRightDoor'] == 0)

    # 1 - latched
    self.seatbelt = powertrain_cp.vl[298]['LeftSeatBelt'] == 1

    self.steer_error = False

    self.brake_error = False
    self.esp_disabled = False
    self.main_on = True
    self.can_valid = True

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.left_blinker_on = powertrain_cp.vl[320]['TurnSignals'] == 1
    self.right_blinker_on = powertrain_cp.vl[320]['TurnSignals'] == 2

    # Alow Park (0) and D/L (2)
    self.gear_shifter_valid = self.gear_shifter in [0, 2]


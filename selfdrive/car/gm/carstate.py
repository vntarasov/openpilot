import numpy as np

import selfdrive.messaging as messaging
from common.realtime import sec_since_boot

from selfdrive.car.gm.can_parser import CANParser

def get_can_parser(CP):
  # this function generates lists for signal, messages and initial values
  dbc_f = 'chevy_volt_premier_2017_can.dbc'
  signals = [
    ("GEAR_SHIFTER", 0x102cc040, 0),
    ("STEER_WHEEL_ANGLE", 0x1e5, 0), # Both OBD 6-14 and OBD 12-13
    ("WHEEL_SPEED_FL", 0x106b8040, 0),
    ("WHEEL_SPEED_FR", 0x106b8040, 0),
    ("WHEEL_SPEED_RL", 0x106b8040, 0),
    ("WHEEL_SPEED_RR", 0x106b8040, 0),
    ("CRUISE_BUTTONS", 0x10758040, 2),
    ("LKAS_GAP_BUTTONS", 0x10756040, 0),
    ("GAS_PEDAL", 0x102ca040, 0),
    ("BRAKE_SENSOR", 0x10250040, 0),
  ]
  checks = [
  ]

  return CANParser(dbc_f, signals, checks)

class CarState(object):
  def __init__(self, CP, logcan):
    if CP.carFingerprint != "CHEVROLET VOLT 2017 PREMIER":
      raise ValueError("unsupported car %s" % CP.carFingerprint)

    self.brake_only = CP.enableCruise

    # initialize can parser
    self.cp = get_can_parser(CP)

    self.car_gas = 0

    self.v_wheel = 0.0

    # Unpressed
    self.cruise_buttons = 2
    self.lkas_gap_buttons = 0

  def update(self, can_pub_main):
    cp = self.cp
    cp.update_can(can_pub_main)

    self.can_valid = cp.can_valid
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = cp.vl[0x10758040]['CRUISE_BUTTONS']
    self.lkas_gap_buttons = cp.vl[0x10756040]['LKAS_GAP_BUTTONS']

    # calc best v_ego estimate, by averaging two opposite corners
    speed_estimate = (
      cp.vl[0x106b8040]['WHEEL_SPEED_FL'] + cp.vl[0x106b8040]['WHEEL_SPEED_FR'] +
      cp.vl[0x106b8040]['WHEEL_SPEED_RL'] + cp.vl[0x106b8040]['WHEEL_SPEED_RR']) / 4.0

    self.v_ego = self.v_wheel = speed_estimate

    self.angle_steers = cp.vl[0x1e5]['STEER_WHEEL_ANGLE']
    self.gear_shifter = cp.vl[0x102cc040]['GEAR_SHIFTER']

    self.user_brake = cp.vl[0x10250040]["BRAKE_SENSOR"]
    # Brake pedal's potentiometer returns near-zero reading
    # even when pedal is not pressed
    self.brake_pressed = self.user_brake > 5
    self.pedal_gas = cp.vl[0x102ca040]["GAS_PEDAL"]
    self.user_gas = self.pedal_gas
    self.user_gas_pressed = self.user_gas > 0

    self.steer_override = False

    self.steer_error = False
    self.steer_not_allowed = False
    self.brake_error = False
    self.door_all_closed = True
    self.seatbelt = True
    self.esp_disabled = False
    self.main_on = True
    self.can_valid = True

    # Alow Park (3) and D/L (0)
    self.gear_shifter_valid = self.gear_shifter in [0, 3]


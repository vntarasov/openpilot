import numpy as np
import selfdrive.messaging as messaging
from common.realtime import sec_since_boot
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser

# Car button codes
class CruiseButtons:
  UNPRESS     = 1
  RES_ACCEL   = 2
  DECEL_SET   = 3
  MAIN        = 5
  CANCEL      = 6

def get_powertrain_can_parser(canbus):
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
    ("AcceleratorPedal", 417, 0),
    ("ACCButtons", 481, CruiseButtons.UNPRESS),
    ("SteeringWheelAngle", 485, 0),
    ("FLWheelSpd", 840, 0),
    ("FRWheelSpd", 840, 0),
    ("RLWheelSpd", 842, 0),
    ("RRWheelSpd", 842, 0),
    ("PRNDL", 309, 0),
    ("LKADriverAppldTrq", 388, 0),
    ("LKATorqueDeliveredStatus", 388, 0)
  ]

  return CANParser(dbc_f, signals, [], canbus.powertrain)

class CarState(object):
  def __init__(self, CP, canbus):
    # initialize can parser
    self.powertrain_cp = get_powertrain_can_parser(canbus)

    self.cruise_buttons = CruiseButtons.UNPRESS

    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=np.matrix([[0.], [0.]]),
                         A=np.matrix([[1., dt], [0., 1.]]),
                         C=np.matrix([1., 0.]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.

    #Minimum speed for LKAS to respond to commands (10kph ~6mph)
    self.LKAS_MINIMUM_SPEED_FOR_STEERING = 3.0 #3.0m/s ~= 10.8kph. Good enough.

  def update(self):
    self.powertrain_cp.update(int(sec_since_boot() * 1e9), False)
    powertrain_cp = self.powertrain_cp

    self.can_valid = powertrain_cp.can_valid
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = powertrain_cp.vl[481]['ACCButtons']

    cv = CV.KPH_TO_MS
    self.v_wheel_fl = powertrain_cp.vl[840]['FLWheelSpd'] * cv
    self.v_wheel_fr = powertrain_cp.vl[840]['FRWheelSpd'] * cv
    self.v_wheel_rl = powertrain_cp.vl[842]['RLWheelSpd'] * cv
    self.v_wheel_rr = powertrain_cp.vl[842]['RRWheelSpd'] * cv
    speed_estimate = (self.v_wheel_fl + self.v_wheel_fr +
      self.v_wheel_rl + self.v_wheel_rr) / 4.0

    self.v_ego_raw = speed_estimate
    v_ego_x = self.v_ego_kf.update(speed_estimate)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])

    self.standstill = self.v_ego == 0.

    self.angle_steers = powertrain_cp.vl[485]['SteeringWheelAngle']
    self.gear_shifter = powertrain_cp.vl[309]['PRNDL']

    self.user_brake = powertrain_cp.vl[241]['BrakePedalPosition']
    # Brake pedal's potentiometer returns near-zero reading
    # even when pedal is not pressed.
    if self.user_brake < 10:
      self.user_brake = 0
    self.brake_pressed = self.user_brake > 0

    self.regen_pressed = bool(powertrain_cp.vl[189]['RegenPaddle'])
    # Regen braking is braking
    self.brake_pressed = self.brake_pressed or self.regen_pressed

    self.pedal_gas = powertrain_cp.vl[417]['AcceleratorPedal']
    self.user_gas_pressed = self.pedal_gas > 0

    self.steer_torque_driver = powertrain_cp.vl[388]['LKADriverAppldTrq']
    self.steer_override = abs(self.steer_torque_driver) > 1.0

    # 0 - inactive, 1 - active, 2 - temporary limited, 3 - failed
    self.lkas_status = powertrain_cp.vl[388]['LKATorqueDeliveredStatus']

    #Because we're below the speed threshold, we know that LKAS is limited,
    #However due to the recovery code, the LKATorQueDeliveredStatus is 0 (inactive)
    #
    #Just pretend that it's limited so alerts are properly sent. Allow engage from standstill though.
    if self.v_ego < self.LKAS_MINIMUM_SPEED_FOR_STEERING and self.v_ego > 0.001:
      self.lkas_status = 2 

    # 1 - open, 0 - closed
    self.door_all_closed = (powertrain_cp.vl[298]['FrontLeftDoor'] == 0 and
      powertrain_cp.vl[298]['FrontRightDoor'] == 0 and
      powertrain_cp.vl[298]['RearLeftDoor'] == 0 and
      powertrain_cp.vl[298]['RearRightDoor'] == 0)

    # 1 - latched
    self.seatbelt = powertrain_cp.vl[298]['LeftSeatBelt'] == 1
    

    #If LKAS is not responding to commands and we've above the ignore threshold. Problem
    if self.v_ego > self.LKAS_MINIMUM_SPEED_FOR_STEERING and self.lkas_status > 1: 
      self.steer_error = True
    else:
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


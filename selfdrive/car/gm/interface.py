#!/usr/bin/env python
import time
import common.numpy_fast as np

from common.realtime import sec_since_boot
from selfdrive.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET, get_events
from selfdrive.controls.lib.vehicle_model import VehicleModel
from .carstate import CarState, CruiseButtons
from .carcontroller import CarController

from cereal import car

import selfdrive.messaging as messaging

# Car chimes, beeps, blinker sounds etc
class CM:
  TOCK = 0x81
  TICK = 0x82
  LOW_BEEP = 0x84
  HIGH_BEEP = 0x85
  LOW_CHIME = 0x86
  HIGH_CHIME = 0x87

# GM cars have 4 CAN buses, which creates many ways
# of how the car can be connected to.
# This ia a helper class for the interface to be setup-agnostic.
# Supports single Panda setup (connected to OBDII port),
# and a CAN forwarding setup (connected to camera module connector).
class CanBus(object):
  def __init__(self):
    # Regardless of the setup
    self.powertrain = 0

    # Only subset of powertrain is forwarded in
    # CAN forwarding setup. 190 is not forwarded.
    probe_signal = "GasPedalAndAcc"
    probe_address = 190
    dbc_f = "gm_global_a_powertrain"
    signals = [(probe_signal, probe_address, 0)]
    cp = CANParser(dbc_f, signals, [], self.powertrain)
    time.sleep(0.1)
    cp.update(int(sec_since_boot() * 1e9), False)

    if cp.ts[probe_address][probe_signal] == 0:
      # Object/obstacle detection CAN is all we got.
      # For carstate, powertrain messages are
      # forwarded to it.
      # for carcontrol, messages that openpilot sends
      # on it are forwarded to either chassis or
      # powertrain based on message address
      print "CAN forwarding setup detected"
      self.can_forwarding = True
      self.obstacle = 0
      self.chassis = 0
      self.sw_gmlan = 1
    else:
      print "Panda(s)-on-OBDII setup"
      self.can_forwarding = False
      self.obstacle = 1
      self.chassis = 2
      self.sw_gmlan = 3

class CarInterface(object):
  def __init__(self, CP, sendcan=None):
    self.CP = CP

    self.frame = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.can_invalid_count = 0

    # *** init the major players ***
    canbus = CanBus()
    self.CS = CarState(CP, canbus)
    self.VM = VehicleModel(CP)

    # sending if read only is False
    if sendcan is not None:
      self.sendcan = sendcan
      self.CC = CarController(canbus)

  @staticmethod
  def compute_gb(accel, speed):
    if accel > 0:
      return float(accel) / 3.0
    else:
      return float(accel) / 6.0

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint):
    ret = car.CarParams.new_message()

    ret.carName = "gm"
    ret.carFingerprint = candidate

    ret.enableCruise = False

    # supports stop and go, but initial engage must be above 15mph
    ret.minEnableSpeed = 15 * CV.MPH_TO_MS

    # kg of standard extra cargo to count for drive, gas, etc...
    std_cargo = 136
    ret.mass = 1607 + std_cargo

    ret.safetyModel = car.CarParams.SafetyModels.gm

    ret.wheelbase = 2.69
    ret.steerRatio = 15.7
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.centerToFront = ret.wheelbase * 0.4 # wild guess

    # hardcoding honda civic 2016 touring params so they can be used to
    # scale unknown params for other cars
    mass_civic = 2923./2.205 + std_cargo
    wheelbase_civic = 2.70
    centerToFront_civic = wheelbase_civic * 0.4
    centerToRear_civic = wheelbase_civic - centerToFront_civic
    rotationalInertia_civic = 2500
    tireStiffnessFront_civic = 85400
    tireStiffnessRear_civic = 90000

    centerToRear = ret.wheelbase - ret.centerToFront
    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = rotationalInertia_civic * \
                            ret.mass * ret.wheelbase**2 / (mass_civic * wheelbase_civic**2)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront = tireStiffnessFront_civic * \
                             ret.mass / mass_civic * \
                             (centerToRear / ret.wheelbase) / (centerToRear_civic / wheelbase_civic)
    ret.tireStiffnessRear = tireStiffnessRear_civic * \
                            ret.mass / mass_civic * \
                            (ret.centerToFront / ret.wheelbase) / (centerToFront_civic / wheelbase_civic)

    ret.steerKpBP = [9., 27.]
    ret.steerKpV = [0.1, 0.35]
    ret.steerKiBP = [9., 27.]
    ret.steerKiV = [0.05, 0.1]
    ret.steerKf = 0.00003   # full torque for 20 deg at 80mph means 0.00007818594
    ret.steerMaxBP = [0.] # m/s
    ret.steerMaxV = [1.]
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [1.]
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]
    ret.longPidDeadzoneBP = [0.]
    ret.longPidDeadzoneV = [0.]

    ret.longitudinalKpBP = [0., 10., 35.]
    ret.longitudinalKpV = [4., 2.4, 1.5]
    ret.longitudinalKiBP = [0., 35.]
    ret.longitudinalKiV = [0.54, 0.36]

    ret.steerLimitAlert = True

    ret.stoppingControl = True
    ret.startAccel = 0.8

    ret.steerRateCost = 0.5

    # TODO: use fingerpting to go to passive
    # if stock driver assist ECUs are online.
    ret.enableCamera = True

    return ret

  # returns a car.CarState
  def update(self, c):
    self.CS.update()

    # create message
    ret = car.CarState.new_message()

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gas pedal information.
    ret.gas = self.CS.pedal_gas / 254.0
    ret.gasPressed = self.CS.user_gas_pressed

    # brake pedal
    ret.brake = self.CS.user_brake / 0xd0
    ret.brakePressed = self.CS.brake_pressed

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers

    # torque and user override. Driver awareness
    # timer resets when the user uses the steering wheel.
    ret.steeringPressed = self.CS.steer_override
    ret.steeringTorque = self.CS.steer_torque_driver

    # cruise state
    ret.cruiseState.available = False
    ret.cruiseState.enabled = False
    ret.cruiseState.speed = 0
    ret.cruiseState.speedOffset = 0.
    ret.cruiseState.standstill = False

    ret.leftBlinker = self.CS.left_blinker_on
    ret.rightBlinker = self.CS.right_blinker_on
    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    buttonEvents = []

    # blinkers
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = self.CS.left_blinker_on
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = self.CS.right_blinker_on
      buttonEvents.append(be)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = 'accelCruise'
      elif but == CruiseButtons.DECEL_SET:
        be.type = 'decelCruise'
      elif but == CruiseButtons.CANCEL:
        be.type = 'cancel'
      elif but == CruiseButtons.MAIN:
        be.type = 'altButton3'
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents

    events = []
    if not self.CS.can_valid:
      self.can_invalid_count += 1
      if self.can_invalid_count >= 5:
        events.append(create_event('commIssue', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    else:
      self.can_invalid_count = 0
    if self.CS.steer_error:
      events.append(create_event('steerUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
    if self.CS.lkas_status > 1:
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.WARNING]))
    if self.CS.brake_error:
      events.append(create_event('brakeUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
    if not self.CS.gear_shifter_valid:
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if not self.CS.main_on:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if self.CS.gear_shifter == 3:
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    if (ret.gasPressed and not self.gas_pressed_prev) or \
       (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    # handle button presses
    for b in ret.buttonEvents:

      # do enable on both accel and decel buttons
      if b.type in ["accelCruise", "decelCruise"] and not b.pressed:
        events.append(create_event('buttonEnable', [ET.ENABLE]))

      # do disable on button down
      if b.type == "cancel" and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    ret.events = events

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed

    # cast to reader so it can't be modified
    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
    hud_v_cruise = c.hudControl.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    chime, chime_count = {
      "none": (0, 0),
      "beepSingle": (CM.HIGH_CHIME, 1),
      "beepTriple": (CM.HIGH_CHIME, 3),
      "beepRepeated": (CM.LOW_CHIME, -1),
      "chimeSingle": (CM.LOW_CHIME, 1),
      "chimeDouble": (CM.LOW_CHIME, 2),
      "chimeRepeated": (CM.LOW_CHIME, -1),
      "chimeContinuous": (CM.LOW_CHIME, -1)}[str(c.hudControl.audibleAlert)]

    self.CC.update(self.sendcan, c.enabled, self.CS, self.frame, \
      c.actuators,
      hud_v_cruise, c.hudControl.lanesVisible, \
      c.hudControl.leadVisible, \
      chime, chime_count)

    self.frame += 1


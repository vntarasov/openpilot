#!/usr/bin/env python
import time
import numpy as np

from selfdrive.car.gm.carstate import CarState, CruiseButtons
from selfdrive.car.gm.carcontroller import CarController
from selfdrive.config import Conversions as CV

from cereal import car

import zmq
import selfdrive.messaging as messaging

# Car chimes, beeps, blinker sounds etc
class CM:
  TOCK = 0x81
  TICK = 0x82
  LOW_BEEP = 0x84
  HIGH_BEEP = 0x85
  LOW_CHIME = 0x86
  HIGH_CHIME = 0x87

class CarInterface(object):
  def __init__(self, CP, logcan, sendcan=None):
    self.logcan = logcan
    self.CP = CP

    self.frame = 0
    self.can_invalid_count = 0

    # *** init the major players ***
    self.CS = CarState(CP, self.logcan)

    # sending if read only is False
    if sendcan is not None:
      self.sendcan = sendcan
      self.CC = CarController()

  # returns a car.CarState
  def update(self):
    # ******************* do can recv *******************
    can_powertrain = []
    can_lowspeed = []
    canMonoTimes = []
    for a in messaging.drain_sock(self.logcan):
      canMonoTimes.append(a.logMonoTime)
      # Only radar needs to listen on object detection bus,
      # src 0 from main Neo board, or 0x11 from Panda.

      # TODO: Parsing all needed car state on CANs other than
      # low-speed GMLAN (src 1), so that openpilot would only
      # need to use it to send chimes.

      powertrain_src = 0x10
      lowspeed_src = 1
      for msg in a.can:
        if msg.src == powertrain_src:
          can_powertrain.append((msg.address, msg.busTime, msg.dat, msg.src))
        elif msg.src == lowspeed_src:
          # Drop sender ECU ID
          address = msg.address & 0x1ffff000
          can_lowspeed.append((address, msg.busTime, msg.dat, msg.src))
    self.CS.update(can_powertrain, can_lowspeed)

    # create message
    ret = car.CarState.new_message()

    # speeds
    ret.vEgo = self.CS.v_ego
    cv = 1.0 / CV.MS_TO_KPH
    ret.wheelSpeeds.fl = self.CS.powertrain_cp.vl[840]['FLWheelSpd'] * cv
    ret.wheelSpeeds.fr = self.CS.powertrain_cp.vl[840]['FRWheelSpd'] * cv
    ret.wheelSpeeds.rl = self.CS.powertrain_cp.vl[842]['RLWheelSpd'] * cv
    ret.wheelSpeeds.rr = self.CS.powertrain_cp.vl[842]['RRWheelSpd'] * cv

    # gas pedal information.
    ret.gas = self.CS.pedal_gas / 254.0
    if not self.CP.enableGas:
      ret.gasPressed = self.CS.pedal_gas > 0
    else:
      ret.gasPressed = self.CS.user_gas_pressed

    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed != 0

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers

    # TODO: torque and user override. Need to reset driver awareness
    # timer when the user uses the steering wheel.
    ret.steeringPressed = self.CS.steer_override
    ret.steeringTorque = 1 if ret.steeringPressed else 0

    buttonEvents = []

    # TODO: blinkers
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = self.CS.left_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = self.CS.right_blinker_on != 0
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

    # errors
    errors = []
    if not self.CS.can_valid:
      self.can_invalid_count += 1
      if self.can_invalid_count >= 5:
        errors.append('commIssue')
    else:
      self.can_invalid_count = 0
    if self.CS.steer_error:
      errors.append('steerUnavailable')
    elif self.CS.steer_not_allowed:
      errors.append('steerTemporarilyUnavailable')
    if self.CS.brake_error:
      errors.append('brakeUnavailable')
    if not self.CS.gear_shifter_valid:
      errors.append('wrongGear')
    if not self.CS.door_all_closed:
      errors.append('doorOpen')
    if not self.CS.seatbelt:
      errors.append('seatbeltNotLatched')
    if self.CS.esp_disabled:
      errors.append('espDisabled')
    if not self.CS.main_on:
      errors.append('wrongCarMode')
    if self.CS.gear_shifter == 3:
      errors.append('reverseGear')

    ret.errors = errors
    ret.canMonoTimes = canMonoTimes

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

    pcm_accel = int(np.clip(c.cruiseControl.accelOverride/1.4,0,1)*0xc6)

    self.CC.update(self.sendcan, c.enabled, self.CS, self.frame, \
      c.gas, c.brake, c.steeringTorque, \
      c.cruiseControl.speedOverride, \
      c.cruiseControl.override, \
      c.cruiseControl.cancel, \
      pcm_accel, \
      hud_v_cruise, c.hudControl.lanesVisible, \
      c.hudControl.leadVisible, \
      chime, chime_count)

    self.frame += 1
    return not (c.enabled and not self.CC.controls_allowed)


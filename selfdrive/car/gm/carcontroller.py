from collections import namedtuple

from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot

from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.controls.lib.drive_helpers import rate_limit
from .carstate import CruiseButtons
from . import gmcan


def actuator_hystereses(final_brake, braking, brake_steady, v_ego, civic):
  # hyst params... TODO: move these to VehicleParams
  brake_hyst_on = 0.055 if civic else 0.1    # to activate brakes exceed this value
  brake_hyst_off = 0.005                     # to deactivate brakes below this value
  brake_hyst_gap = 0.01                      # don't change brake command for small ocilalitons within this value

  #*** histeresys logic to avoid brake blinking. go above 0.1 to trigger
  if (final_brake < brake_hyst_on and not braking) or final_brake < brake_hyst_off:
    final_brake = 0.
  braking = final_brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if final_brake == 0.:
    brake_steady = 0.
  elif final_brake > brake_steady + brake_hyst_gap:
    brake_steady = final_brake - brake_hyst_gap
  elif final_brake < brake_steady - brake_hyst_gap:
    brake_steady = final_brake + brake_hyst_gap
  final_brake = brake_steady

  if not civic:
    brake_on_offset_v  = [.25, .15]   # min brake command on brake activation. below this no decel is perceived
    brake_on_offset_bp = [15., 30.]     # offset changes VS speed to not have too abrupt decels at high speeds
    # offset the brake command for threshold in the brake system. no brake torque perceived below it
    brake_on_offset = interp(v_ego, brake_on_offset_bp, brake_on_offset_v)
    brake_offset = brake_on_offset - brake_hyst_on
    if final_brake > 0.0:
      final_brake += brake_offset

  return final_brake, braking, brake_steady

class CarController(object):
  def __init__(self):
    self.braking = False
    self.brake_steady = 0.
    self.final_brake_last = 0.
    self.start_time = sec_since_boot()
    self.chime = 0
    self.lkas_active = False
    self.inhibit_steer_for = 0
    self.steer_idx = 0

    # redundant safety check with the board
    self.controls_allowed = False

  def update(self, sendcan, enabled, CS, frame, actuators, \
             hud_v_cruise, hud_show_lanes, hud_show_car, chime, chime_cnt):
    """ Controls thread """

    # *** apply brake hysteresis ***
    final_brake, self.braking, self.brake_steady = actuator_hystereses(
      actuators.brake, self.braking, self.brake_steady, CS.v_ego, False)

    # no output if not enabled, but keep sending keepalive messages
    final_gas = actuators.gas
    final_steer = actuators.steer
    if not enabled:
      final_gas = 0.
      final_brake = 0.
      final_steer = 0.

    # *** rate limit after the enable check ***
    final_brake = rate_limit(final_brake, self.final_brake_last, -2., 1./100)
    self.final_brake_last = final_brake

    # **** process the car messages ****

    # *** compute control surfaces ***
    tt = sec_since_boot()
    GAS_MAX = 2047

    # If final_brake less than this, do pure regen
    REGEN_ONLY_BRAKE = 0.4

    # Max input final_brake
    BRAKE_SCALE = 1.0

    # More aggressive braking near full stop
    NEAR_STOP_BRAKE_PHASE = 1.3 # m/s. ~3mph

    # This is only a safety limit
    BRAKE_MAX = 0xff

    # As far as DBC goes, there are
    # 11 bits for steering, including 1 bit for sign.
    # 12-th bit is boolean flag LKAS on/off.
    STEER_MAX =  0xff

    # Below that means regen braking
    GAS_OFFSET = 2048

    # Regen braking triggered by ACC is slightly less powerful
    # than by max regen paddle
    MAX_ACC_REGEN = -644 # With offset, raw throttle value 1404

    if final_brake != 0:
      if final_brake < REGEN_ONLY_BRAKE:
        # No friction brakes
        apply_brake = 0
        # Interpolate to MAX_ACC_REGEN at REGEN_ONLY_BRAKE
        apply_gas = int(clip(final_brake * MAX_ACC_REGEN / REGEN_ONLY_BRAKE, MAX_ACC_REGEN, 0))
      else:
        # Max ACC regen
        apply_gas = MAX_ACC_REGEN
        # Remaining by friction brakes
        # Interpolate from 0 at REGEN_ONLY_BRAKE to BRAKE_MAX at full brake
        apply_brake = int(clip(BRAKE_MAX * (final_brake - REGEN_ONLY_BRAKE) / (BRAKE_SCALE - REGEN_ONLY_BRAKE), 0, BRAKE_MAX))

    else:
      apply_brake = 0
      apply_gas = int(clip(final_gas*GAS_MAX, 0, GAS_MAX-1))

    steer = final_steer * STEER_MAX
    apply_steer = int(clip(steer, -STEER_MAX, STEER_MAX))

    # no gas if driver is hitting the brake
    if apply_gas > 0 and CS.brake_pressed:
      print "CANCELLING GAS", apply_brake
      apply_gas = 0

    # no computer brake if the gas is being pressed
    if CS.car_gas > 0 and apply_brake != 0:
      print "CANCELLING BRAKE"
      apply_brake = 0

    if not enabled:
      # Without 'engaged' flag sent, won't actually trigger regen braking
      apply_gas = MAX_ACC_REGEN

    # *** entry into controls state ***
    if (CS.prev_cruise_buttons == CruiseButtons.DECEL_SET or CS.prev_cruise_buttons == CruiseButtons.RES_ACCEL) and \
        CS.cruise_buttons == CruiseButtons.UNPRESS and not self.controls_allowed:
      print "CONTROLS ARE LIVE"
      self.controls_allowed = True

    # *** exit from controls state on cancel, gas, or brake ***
    if (CS.cruise_buttons == CruiseButtons.CANCEL or CS.brake_pressed or
        CS.user_gas_pressed or (CS.pedal_gas > 0 and CS.brake_only)) and self.controls_allowed:
      print "CONTROLS ARE DEAD"
      self.controls_allowed = False

    # Send CAN commands.
    can_sends = []

    # Send ADAS keepalive, 10hz
    adas_keepalive_step = 10
    if frame % adas_keepalive_step == 0:
      can_sends += gmcan.create_adas_keepalive()

    if enabled and CS.lkas_status == 1:
      self.lkas_active = True
    if not enabled:
      self.lkas_active = False

    # Wait for LKAS to become active,
    # before enabling reset logic.
    if self.lkas_active and CS.lkas_status != 1:
      # SCM ignores steering command. Workaround is to
      # temporary disable steering command.
      # Re-enabling after 200ms seems to be working great
      self.lkas_active = False
      if self.inhibit_steer_for == 0:
        self.inhibit_steer_for = 20

    if self.inhibit_steer_for > 0:
      apply_steer = 0
      self.inhibit_steer_for -= 1

    # Stock ASCM refresh rate is at 10Hz when it's not
    # in LKA control and 50Hz when it is.
    steer_active_step = 5 # 20Hz
    steer_inactive_step = 10 # 10Hz
    send_steer = False
    if not enabled or self.inhibit_steer_for > 0:
      send_steer = (frame % steer_inactive_step) == 0
    else:
      send_steer = (frame % steer_active_step) == 0

    if send_steer:
      can_sends.append(gmcan.create_steering_control(apply_steer, self.steer_idx))
      self.steer_idx = (self.steer_idx + 1) % 4

    # Gas/regen and brakes - all at 25Hz
    if (frame % 4) == 0:
      idx = (frame / 4) % 4

      at_full_stop = enabled and CS.v_ego == 0
      near_stop = enabled and (CS.v_ego < NEAR_STOP_BRAKE_PHASE)
      can_sends.append(gmcan.create_friction_brake_command(apply_brake, idx, near_stop, at_full_stop))

      gas_amount = apply_gas + GAS_OFFSET
      at_full_stop = enabled and CS.v_ego == 0
      can_sends.append(gmcan.create_gas_regen_command(gas_amount, idx, enabled, at_full_stop))

    # Send dashboard UI commands (ACC status), 25hz
    if (frame % 4) == 0:
      can_sends.append(gmcan.create_acc_dashboard_command(enabled, hud_v_cruise / CV.MS_TO_KPH, hud_show_car))

    # Radar needs to know current speed and yaw rate (50hz),
    # and that ADAS is alive (10hz)
    time_and_headlights_step = 10
    if frame % time_and_headlights_step == 0:
      idx = (frame / time_and_headlights_step) % 4
      can_sends.append(gmcan.create_adas_time_status(int((tt - self.start_time) * 60), idx))
      can_sends.append(gmcan.create_adas_headlights_status())

    speed_and_accelerometer_step = 2
    if frame % speed_and_accelerometer_step == 0:
      idx = (frame / speed_and_accelerometer_step) % 4
      can_sends.append(gmcan.create_adas_steering_status(idx))
      can_sends.append(gmcan.create_adas_accelerometer_speed_status(CS.v_ego, idx))

    # Send chimes
    if self.chime != chime:
      duration = 0x3c

      # There is no 'repeat forever' chime command
      # TODO: Manage periodic re-issuing of chime command
      # and chime cancellation
      if chime_cnt == -1:
        chime_cnt = 10

      if chime != 0:
        can_sends.append(gmcan.create_chime_command(chime, duration, chime_cnt))

      # If canceling a repeated chime, cancel command must be
      # issued for the same chime type and duration
      self.chime = chime

    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())

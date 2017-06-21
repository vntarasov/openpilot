def create_steering_control(apply_steer, idx):
  apply_steer = apply_steer & 0x7ff
  lkas_enabled = 8 if apply_steer != 0 else 0
  checksum = 0x1000 - (lkas_enabled << 8) - apply_steer - idx
  checksum = checksum & 0xfff
  dat = [(idx << 4) | lkas_enabled | (apply_steer >> 8),
    apply_steer & 0xff, checksum >> 8, checksum & 0xff]
  return [0x180, 0, "".join(map(chr, dat)), 0x10]

def create_adas_keepalive():
  dat = "\x00\x00\x00\x00\x00\x00\x00"
  return [[0x409, 0, dat, 0x10], [0x40a, 0, dat, 0x10]]

def create_gas_regen_command(throttle, idx, acc_engaged, at_full_stop):
  eng_bit = 1 if acc_engaged else 0
  gas_high = (throttle >> 5) | 0x80
  gas_low = (throttle << 3) & 0xff
  full_stop = 0x20 if at_full_stop else 0
  chk1 = (0x100 - gas_high - 1) & 0xff
  chk2 = (0x100 - gas_low - idx) & 0xff
  dat = [(idx << 6) | eng_bit, 0x42 | full_stop, gas_high, gas_low, 1 - eng_bit, 0xbd - full_stop, chk1, chk2]
  return [0x2cb, 0, "".join(map(chr, dat)), 0x10]

def create_friction_brake_command(apply_brake, idx, near_stop, at_full_stop):
  if apply_brake == 0:
    mode = 0x10
  else:
    mode = 0xa0

    if at_full_stop:
      mode = 0xd0
    elif near_stop:
      mode = 0xb0

  brake = (0x1000 - apply_brake) & 0xfff

  checksum = (0x10000 - (mode << 8) - brake - idx) & 0xffff
  chk1 = checksum >> 8
  chk2 = checksum & 0xff

  dat = [mode | (brake >> 8), brake & 0xff, chk1, chk2, idx]
  return [0x315, 0, "".join(map(chr, dat)), 0x14]

def create_acc_dashboard_command(acc_engaged, target_speed_ms, lead_car_in_sight):
  engaged = 0x90 if acc_engaged else 0 # 0x10
  lead_car = 0x10 if lead_car_in_sight else 0
  target_speed = int(target_speed_ms * 208) & 0xfff
  speed_high = target_speed >> 8
  speed_low = target_speed & 0xff
  dat = [0x01, 0x00, engaged | speed_high, speed_low, 0x01, lead_car]
  return [0x370, 0, "".join(map(chr, dat)), 0x10]

def create_adas_time_status(tt, idx):
  dat = [(tt >> 20) & 0xff, (tt >> 12) & 0xff, (tt >> 4) & 0xff,
    ((tt & 0xf) << 4) + (idx << 2)]
  chksum = 0x1000 - dat[0] - dat[1] - dat[2] - dat[3]
  chksum = chksum & 0xfff
  dat += [0x40 + (chksum >> 8), chksum & 0xff, 0x12]
  return [0xa1, 0, "".join(map(chr, dat)), 0]

def create_adas_steering_status(idx):
  dat = [idx << 6, 0xf0, 0x20, 0, 0, 0]
  chksum = 0x60 + sum(dat)
  dat += [chksum >> 8, chksum & 0xff]
  return [0x306, 0, "".join(map(chr, dat)), 0]

def create_adas_accelerometer_speed_status(speed_ms, idx):
  spd = int(speed_ms * 16) & 0xfff
  accel = 0 & 0xfff
  # 0 if in park/neutral, 0x10 if in reverse, 0x08 for D/L
  stick = 0x08
  near_range_cutoff = 0x27
  near_range_mode = 1 if spd <= near_range_cutoff else 0
  far_range_mode = 1 - near_range_mode
  dat = [0x08, spd >> 4, ((spd & 0xf) << 4) | (accel >> 8), accel & 0xff, 0]
  chksum = 0x62 + far_range_mode + (idx << 2) + dat[0] + dat[1] + dat[2] + dat[3] + dat[4]
  dat += [(idx << 5) + (far_range_mode << 4) + (near_range_mode << 3) + (chksum >> 8), chksum & 0xff]
  return [0x308, 0, "".join(map(chr, dat)), 0]

def create_adas_headlights_status():
  return [0x310, 0, "\x42\x04", 0]

def create_chime_command(chime_type, duration, repeat_cnt):
  dat = [chime_type, duration, repeat_cnt, 0xff, 0]
  return [0x10400060, 0, "".join(map(chr, dat)), 1]

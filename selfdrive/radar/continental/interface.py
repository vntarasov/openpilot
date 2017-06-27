#!/usr/bin/env python
import numpy as np
from selfdrive.car.gm.can_parser import CANParser
from selfdrive.boardd.boardd import can_capnp_to_can_list

from cereal import car

import zmq
from selfdrive.services import service_list
import selfdrive.messaging as messaging
import math

NUM_TARGETS_MSG = 1120
SLOT_1_MSG = NUM_TARGETS_MSG + 1
NUM_SLOTS = 20
LAST_RADAR_MSG = 0x47f

def _create_radard_can_parser():
  # C1A-ARS3-A by Continental
  dbc_f = 'gm_global_a_object.dbc'
  radar_targets = range(SLOT_1_MSG, SLOT_1_MSG + NUM_SLOTS)
  signals = zip(['LRRNumObjects'] +
                ['TrkRange'] * NUM_SLOTS + ['TrkRangeRate'] * NUM_SLOTS +
                ['TrkRangeAccel'] * NUM_SLOTS + ['TrkAzimuth'] * NUM_SLOTS +
                ['TrkWidth'] * NUM_SLOTS + ['TrkObjectID'] * NUM_SLOTS,
                [NUM_TARGETS_MSG] + radar_targets * 6,
                [0] + [0.0] * NUM_SLOTS + [0.0] * NUM_SLOTS +
                [0.0] * NUM_SLOTS + [0.0] * NUM_SLOTS +
                [0.0] * NUM_SLOTS + [0] * NUM_SLOTS)

  checks = []

  return CANParser(dbc_f, signals, checks)

class RadarInterface(object):
  def __init__(self):
    # radar
    self.pts = {}
    self.track_id = 0
    self.num_targets = 0

    self.rcp = _create_radard_can_parser()

    context = zmq.Context()
    self.logcan = messaging.sub_sock(context, service_list['can'].port)

  def update(self):
    canMonoTimes = []
    can_pub_radar = []

    while 1:
      for a in messaging.drain_sock(self.logcan, wait_for_one=True):
        canMonoTimes.append(a.logMonoTime)
        can_pub_radar.extend(can_capnp_to_can_list(a.can, [0]))

      # Process all targets after last radar packet
      if any(x[0] == LAST_RADAR_MSG for x in can_pub_radar):
        break

    updated_messages = self.rcp.update_can(can_pub_radar)

    ret = car.RadarState.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("notValid")
    ret.errors = errors
    ret.canMonoTimes = canMonoTimes

    currentTargets = set()
    if self.rcp.vl[NUM_TARGETS_MSG]['LRRNumObjects'] != self.num_targets:
      self.num_targets = self.rcp.vl[NUM_TARGETS_MSG]['LRRNumObjects']

    # Not all radar messages describe targets,
    # no need to monitor all of the sself.rcp.msgs_upd
    for ii in updated_messages:
      if ii == NUM_TARGETS_MSG:
        continue

      if self.num_targets == 0:
        break

      cpt = self.rcp.vl[ii]
      # Zero distance means it's an empty target slot
      if cpt['TrkRange'] > 0.0:
        targetId = cpt['TrkObjectID']
        currentTargets.add(targetId)
        if targetId not in self.pts:
          self.pts[targetId] = car.RadarState.RadarPoint.new_message()
          self.pts[targetId].trackId = targetId
        distance = cpt['TrkRange']
        self.pts[targetId].dRel = distance # from front of car
        # From driver's pov, left is positive
        deg_to_rad = np.pi/180.
        self.pts[targetId].yRel = math.sin(deg_to_rad * cpt['TrkAzimuth']) * distance
        self.pts[targetId].vRel = cpt['TrkRangeRate']
        self.pts[targetId].aRel = float('nan')
        self.pts[targetId].yvRel = float('nan')

    for oldTarget in self.pts.keys():
      if not oldTarget in currentTargets:
        del self.pts[oldTarget]

    ret.points = self.pts.values()
    return ret

if __name__ == "__main__":
  RI = RadarInterface()
  while 1:
    ret = RI.update()
    print(chr(27) + "[2J")
    print ret



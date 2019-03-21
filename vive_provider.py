#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import time
import openvr
import math
import sys
import numpy as np
from utils import *
# translation
# deux vecteurs (1, 0, 0) dans repere terrain, et le vecteur dans repere du vive
# angle entre deux vecteurs normalisés
#   acos(dot(v1, v2))
# axe
# cross


class Calib:
    def __init__(self, positions, halfField):
        self.positions = positions
        self.halfField = halfField

        # deducing last position from first three
        self.positions['3'] = [self.positions[2][0],
                               self.positions[0][1], self.positions[2][2]]
        # self.get_corrected_position()

    def get_corrected_position(self):
        center = [0, 0, 0]

        if self.halfField:
            center[0] = self.positions[2][0]  # center x
        else:
            center[0] = (max(self.positions[1][0], self.positions[2][0]) -
                         max(self.positions[1][0], self.positions[2][0])) / 2,  # center y

        center[1] = (max(self.positions[0][1], self.positions[1][1]) -
                     min(self.positions[0][1], self.positions[1][1]))/2  # center y
        center[2] = self.positions[0][2]

        print(center)

    def get_center(self):
        center = [0, 0, 0]

        if self.halfField:
            center[0] = self.positions[2][0]  # center x
        else:
            center[0] = (max(self.positions[1][0], self.positions[2][0]) -
                         max(self.positions[1][0], self.positions[2][0])) / 2  # center x

        center[1] = (max(self.positions[0][1], self.positions[1][1]) -
                     min(self.positions[0][1], self.positions[1][1]))/2  # center y
        center[2] = self.positions[0][2]

        return center


class Vive_provider:

    def __init__(self, calibFile="calibFile.txt"):
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.trackers = {}
        self.lastInfos = {}

        with open("calibFile.txt", "r") as c:
            cc = eval(c.read())
            self.calib = Calib(cc["positions"], cc["halfField"])

        self.scanTrackers()

    def scanTrackers(self):

        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        ids = {}
        for i in range(1, openvr.k_unMaxTrackedDeviceCount):
            pose = poses[i]
            if not pose.bDeviceIsConnected:
                continue
            if not pose.bPoseIsValid:
                continue
            device_class = openvr.VRSystem().getTrackedDeviceClass(i)
            # if(not device_class == openvr.TrackedDeviceClass_GenericTracker or not device_class == openvr.TrackedDeviceClass_Controller):
            #     continue
            if not device_class == openvr.TrackedDeviceClass_GenericTracker:
                continue
            serial_number = openvr.VRSystem().getStringTrackedDeviceProperty(i,
                                                                             openvr.Prop_SerialNumber_String)
            ids[str(i)] = serial_number

        self.trackers = ids

    # Returns dictionary :
    #
    # {
    #    vive_timestamp   : <>,
    #    time_since_epoch : <>,
    #    tracker_1 {
    #       pose                        : <>,
    #       velocity                    : <>,
    #       angularVelocity             : <>,
    #       vive_timestamp_last_tracked : <>,
    #       time_since_last_tracked     : <>
    #    },
    #    tracker_2 {
    #       pose                        : <>,
    #       velocity                    : <>,
    #       angularVelocity             : <>,
    #       vive_timestamp_last_tracked : <>,
    #       time_since_last_tracked     : <>
    #    },
    # ...
    # }
    def getTrackersInfos(self):

        pose = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        ret = {}

        ret['vive_timestamp'] = int(time.perf_counter()*1000000)
        ret['time_since_epoch'] = int(time.time()*1000000)

        for t in self.trackers:
            id = int(t[0])
            trackerDict = {}
            trackerDict['pose'] = convert_to_quaternion(
                pose[id].mDeviceToAbsoluteTracking)
            trackerDict['velocity'] = [pose[id].vVelocity[0],
                                       pose[id].vVelocity[1], pose[id].vVelocity[2]]
            trackerDict['angularVelocity'] = [pose[id].vAngularVelocity[0],
                                              pose[id].vAngularVelocity[1], pose[id].vAngularVelocity[2]]

            # if first iteration, lastInfos is empty
            if(pose[id].bPoseIsValid or not self.lastInfos):
                trackerDict['vive_timestamp_last_tracked'] = ret['vive_timestamp']
                trackerDict['time_since_last_tracked'] = 0
            else:
                trackerDict['vive_timestamp_last_tracked'] = self.lastInfos["tracker_" +
                                                                            str(id)]['vive_timestamp_last_tracked']
                trackerDict['time_since_last_tracked'] = ret['vive_timestamp'] - \
                    self.lastInfos["tracker_" +
                                   str(id)]['vive_timestamp_last_tracked']

            # trackerDict['serialNumber'] = t[1]
            trackerDict['serial_number'] = "TODO"

            ret["tracker_"+str(id)] = trackerDict

        self.lastInfos = ret.copy()

        return ret

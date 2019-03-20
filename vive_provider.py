import sys
import time
import openvr
import math
import sys
import numpy as np
from utils import *

class Vive_provider:
    
    def __init__(self):
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.trackers = {}
        self.lastInfos = {}
        self.scanTrackers()
        
    def scanTrackers(self):
        
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        
        ids = {}
        for i in range(1, openvr.k_unMaxTrackedDeviceCount):
            pose = poses[i]
            if not pose.bDeviceIsConnected:
                continue
            if not pose.bPoseIsValid:
                continue
            device_class = openvr.VRSystem().getTrackedDeviceClass(i)
            if not device_class == openvr.TrackedDeviceClass_GenericTracker:
                continue            
            serial_number = openvr.VRSystem().getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
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
        
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        ret = {}
        
        ret['vive_timestamp'] = int(time.perf_counter()*1000000)
        ret['time_since_epoch'] = int(time.time()*1000000)
        
        for t in self.trackers:
            id = int(t[0])
            trackerDict = {}
            trackerDict['pose'] = convert_to_quaternion(pose[id].mDeviceToAbsoluteTracking)
            trackerDict['velocity'] = [pose[id].vVelocity[0], pose[id].vVelocity[1], pose[id].vVelocity[2]]
            trackerDict['angularVelocity'] = [pose[id].vAngularVelocity[0], pose[id].vAngularVelocity[1], pose[id].vAngularVelocity[2]]
            
            if(pose[id].bPoseIsValid or not self.lastInfos): # if first iteration, lastInfos is empty
                trackerDict['vive_timestamp_last_tracked'] = ret['vive_timestamp']
                trackerDict['time_since_last_tracked'] = 0
            else:
                trackerDict['vive_timestamp_last_tracked'] = self.lastInfos["tracker_"+str(id)]['vive_timestamp_last_tracked']
                trackerDict['time_since_last_tracked'] = ret['vive_timestamp'] - self.lastInfos["tracker_"+str(id)]['vive_timestamp_last_tracked']

            trackerDict['serialNumber'] = t[1]
                
            ret["tracker_"+str(id)] = trackerDict

        self.lastInfos = ret.copy()
            
        return ret





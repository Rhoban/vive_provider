import sys
import time
import openvr
import math
import sys
import numpy as np
from utils import *

class Vive_provider:
    
    def __init__(self, nbTrackers):
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.nbTrackers = nbTrackers
        self.trackers = [1] # TODO initialize with scanForTrackers()

    # TODO retrieve serial numbers of trackers, and associate with index
    # Not very clear how it works at the moment
    def scanForTrackers(self):
        ids = [1]

        # nothing is working in here yet
        # aze = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        
        # for i in range(1, self.nbTrackers+10):
        #     if(aze[i].bPoseIsValid and aze[i].bDeviceIsConnected and ):
        #         print("coucou")
            # print(aze[i].mDeviceToAbsoluteTracking)
            # aaa = openvr.ETrackedDeviceProperty()
            # aze = self.vr.getStringTrackedDeviceProperty(i, aaa)
            

            # vr::VRSystem()->GetStringTrackedDeviceProperty(deviceID, vr::Prop_SerialNumber_String, serialNumber, sizeof(serialNumber));

        return ids

    
    # Returns dictionary :
    #   pose (quaternion [x, y, z, r_w, r_x, r_y, r_z])
    #   velocity (3d vector)
    #   angularVelocity (3d vector)
    #   vive_timestamp
    #   time_since_epoch
    # Returns none if tracker is not visible or if trackerId is not in the list of ids
    def getTrackerInfo(self, trackerId):
        
        if (trackerId not in self.trackers):
            print("Error : Invalid trackerId") # TODO maybe raise an exception here ?
            return None
        
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        
        if(not pose[trackerId].bPoseIsValid):
            return None

        ret = {}

        ret['pose'] = convert_to_quaternion(pose[trackerId].mDeviceToAbsoluteTracking)
        ret['velocity'] = [pose[trackerId].vVelocity[0], pose[trackerId].vVelocity[1], pose[trackerId].vVelocity[2]]
        ret['angularVelocity'] = [pose[trackerId].vAngularVelocity[0], pose[trackerId].vAngularVelocity[1], pose[trackerId].vAngularVelocity[2]]
        ret['vive_timestamp'] = time.perf_counter()
        ret['time_since_epoch'] = int(time.time()*1000000)
        return ret




#!/usr/bin/python3
import sys
import time
import openvr
import math
import sys
import numpy as np
from utils import *
import  numpy.linalg as linalg
import os

class Calib:
    def __init__(self, positions, halfField):
        self.positions = positions
        self.halfField = halfField

        self.t = np.mat([])
        self.R = np.mat([])
        self.computeTranslationAndRotation()

    def computeTranslationAndRotation(self):
        field_positions = []
        if(self.halfField):
            field_positions = [[-4.5, 3, 0],
                               [-4.5, -3, 0],
                               [0, 3, 0]]
        else:
            field_positions = [[-4.5, 3, 0],
                               [-4.5, -3, 0],
                               [4.5, -3, 0]]
        
        positions = [self.positions[0],
                     self.positions[1],
                     self.positions[2]]
        
        R, t = rigid_transform_3D(np.mat(positions), np.mat(field_positions))

        self.t = t
        self.R = R

    def get_transformation_matrix(self):
        m = self.R.copy()
        m = np.hstack((m, self.t))
        m = np.vstack((m, [0, 0, 0, 1]))
        
        return m

class Tracker:

    def __init__(self, serial_number, device_type, openvr_id):
        self.serial_number = serial_number
        self.device_type = device_type
        self.openvr_id = openvr_id
        

    
class Vive_provider:
    
    def __init__(self, calibFilePath="calibFile.txt"):
        
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.trackers = {}
        self.lastInfos = {}

        
        if os.path.exists(calibFilePath):
            with open(calibFilePath, "r") as calibFile:
                c = eval(calibFile.read())
                self.calib = Calib(c["positions"], c["halfField"])
        else:
            print("Calibration file not found")
            print("Exiting ...")
            sys.exit()
            
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

            if(device_class == openvr.TrackedDeviceClass_GenericTracker or device_class == openvr.TrackedDeviceClass_Controller):
                serial_number = openvr.VRSystem().getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
                if device_class == openvr.TrackedDeviceClass_Controller:
                    device_type = "controller"
                else:
                    device_type = "tracker"
                    
                t = Tracker(serial_number, device_type, i)
                
                self.trackers[str(serial_number)] = t
                
    def getTrackersInfos(self, raw=False):
        
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        ret = {}
        
        ret['vive_timestamp'] = int(time.perf_counter()*1000000)
        ret['time_since_epoch'] = int(time.time()*1000000)

        trackersDict = {}
        
        for t in self.trackers.values():
            currentTrackerDict = {}
            
            currentTracker = pose[t.openvr_id]
            
            p = currentTracker.mDeviceToAbsoluteTracking
            m = np.matrix([list(p[0]), list(p[1]), list(p[2])])
            m = np.vstack((m, [0, 0, 0, 1]))
            corrected = m

            if not raw:
                Rz = np.matrix([
                    [math.cos(math.pi/2), -math.sin(math.pi/2), 0, 0],
                    [math.sin(math.pi/2), math.cos(math.pi/2), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
                m = m*Rz
                corrected = self.calib.get_transformation_matrix()*m

            currentTrackerDict['openvr_id'] = t.openvr_id
            currentTrackerDict['serial_number'] = t.serial_number
            currentTrackerDict['device_type'] = t.device_type
            currentTrackerDict['pose'] = convert_to_quaternion(np.array(corrected[:3, :4]))
            currentTrackerDict['pose_matrix'] = corrected
            currentTrackerDict['velocity'] = [currentTracker.vVelocity[0], currentTracker.vVelocity[1], currentTracker.vVelocity[2]]
            currentTrackerDict['angular_velocity'] = [currentTracker.vAngularVelocity[0], currentTracker.vAngularVelocity[1], currentTracker.vAngularVelocity[2]]
            
            if(currentTracker.bPoseIsValid or not self.lastInfos): # if first iteration, lastInfos is empty
                currentTrackerDict['vive_timestamp_last_tracked'] = ret['vive_timestamp']
                currentTrackerDict['time_since_last_tracked'] = 0
            else:
                currentTrackerDict['vive_timestamp_last_tracked'] = self.lastInfos["trackers"][str(t.serial_number)]['vive_timestamp_last_tracked']
                currentTrackerDict['time_since_last_tracked'] = ret['vive_timestamp'] - self.lastInfos[str(t.serial_number)]['vive_timestamp_last_tracked']
            
            trackersDict[str(t.serial_number)] = currentTrackerDict
                
        ret["trackers"] = trackersDict             

        self.lastInfos = ret.copy()
            
        return ret

    
    def getControllersInfos(self, raw=False):
        controllers = []
        trackers = self.getTrackersInfos(raw)

        for t in self.trackers.values():
            if(str(t.device_type) == "controller"):
                controllers.append(self.trackers[t.serial_number])

        return controllers
                

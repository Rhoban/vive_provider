#!/usr/bin/python3
import sys
import time
import openvr
import math
import sys
import numpy as np
from utils import *
import  numpy.linalg as linalg

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

                ids[str(i)] = {}
                ids[str(i)]["serial_number"] = serial_number
                if device_class == openvr.TrackedDeviceClass_Controller: 
                    ids[str(i)]["device_type"] = "controller"        
                else:
                    ids[str(i)]["device_type"] = "tracker"
                
        print(ids)
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
    
    def getControllersInfos(self, raw=False):
        controllers = []
        trackers = self.getTrackersInfos(raw)

        for id in self.trackers:
            name = 'tracker_'+str(id)
            if trackers[name]['device_type'] == 'controller':
                controllers.append(trackers[name])

        return controllers

    def getTrackersInfos(self, raw=False):
        
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        ret = {}
        
        ret['vive_timestamp'] = int(time.perf_counter()*1000000)
        ret['time_since_epoch'] = int(time.time()*1000000)

        for t in self.trackers:
            id = int(t[0])
            trackerDict = {}

            ppose = convert_to_quaternion(pose[id].mDeviceToAbsoluteTracking)

            p = pose[id].mDeviceToAbsoluteTracking
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
            
            trackerDict['pose'] = convert_to_quaternion(np.array(corrected[:3, :4]))
            trackerDict['pose_matrix'] = corrected
            trackerDict['velocity'] = [pose[id].vVelocity[0], pose[id].vVelocity[1], pose[id].vVelocity[2]]
            trackerDict['angularVelocity'] = [pose[id].vAngularVelocity[0], pose[id].vAngularVelocity[1], pose[id].vAngularVelocity[2]]
            
            if(pose[id].bPoseIsValid or not self.lastInfos): # if first iteration, lastInfos is empty
                trackerDict['vive_timestamp_last_tracked'] = ret['vive_timestamp']
                trackerDict['time_since_last_tracked'] = 0
            else:
                trackerDict['vive_timestamp_last_tracked'] = self.lastInfos["tracker_"+str(id)]['vive_timestamp_last_tracked']
                trackerDict['time_since_last_tracked'] = ret['vive_timestamp'] - self.lastInfos["tracker_"+str(id)]['vive_timestamp_last_tracked']

            # trackerDict['serialNumber'] = t[1]
            trackerDict['serial_number'] = "TODO"
            trackerDict['device_type'] = self.trackers[str(t)]['device_type']
            # trackerDict['button_state'] = self.trackers[str(t)]['device_type']

            # if trackerDict['device_type'] == 'controller':
            #     _, state = self.vr.getControllerState(id)
            #     trackerDict['buttonPressed'] = (state.ulButtonPressed != 0)
            # print(self.vr.getControllerState(t))
# self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)z
            
                
            ret["tracker_"+str(id)] = trackerDict

        self.lastInfos = ret.copy()
            
        return ret





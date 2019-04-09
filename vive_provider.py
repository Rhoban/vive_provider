#!/usr/bin/python3
import sys
import time
import openvr
import math
import sys
import json
import numpy as np
from utils import *
import  numpy.linalg as linalg
from functools import reduce
import os
from vive_pb2 import *
import socket

class Calib:
    def __init__(self, calibFilePath):
        self.calibration = None
        loaded = False
        if os.path.exists(calibFilePath):
            with open(calibFilePath, "r") as calibFile:
                self.calibration = json.loads(calibFile.read())
                loaded = True
            for id in self.calibration:
                self.calibration[id] = np.mat(self.calibration[id])
        else:
            print("! WARNING: Calibration file not found")


    def transform_frame(self, references, trackerToWorld):
        if self.calibration is None:
            return trackerToWorld

        frames = []
        # return self.calibration['worldToField']*trackerToWorld

        for id in references:
            if id in self.calibration:
                referenceToWorld = references[id]
                fieldToReference = self.calibration[id]

                trackerToReference = np.linalg.inv(referenceToWorld)*trackerToWorld
                trackerToField = np.linalg.inv(fieldToReference)*trackerToReference
                # print(str(id)+' '+str(trackerToReference))
                frames.append(trackerToField)
        
        if len(frames) > 1:
            return reduce(lambda a, b: average_transforms(a, b), frames)
        elif len(frames) == 1:
            return frames[0]
        else:
            print('! ERROR: Cant find a suitable reference!')
            exit()

    def check_consistency(self, references):

        if self.calibration is None:
            return
        keys = list(references.keys())
        for i in range(len(keys)):
            for j in range(i+1, len(keys)):
                keyA = keys[i]
                keyB = keys[j]
                if keyA in self.calibration and keyB in self.calibration:
                    expectedTransformation = self.calibration[keyB] * np.linalg.inv(self.calibration[keyA])
                    transformation = np.linalg.inv(references[keyB]) * references[keyA]

                    if not np.allclose(expectedTransformation.T[3,:3], transformation.T[3,:3], atol=0.15):
                        print('! ERROR: Translation from %s to %s is not consistent with calibration' % (keyA, keyB))
                        # print('Expected: %g, %g, %g' % tuple(np.array(expectedTransformation.T[3,:3])[0]))
                        # print('Got: %g, %g, %g' % tuple(np.array(transformation.T[3,:3])[0]))
                    elif not np.allclose(expectedTransformation[:3,:3], transformation[:3,:3], atol=0.05):   
                        print('! ERROR: Rotation from %s to %s is not consistent with calibration' % (keyA, keyB))

class Tracker:

    def __init__(self, serial_number, device_type, openvr_id):
        self.serial_number = serial_number
        self.device_type = device_type
        self.openvr_id = openvr_id
        

    
class Vive_provider:
    
    def __init__(self, enableButtons=False, calibFilePath="calibFile.json", clientMode=False):
        self.clientMode = clientMode
        
        self.trackers = {}
        self.references = {}
        self.lastInfos = {}
        self.enableButtons = enableButtons
        self.taggedPositions = []
        
        if(not self.clientMode):
            self.vr = openvr.init(openvr.VRApplication_Other)
            self.calib = Calib(calibFilePath)
            if os.path.exists('taggedPositions.json'):
                f = open('taggedPositions.json', 'r')
                self.taggedPositions = json.load(f)
                f.close()
        else:
            self.vr = None
            self.calib = None
            
            self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.client.bind(("", 37020))

        self.scanTrackers()            
            

    def scanTrackers(self):

        if self.clientMode :
            pb_msg = GlobalMsg()
            data, addr = self.client.recvfrom(1024)
            pb_msg.ParseFromString(data)
            infos= GlobalMsg_to_trackersInfos(data)

            for t in infos['trackers'].values():
                self.trackers[t['serial_number']] = t['openvr_id']

            return
            
        
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        ids = {}
        
        for i in range(1, openvr.k_unMaxTrackedDeviceCount):
            pose = poses[i]
            if not pose.bDeviceIsConnected:
                continue
            if not pose.bPoseIsValid:
                continue
            
            device_class = openvr.VRSystem().getTrackedDeviceClass(i)
            serial_number = openvr.VRSystem().getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String).decode('UTF-8')

            if(device_class == openvr.TrackedDeviceClass_GenericTracker or device_class == openvr.TrackedDeviceClass_Controller):
                if device_class == openvr.TrackedDeviceClass_Controller:
                    device_type = "controller"
                else:
                    device_type = "tracker"
                    
                t = Tracker(serial_number, device_type, i)
                
                self.trackers[serial_number] = t
            elif device_class == openvr.TrackedDeviceClass_TrackingReference:
                self.references[serial_number] = i
                
    def getTrackersInfos(self, raw=False):
        
        if self.clientMode:
            pb_msg = GlobalMsg()
            data, addr = self.client.recvfrom(1024)
            pb_msg.ParseFromString(data)
            return GlobalMsg_to_trackersInfos(data)
            

        ret['vive_timestamp'] = int(time.perf_counter()*1000000)
        ret['time_since_epoch'] = int(time.time()*1000000)
        
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        ret = {}

        references = {}
        for r in self.references:
            openvr_id = self.references[r]
            currentTracker = pose[openvr_id]
    
            p = currentTracker.mDeviceToAbsoluteTracking
            m = np.matrix([list(p[0]), list(p[1]), list(p[2])])
            m = np.vstack((m, [0, 0, 0, 1]))
            corrected = m

            # if not raw:
            #     corrected = self.calib.get_transformation_matrix()*m

            references[r] = corrected
        ret["references"] = references

        trackersDict = {}
        
        for t in self.trackers.values():
            currentTrackerDict = {}
            
            currentTracker = pose[t.openvr_id]
    
            p = currentTracker.mDeviceToAbsoluteTracking
            m = np.matrix([list(p[0]), list(p[1]), list(p[2])])
            m = np.vstack((m, [0, 0, 0, 1]))
            trackerToWorld = m

            if t.device_type == 'controller':
                trackerToWorld = trackerToWorld*translation_transformation([0, -0.025, -0.025])

            if t.device_type == 'tracker':
                # These transformations are here to switch from the official frame to
                # our frame where the tracker LED is along the X axis
                trackerToWorld = trackerToWorld*rotation_transformation(math.pi, 'y')
                trackerToWorld = trackerToWorld*rotation_transformation(math.pi/2, 'z')
                trackerToWorld = trackerToWorld*translation_transformation([0, 0, -0.01])
                
            if not raw:
                trackerToWorld = self.calib.transform_frame(references, trackerToWorld)

            currentTrackerDict['openvr_id'] = t.openvr_id
            currentTrackerDict['serial_number'] = t.serial_number
            currentTrackerDict['device_type'] = t.device_type
            currentTrackerDict['pose'] = convert_to_quaternion(np.array(trackerToWorld[:3, :4]))
            currentTrackerDict['pose_matrix'] = trackerToWorld
            currentTrackerDict['velocity'] = [currentTracker.vVelocity[0], currentTracker.vVelocity[1], currentTracker.vVelocity[2]]
            currentTrackerDict['angular_velocity'] = [currentTracker.vAngularVelocity[0], currentTracker.vAngularVelocity[1], currentTracker.vAngularVelocity[2]]
            
            if(currentTracker.bPoseIsValid or not self.lastInfos): # if first iteration, lastInfos is empty
                currentTrackerDict['vive_timestamp_last_tracked'] = ret['vive_timestamp']
                currentTrackerDict['time_since_last_tracked'] = 0
            else:
                currentTrackerDict['vive_timestamp_last_tracked'] = self.lastInfos["trackers"][t.serial_number]['vive_timestamp_last_tracked']
                currentTrackerDict['time_since_last_tracked'] = ret['vive_timestamp'] - self.lastInfos['trackers'][t.serial_number]['vive_timestamp_last_tracked']

            if self.enableButtons:
                if currentTrackerDict['device_type'] == 'controller':
                    _, state = self.vr.getControllerState(t.openvr_id)
                    currentTrackerDict['buttonPressed'] = (state.ulButtonPressed != 0)

            
            trackersDict[t.serial_number] = currentTrackerDict
                     
        ret["trackers"] = trackersDict
        ret["tagged_positions"] = self.taggedPositions

        references_corrected = {}
        for id in references:
            references_corrected[id] = self.calib.transform_frame(references, references[id])

        ret["references_corrected"] = references_corrected

        if not raw:
            self.calib.check_consistency(references)
        self.lastInfos = ret.copy()
            
        return ret

    def vibrate(self, serial, duration=250):
        openvr_id = self.trackers[serial].openvr_id
        for x in range(duration): 
            self.vr.triggerHapticPulse(openvr_id, 0, 5000)
            time.sleep(0.001) 
    
    def getControllersInfos(self, raw=False):
        controllers = []
        trackers = self.getTrackersInfos(raw)['trackers']

        for t in self.trackers.values():
            if(str(t.device_type) == "controller"):
                controllers.append(trackers[t.serial_number])

        return controllers
                

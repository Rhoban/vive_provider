import math
from vive_pb2 import *
import time
import numpy as np

import  numpy.linalg as linalg

#Convert the standard 3x4 position/rotation matrix to a x,y,z location and the appropriate Euler angles (in degrees)
def convert_to_euler(pose_mat):
    yaw = 180 / math.pi * math.atan(pose_mat[1][0] /pose_mat[0][0])
    pitch = 180 / math.pi * math.atan(-1 * pose_mat[2][0] / math.sqrt(pow(pose_mat[2][1], 2) + math.pow(pose_mat[2][2], 2)))
    roll = 180 / math.pi * math.atan(pose_mat[2][1] /pose_mat[2][2])
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x, y, z, yaw, pitch, roll]

#Convert the standard 3x4 position/rotation matrix to a x,y,z location and the appropriate Quaternion
def convert_to_quaternion(pose_mat):
    # Per issue #2, adding a abs() so that sqrt only results in real numbers
    r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
    r_x = (pose_mat[2][1]-pose_mat[1][2])/(4*r_w+0.0000001)
    r_y = (pose_mat[0][2]-pose_mat[2][0])/(4*r_w+0.0000001)
    r_z = (pose_mat[1][0]-pose_mat[0][1])/(4*r_w+0.0000001)

    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x,y,z,r_w,r_x,r_y,r_z]

def trackersInfos_to_GlobalMsg(trackersInfos):
    
    pb_msg = GlobalMsg()

    pb_msg.vive_timestamp = trackersInfos['vive_timestamp']
    pb_msg.time_since_epoch = trackersInfos['time_since_epoch']

    nbTrackers = len(trackersInfos) - 2
    for i in range(0, nbTrackers):
        tracker = trackersInfos["tracker_"+str(i+1)]
        pb_msg.trackers.add()
        pb_msg.trackers[i].tracker_idx = i+1
        pb_msg.trackers[i].time_since_last_tracked = tracker['time_since_last_tracked']
        pb_msg.trackers[i].pos.x = tracker['pose'][0]
        pb_msg.trackers[i].pos.y = tracker['pose'][1]
        pb_msg.trackers[i].pos.z = tracker['pose'][2]
        pb_msg.trackers[i].orientation.qw = tracker['pose'][3]
        pb_msg.trackers[i].orientation.qx = tracker['pose'][4]
        pb_msg.trackers[i].orientation.qy = tracker['pose'][5]
        pb_msg.trackers[i].orientation.qz = tracker['pose'][6]
        pb_msg.trackers[i].cartesian_velocity.x = tracker['velocity'][0]
        pb_msg.trackers[i].cartesian_velocity.y = tracker['velocity'][1]
        pb_msg.trackers[i].cartesian_velocity.z = tracker['velocity'][2]
        pb_msg.trackers[i].serial_number = tracker['serial_number']

    return pb_msg

def GlobalMsg_to_trackersInfos(data):
    
    pb_msg = GlobalMsg()
    pb_msg.ParseFromString(data)

    nbTrackers = len(pb_msg.trackers)

    ret = {}
    ret['vive_timestamp'] = pb_msg.vive_timestamp
    ret['time_since_epoch'] = pb_msg.time_since_epoch
    
    for i in range(0, nbTrackers):
        trackerDict = {}
        trackerDict['pose'] = [pb_msg.trackers[i].pos.x, pb_msg.trackers[i].pos.y, pb_msg.trackers[i].pos.z, pb_msg.trackers[i].orientation.qw, pb_msg.trackers[i].orientation.qx, pb_msg.trackers[i].orientation.qy, pb_msg.trackers[i].orientation.qz]
        trackerDict['velocity'] = [pb_msg.trackers[i].cartesian_velocity.x, pb_msg.trackers[i].cartesian_velocity.y, pb_msg.trackers[i].cartesian_velocity.z]
        trackerDict['angularVelocity'] = [0, 0, 0]
        trackerDict['vive_timestamp_last_tracked'] = 0
        trackerDict['time_since_last_tracked'] = pb_msg.trackers[i].time_since_last_tracked
            
        trackerDict['serial_number'] = "TODO"
                
        ret["tracker_"+str(i)] = trackerDict

    return ret



    
def get_dummy_trackerInfos():
    dummy_trackerInfos = {}

    dummy_trackerInfos['vive_timestamp'] = int(time.perf_counter()*1000000)
    dummy_trackerInfos['time_since_epoch'] = int(time.time()*1000000)

    for i in range(1, 3):
        trackerDict = {}
        trackerDict['pose'] = [1, 2, 3, 0, 0, 0, 0]
        trackerDict['velocity'] = [1, 2, 3]
        trackerDict['angularVelocity'] = [1, 2, 3]
        trackerDict['vive_timestamp_last_tracked'] = time.perf_counter()
        trackerDict['time_since_last_tracked'] = 0
        trackerDict['serial_number'] = "SERIALNUMBER123"
        dummy_trackerInfos["tracker_"+str(i)] = trackerDict

    return dummy_trackerInfos

# Taken from https://nghiaho.com/?page_id=671
# R : rotation matrix
# t : translation vector
def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.transpose(AA) * BB

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       # print("Reflection detected")
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    # print(t)

    return R, t

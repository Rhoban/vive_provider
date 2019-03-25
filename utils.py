import math
from vive_pb2 import *
import time
import numpy as np

import  numpy.linalg as linalg

# Converts a numpy matrix to euler angles (rpy)
def convert_to_euler(R) :    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy) 
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy) 
        z = 0 
 
    return np.array([x, y, z])


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

def trackersInfos_to_GlobalMsg(trackersInfos, seq):
    pb_msg = GlobalMsg()

    
    pb_msg.vive_timestamp = trackersInfos['vive_timestamp']
    pb_msg.time_since_epoch = trackersInfos['time_since_epoch']
    pb_msg.seq = seq

    trackers = trackersInfos["trackers"]
    # print(trackers)
    i = 0
    for t in trackers.values():
        pb_msg.trackers.add()
        
        pb_msg.trackers[i].tracker_idx = t['openvr_id']
        pb_msg.trackers[i].time_since_last_tracked = t['time_since_last_tracked']
        pb_msg.trackers[i].pos.x = t['pose'][0]
        pb_msg.trackers[i].pos.y = t['pose'][1]
        pb_msg.trackers[i].pos.z = t['pose'][2]
        pb_msg.trackers[i].orientation.qw = t['pose'][3]
        pb_msg.trackers[i].orientation.qx = t['pose'][4]
        pb_msg.trackers[i].orientation.qy = t['pose'][5]
        pb_msg.trackers[i].orientation.qz = t['pose'][6]
        pb_msg.trackers[i].cartesian_velocity.x = t['velocity'][0]
        pb_msg.trackers[i].cartesian_velocity.y = t['velocity'][1]
        pb_msg.trackers[i].cartesian_velocity.z = t['velocity'][2]
        pb_msg.trackers[i].serial_number = t['serial_number']
        pb_msg.trackers[i].device_type = t['device_type']

        pose_matrix = np.array(t['pose_matrix'])
        pb_msg.trackers[i].pose_matrix.i0j0 = pose_matrix[0][0]
        pb_msg.trackers[i].pose_matrix.i0j1 = pose_matrix[0][1]
        pb_msg.trackers[i].pose_matrix.i0j2 = pose_matrix[0][2]
        pb_msg.trackers[i].pose_matrix.i0j3 = pose_matrix[0][3]
        pb_msg.trackers[i].pose_matrix.i1j0 = pose_matrix[1][0]
        pb_msg.trackers[i].pose_matrix.i1j1 = pose_matrix[1][1]
        pb_msg.trackers[i].pose_matrix.i1j2 = pose_matrix[1][2]
        pb_msg.trackers[i].pose_matrix.i1j3 = pose_matrix[1][3]
        pb_msg.trackers[i].pose_matrix.i2j0 = pose_matrix[2][0]
        pb_msg.trackers[i].pose_matrix.i2j1 = pose_matrix[2][1]
        pb_msg.trackers[i].pose_matrix.i2j2 = pose_matrix[2][2]
        pb_msg.trackers[i].pose_matrix.i2j3 = pose_matrix[2][3]

        i+=1
        
    return pb_msg

def GlobalMsg_to_trackersInfos(data):
    
    pb_msg = GlobalMsg()
    pb_msg.ParseFromString(data)

    nbTrackers = len(pb_msg.trackers)

    ret = {}
    ret['vive_timestamp'] = pb_msg.vive_timestamp
    ret['time_since_epoch'] = pb_msg.time_since_epoch
    ret['seq'] = pb_msg.seq

    trackersDict = {}
    for i in range(0, nbTrackers):
        currentTrackerDict = {}
        currentTrackerDict['openvr_id'] = pb_msg.trackers[i].tracker_idx
        currentTrackerDict['time_since_last_tracked'] = pb_msg.trackers[i].time_since_last_tracked
        currentTrackerDict['pose'] = [0, 0, 0, 0, 0, 0, 0]
        currentTrackerDict['pose'][0] = pb_msg.trackers[i].pos.x
        currentTrackerDict['pose'][1] = pb_msg.trackers[i].pos.y
        currentTrackerDict['pose'][2] = pb_msg.trackers[i].pos.z
        currentTrackerDict['pose'][3] = pb_msg.trackers[i].orientation.qw
        currentTrackerDict['pose'][4] = pb_msg.trackers[i].orientation.qx
        currentTrackerDict['pose'][5] = pb_msg.trackers[i].orientation.qy
        currentTrackerDict['pose'][6] = pb_msg.trackers[i].orientation.qz
        currentTrackerDict['velocity'] = [0, 0, 0]
        currentTrackerDict['velocity'][0] = pb_msg.trackers[i].cartesian_velocity.x
        currentTrackerDict['velocity'][1] = pb_msg.trackers[i].cartesian_velocity.y
        currentTrackerDict['velocity'][2] = pb_msg.trackers[i].cartesian_velocity.z
        currentTrackerDict['serial_number'] = pb_msg.trackers[i].serial_number
        currentTrackerDict['device_type'] = pb_msg.trackers[i].device_type

        pose_matrix = [[0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [0, 0, 0, 0]]

        pose_matrix[0][0] = pb_msg.trackers[i].pose_matrix.i0j0
        pose_matrix[0][1] = pb_msg.trackers[i].pose_matrix.i0j1
        pose_matrix[0][2] = pb_msg.trackers[i].pose_matrix.i0j2
        pose_matrix[0][3] = pb_msg.trackers[i].pose_matrix.i0j3
        pose_matrix[1][0] = pb_msg.trackers[i].pose_matrix.i1j0
        pose_matrix[1][1] = pb_msg.trackers[i].pose_matrix.i1j1
        pose_matrix[1][2] = pb_msg.trackers[i].pose_matrix.i1j2
        pose_matrix[1][3] = pb_msg.trackers[i].pose_matrix.i1j3
        pose_matrix[2][0] = pb_msg.trackers[i].pose_matrix.i2j0
        pose_matrix[2][1] = pb_msg.trackers[i].pose_matrix.i2j1
        pose_matrix[2][2] = pb_msg.trackers[i].pose_matrix.i2j2
        pose_matrix[2][3] = pb_msg.trackers[i].pose_matrix.i2j3
        pose_matrix[3][0] = 0
        pose_matrix[3][1] = 0
        pose_matrix[3][2] = 0
        pose_matrix[3][3] = 1
        currentTrackerDict['pose_matrix'] = np.matrix(pose_matrix)

        trackersDict[currentTrackerDict['serial_number']] = currentTrackerDict
        

    ret["trackers"] = trackersDict
    
    return ret

# TODO    
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
        pose_matrix = [[0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [0, 0, 0, 0]]
        trackerDict['pose_matrix'] = pose_matrix
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

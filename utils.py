import math
from vive_pb2 import *
import time
import numpy as np
from transforms3d import axangles

import numpy.linalg as linalg

# UDP server PORT
VIVE_SERVER_PORT: int = 44444

# File name for points used for calibration
FIELD_POINTS_FILENAME: str = "field_points.json"

# File name to output calibration
CALIBRATION_FILENAME: str = "calibration.json"

# File name for tagged positions
TAGGED_POSITIONS_FILENAME: str = "tagged_positions.json"


def tracker_infos_to_GlobalMsg(tracker_infos: dict) -> GlobalMsg:
    """
    Converts tracker infos to a protobuf message

    :param dict tracker_infos: tracker info
    :param int seq: sequence number
    :return GlobalMsg: global message
    """
    pb_msg = GlobalMsg()

    pb_msg.vive_timestamp = tracker_infos["vive_timestamp"]
    pb_msg.time_since_epoch = tracker_infos["time_since_epoch"]

    trackers = tracker_infos["trackers"]

    for t in trackers.values():
        pb_msg.trackers.add()

        pb_msg.trackers[-1].tracker_idx = t["openvr_index"]
        pb_msg.trackers[-1].time_since_last_tracked = t["time_since_last_tracked"]
        pb_msg.trackers[-1].pos.x = t["position"][0]
        pb_msg.trackers[-1].pos.y = t["position"][1]
        pb_msg.trackers[-1].pos.z = t["position"][2]
        pb_msg.trackers[-1].orientation.qw = t["orientation"][0]
        pb_msg.trackers[-1].orientation.qx = t["orientation"][1]
        pb_msg.trackers[-1].orientation.qy = t["orientation"][2]
        pb_msg.trackers[-1].orientation.qz = t["orientation"][3]
        pb_msg.trackers[-1].cartesian_velocity.x = t["velocity"][0]
        pb_msg.trackers[-1].cartesian_velocity.y = t["velocity"][1]
        pb_msg.trackers[-1].cartesian_velocity.z = t["velocity"][2]
        pb_msg.trackers[-1].serial_number = t["serial_number"]
        pb_msg.trackers[-1].device_type = t["device_type"]

    for p in tracker_infos["tagged_positions"]:
        pb_msg.tagged_positions.add()
        pb_msg.tagged_positions[-1].x = p[0]
        pb_msg.tagged_positions[-1].y = p[1]
        pb_msg.tagged_positions[-1].z = p[2]

    return pb_msg


def GlobalMsg_to_tracker_infos(data: str) -> dict:
    """
    Parses a (string) data and convert it to trackers infos

    :param str data: string (serialized) representation of GlobalMsg
    :return dict: tracker infos
    """
    pb_msg = GlobalMsg()
    pb_msg.ParseFromString(data)

    infos = {}
    infos["vive_timestamp"] = pb_msg.vive_timestamp
    infos["time_since_epoch"] = pb_msg.time_since_epoch
    infos["seq"] = pb_msg.seq

    infos["trackers"] = {}
    for tracker in pb_msg.trackers:
        entry = {}
        entry["openvr_index"] = tracker.tracker_idx
        entry["time_since_last_tracked"] = tracker.time_since_last_tracked
        entry["position"] = [0, 0, 0]
        entry["position"][0] = tracker.pos.x
        entry["position"][1] = tracker.pos.y
        entry["position"][2] = tracker.pos.z
        entry["orientation"] = [0, 0, 0, 0]
        entry["orientation"][0] = tracker.orientation.qw
        entry["orientation"][1] = tracker.orientation.qx
        entry["orientation"][2] = tracker.orientation.qy
        entry["orientation"][3] = tracker.orientation.qz
        entry["velocity"] = [0, 0, 0]
        entry["velocity"][0] = tracker.cartesian_velocity.x
        entry["velocity"][1] = tracker.cartesian_velocity.y
        entry["velocity"][2] = tracker.cartesian_velocity.z
        entry["serial_number"] = tracker.serial_number
        entry["device_type"] = tracker.device_type

        infos["trackers"][entry["serial_number"]] = entry

    infos["tagged_positions"] = {}
    for i in range(0, len(pb_msg.tagged_positions)):
        infos["tagged_positions"].append(
            [pb_msg.tagged_positions[i].x, pb_msg.tagged_positions[i].y, pb_msg.tagged_positions[i].z]
        )

    return infos


def tracker_to_matrix(openvr_tracker) -> np.array:
    """
    Converts an OpenVR tracker object to a 4x4 numpy matrix

    :param openvr_tracker: OpenVR tracker
    :return np.array: matrix
    """
    frame = np.eye(4)
    frame[:3] = openvr_tracker.mDeviceToAbsoluteTracking[:3]

    return frame


def rigid_transform_3D(A: np.array, B: np.array) -> np.array:
    """
    Finds best transformation to match point-to-point
    https://nghiaho.com/?page_id=671

    :param list A: point cloud A
    :param list B: point cloud B
    :return np.array: A 4x4 transformation matrix, transforming points from A to B
    """
    assert len(A) == len(B)

    # Computing centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Centre the points
    AA = A - centroid_A
    BB = B - centroid_B

    # Computing covariance
    H = np.transpose(AA) @ BB

    U, S, Vt = linalg.svd(H)

    # Building output frame
    T = np.eye(4)
    T[:3, :3] = Vt.T @ U.T

    # Special reflection case
    if linalg.det(T[:3, :3]) < 0:
        Vt[2, :] *= -1
        T[:3, :3] = Vt.T @ U.T

    T[:3, 3] = -T[:3, :3] @ centroid_A.T + centroid_B.T

    return T


def frame_inv(T: np.array) -> np.array:
    """
    Compute the inverse of a frame

    :param np.array T: input frame
    :return np.array: output (inverse) frame
    """

    R = T[:3, :3]
    t = T[:3, 3:]
    upper = np.hstack((R.T, -R.T @ t))
    lower = np.array([0.0, 0.0, 0.0, 1.0])

    return np.vstack((upper, lower))


def translation_transformation(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.array:
    """
    Returns at translation matrix

    :param list translation: translation x, y and z, defaults to [0., 0., 0.]
    :return np.array: _description_
    """
    transform = np.eye(4)

    transform[0, 3] = x
    transform[1, 3] = y
    transform[2, 3] = z

    return transform


def rotation_transformation(theta: float, axis: str = "z") -> np.array:
    """
    Returns a 4x4 transformation matrix that is a pure rotation

    :param float theta: rotation angle
    :param str axis: rotation axis, defaults to "z"
    :return np.array: the transformation matrix
    """
    axises = {"x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "z": [0.0, 0.0, 1.0]}
    frame = np.eye(4)
    frame[:3, :3] = axangles.axangle2mat(axises[axis], theta)

    return frame


def average_transforms(frameA: np.array, frameB: np.array, alpha: float = 0.5) -> np.array:
    """
    Averages two transforms

    :param np.array frameA: first
    :param np.array frameB: second transform
    :param float alpha: interpolation coefficient (0: A, 1: B), defaults to 0.5
    :return np.array: averaged frame
    """
    frame = np.eye(4)

    RA = frameA[:3, :3]
    RB = frameB[:3, :3]
    delta = RB @ RA.T
    axis, angle = axangles.mat2axangle(delta)
    frame[:3, :3] = axangles.axangle2mat(axis, angle * alpha) @ RA

    posA = frameA[:3, 3]
    posB = frameB[:3, 3]
    frame[:3, 3] = posA + alpha * (posB - posA)

    return frame

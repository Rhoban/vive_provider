#!/usr/bin/python3
import copy
import time
import openvr
import vive_utils
import math
import sys
import json
import numpy as np
from vive_utils import *
import numpy.linalg as linalg
from functools import reduce
import os
from vive_pb2 import *
from transforms3d import quaternions
import socket


class Calibration:
    def __init__(self, calibration_file_path: str):
        """
        Initializes the calibration

        :param str calibration_file_path: the calibration file path
        """
        self.calibration = None

        if os.path.exists(calibration_file_path):
            with open(calibration_file_path, "r") as calibration:
                self.calibration = json.loads(calibration.read())
            for serial_number in self.calibration:
                self.calibration[serial_number] = np.array(self.calibration[serial_number])
        else:
            print("! WARNING: Calibration file not found")

    def reference_calibration(self, serial_number: str):
        """
        Returns the transformation (T_reference_field) of the given reference if it exists
        in calibration, else None

        :param str serial_number: the reference's serial number
        :return ?np.array: frame
        """

        if self.calibration is not None and serial_number in self.calibration:
            return self.calibration[serial_number]

        return None

    def transform_frame(self, references: dict, T_world_tracker: np.array) -> np.array:
        """
        Transform a frame to the calibration frame

        :param dict references: dict mapping references to the T_world_reference frame
        :param np.array T_world_tracker: obtained transforamtion
        :return np.array: T_field_tracker
        """
        if self.calibration is None:
            # No calibration, we keep the input as it is
            return T_world_tracker

        # The list of frames found
        frames: list = []

        # Using reference lighthouses to transform tracker to the field
        for serial_number in references:
            if serial_number in self.calibration:
                T_world_reference = references[serial_number]
                T_field_reference = self.calibration[serial_number]

                T_reference_tracker = vive_utils.frame_inv(T_world_reference) @ T_world_tracker
                T_field_tracker = T_field_reference @ T_reference_tracker

                frames.append(T_field_tracker)

        if len(frames) > 0:
            # In case multiple references are found, average them
            k = 1
            frame = frames[0]
            for other_frame in frames[1:]:
                k += 1
                frame = average_transforms(frame, other_frame, 1 / k)

            return frame
        else:
            print("! ERROR: Cant find a suitable reference!")
            exit()

    def check_consistency(self, references: dict) -> None:
        """
        Checks the consistency between calibration and current frame
        (Print warning/errors)

        :param dict references: dict mapping reference names to frames
        """

        if self.calibration is None:
            return

        keys = list(references.keys())
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                keyA = keys[i]
                keyB = keys[j]

                if keyA in self.calibration and keyB in self.calibration:
                    expected_transformation = self.calibration[keyB] * np.linalg.inv(self.calibration[keyA])
                    transformation = np.linalg.inv(references[keyB]) * references[keyA]

                    # XXX: Why do we check that? The references to world frame can be wrong but still keep correct
                    #      relatively; the world is an arbitrary frame (if the reference frame changes w.r.t the
                    #      world and the tracker change the same way it doesn't affect the final frame)
                    # XXX: We could use norms and angular difference here instead
                    # if not np.allclose(expected_transformation.T[3, :3], transformation.T[3, :3], atol=0.15):
                    #     print("! ERROR: Translation from %s to %s is not consistent with calibration" % (keyA, keyB))
                    # elif not np.allclose(expected_transformation[:3, :3], transformation[:3, :3], atol=0.05):
                    #     print("! ERROR: Rotation from %s to %s is not consistent with calibration" % (keyA, keyB))


class ViveProvider:
    """
    Provides information from the Vive, can be run directly or as a client mode that listens on the network
    """

    def __init__(self, enable_buttons=False, calibration_file_path=CALIBRATION_FILENAME, client_mode=False):
        self.client_mode: bool = client_mode

        # Last infos (from previous call)
        self.last_infos = None

        # Should we check for button pressed?
        # For some reason, requesting button states mess up with the orientation objets, that is why
        # we only use them for tagging positions and field
        self.enable_buttons = enable_buttons

        # Tagged positions
        self.tagged_positions = []

        if self.client_mode:
            # Listening for UDP frames
            self.vr = None
            self.calib = None

            self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.client.bind(("", VIVE_SERVER_PORT))
        else:
            # Initalizng OpenVR
            self.vr = openvr.init(openvr.VRApplication_Other)
            self.calibration = Calibration(calibration_file_path)
            if os.path.exists(TAGGED_POSITIONS_FILENAME):
                f = open(TAGGED_POSITIONS_FILENAME, "r")
                self.tagged_positions = json.load(f)
                f.close()

    def get_tracker_infos(self, raw: bool = False) -> dict:
        """
        Get all information in a dict

        :param bool raw: should we not use calibration ?, defaults to False
        :return dict: a dictionary containing detection information
        """

        if self.client_mode:
            # In client mode, receive the next packet from the network and parses it
            pb_msg = GlobalMsg()
            data, _ = self.client.recvfrom(1024)
            pb_msg.ParseFromString(data)

            return GlobalMsg_to_tracker_infos(data)

        infos = {}

        # Steady monotonic clock and system clock
        infos["vive_timestamp"] = int(time.perf_counter() * 1000000)
        infos["time_since_epoch"] = int(time.time() * 1000000)

        # Pre-tagged positions are always included
        infos["tagged_positions"] = self.tagged_positions

        infos["references"] = {}
        infos["trackers"] = {}

        # Request informations
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                        openvr.k_unMaxTrackedDeviceCount)

        trackers = []

        for index in range(1, openvr.k_unMaxTrackedDeviceCount):
            if poses[index].bDeviceIsConnected:
                pose = poses[index]
                device_class = openvr.VRSystem().getTrackedDeviceClass(index)
                serial_number = openvr.VRSystem().getStringTrackedDeviceProperty(index, openvr.Prop_SerialNumber_String)

                if device_class == openvr.TrackedDeviceClass_TrackingReference:
                    # Feeding the references
                    infos["references"][serial_number] = tracker_to_matrix(pose)
                elif device_class == openvr.TrackedDeviceClass_Controller:
                    # Controller (joystick)
                    trackers.append((serial_number, "controller", pose, index))
                elif device_class == openvr.TrackedDeviceClass_GenericTracker:
                    # Generic tracker
                    trackers.append((serial_number, "tracker", pose, index))
                else:
                    print(f"Unknown class: {device_class}")

        # References corrected in calibrated frame
        infos["calibration"] = {}
        for serial_number in infos["references"]:
            T_world_reference = self.calibration.reference_calibration(serial_number)

            if T_world_reference is not None:
                infos["calibration"][serial_number] = {
                    "position": T_world_reference[:3, 3],
                    "orientation": quaternions.mat2quat(T_world_reference[:3, :3]),
                }

        if not raw:
            self.calibration.check_consistency(infos["references"])

        # Tracker information
        for serial_number, device_type, pose, openvr_index in trackers:

            entry = {}
            T_world_tracker = tracker_to_matrix(pose)

            if device_type == "controller":
                T_world_tracker = T_world_tracker @ translation_transformation(0, -0.025, -0.025)

            if device_type == "tracker":
                # These transformations are here to switch from the official frame to
                # our frame where the tracker LED is along the X axis
                T_world_tracker = T_world_tracker @ rotation_transformation(math.pi, "y")
                T_world_tracker = T_world_tracker @ rotation_transformation(math.pi / 2, "z")
                # T_world_tracker = T_world_tracker @ translation_transformation(0, 0, -0.01)

            if not raw:
                T_world_tracker = self.calibration.transform_frame(infos["references"], T_world_tracker)

            entry["openvr_index"] = openvr_index
            entry["serial_number"] = serial_number
            entry["device_type"] = device_type
            entry["position"] = T_world_tracker[:3, 3]
            entry["orientation"] = quaternions.mat2quat(T_world_tracker[:3, :3])
            entry["velocity"] = [
                pose.vVelocity[0],
                pose.vVelocity[1],
                pose.vVelocity[2],
            ]
            entry["angular_velocity"] = [
                pose.vAngularVelocity[0],
                pose.vAngularVelocity[1],
                pose.vAngularVelocity[2],
            ]

            if pose.bPoseIsValid or self.last_infos is None or serial_number not in self.last_infos["trackers"]:
                entry["vive_timestamp_last_tracked"] = infos["vive_timestamp"]
                entry["time_since_last_tracked"] = 0
            else:
                entry["vive_timestamp_last_tracked"] = self.last_infos["trackers"][serial_number][
                    "vive_timestamp_last_tracked"
                ]
                entry["time_since_last_tracked"] = (
                        infos["vive_timestamp"] - self.last_infos["trackers"][serial_number][
                    "vive_timestamp_last_tracked"]
                )

            # Checking for buttons press
            if self.enable_buttons:
                if entry["device_type"] == "controller":
                    _, state = self.vr.getControllerState(openvr_index)
                    entry["button_pressed"] = state.ulButtonPressed != 0

            infos["trackers"][serial_number] = entry

        if self.last_infos is not None:
            infos["trackers"] = {**self.last_infos["trackers"], **infos["trackers"]}

        # Keeping last infos, if a tracker is not present, keeping the previous one
        self.last_infos = copy.deepcopy(infos)

        return infos

    def get_tracker_infos_without_calibration(self, raw: bool = False, tracker_calibration_name: dict = None) -> dict:
        """
        Get all information in a dict

        :param bool raw: should we not use calibration ?, defaults to False
        :return dict: a dictionary containing detection information
        """

        if self.client_mode:
            # In client mode, receive the next packet from the network and parses it
            pb_msg = GlobalMsg()
            data, _ = self.client.recvfrom(1024)
            pb_msg.ParseFromString(data)

            return GlobalMsg_to_tracker_infos(data)

        infos = {}

        # Steady monotonic clock and system clock
        infos["vive_timestamp"] = int(time.perf_counter() * 1000000)
        infos["time_since_epoch"] = int(time.time() * 1000000)

        # Pre-tagged positions are always included
        infos["tagged_positions"] = self.tagged_positions

        infos["references"] = {}
        infos["trackers"] = {}

        # Request informations
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                        openvr.k_unMaxTrackedDeviceCount)

        trackers = []

        for index in range(1, openvr.k_unMaxTrackedDeviceCount):
            if poses[index].bDeviceIsConnected:
                pose = poses[index]
                device_class = openvr.VRSystem().getTrackedDeviceClass(index)
                serial_number = openvr.VRSystem().getStringTrackedDeviceProperty(index, openvr.Prop_SerialNumber_String)

                if device_class == openvr.TrackedDeviceClass_TrackingReference:
                    # Feeding the references
                    infos["references"][serial_number] = tracker_to_matrix(pose)
                elif device_class == openvr.TrackedDeviceClass_Controller:
                    # Controller (joystick)
                    trackers.append((serial_number, "controller", pose, index))
                elif device_class == openvr.TrackedDeviceClass_GenericTracker:
                    # Generic tracker
                    trackers.append((serial_number, "tracker", pose, index))
                else:
                    print(f"Unknown class: {device_class}")

        # References corrected in calibrated frame
        infos["calibration"] = {}
        for serial_number in infos["references"]:
            T_world_reference = self.calibration.reference_calibration(serial_number)

            if T_world_reference is not None:
                infos["calibration"][serial_number] = {
                    "position": T_world_reference[:3, 3],
                    "orientation": quaternions.mat2quat(T_world_reference[:3, :3]),
                }

        # Tracker information
        for serial_number, device_type, pose, openvr_index in trackers:

            entry = {}
            T_world_tracker = tracker_to_matrix(pose)

            if device_type == "controller":
                T_world_tracker = T_world_tracker @ translation_transformation(0, -0.025, -0.025)

            if device_type == "tracker":
                # These transformations are here to switch from the official frame to
                # our frame where the tracker LED is along the X axis
                T_world_tracker = T_world_tracker @ rotation_transformation(math.pi, "y")
                T_world_tracker = T_world_tracker @ rotation_transformation(math.pi / 2, "z")
                id = serial_number
                if id in tracker_calibration_name.keys():
                    # subtract the size of the stud (4.2 cm) + grass (0.1 cm) -> -0.0043 m
                    T_world_tracker = T_world_tracker @ translation_transformation(0, 0, -0.043)

            if not raw:
                T_world_tracker = self.calibration.transform_frame(infos["references"], T_world_tracker)

            entry["openvr_index"] = openvr_index
            entry["serial_number"] = serial_number
            entry["device_type"] = device_type
            entry["position"] = T_world_tracker[:3, 3]
            entry["orientation"] = quaternions.mat2quat(T_world_tracker[:3, :3])
            entry["velocity"] = [
                pose.vVelocity[0],
                pose.vVelocity[1],
                pose.vVelocity[2],
            ]
            entry["angular_velocity"] = [
                pose.vAngularVelocity[0],
                pose.vAngularVelocity[1],
                pose.vAngularVelocity[2],
            ]

            if pose.bPoseIsValid or self.last_infos is None or serial_number not in self.last_infos["trackers"]:
                entry["vive_timestamp_last_tracked"] = infos["vive_timestamp"]
                entry["time_since_last_tracked"] = 0
            else:
                entry["vive_timestamp_last_tracked"] = self.last_infos["trackers"][serial_number][
                    "vive_timestamp_last_tracked"
                ]
                entry["time_since_last_tracked"] = (
                        infos["vive_timestamp"] - self.last_infos["trackers"][serial_number][
                    "vive_timestamp_last_tracked"]
                )

            # Checking for buttons press
            if self.enable_buttons:
                if entry["device_type"] == "controller":
                    _, state = self.vr.getControllerState(openvr_index)
                    entry["button_pressed"] = state.ulButtonPressed != 0

            infos["trackers"][serial_number] = entry

        if self.last_infos is not None:
            infos["trackers"] = {**self.last_infos["trackers"], **infos["trackers"]}

        # Keeping last infos, if a tracker is not present, keeping the previous one
        self.last_infos = copy.deepcopy(infos)

        return infos

    def vibrate(self, index: int, duration: int = 250):
        """
        Gets a controller to vibrate

        :param int index: controller's index
        :param int duration: duration of the vibration (ms), defaults to 250
        """
        for _ in range(duration):
            self.vr.triggerHapticPulse(index, 0, 5000)
            time.sleep(1 / 1000.0)

    def get_controllers_infos(self, raw: bool = True) -> list:
        """
        Returns a list of controller information (dict)

        :param bool raw: should we use raw information, defaults to True
        :return list: list of dict (controller-type trackers)
        """
        trackers = self.get_tracker_infos(raw)["trackers"]
        controllers = filter(lambda entry: entry["device_type"] == "controller", trackers.values())

        return list(controllers)

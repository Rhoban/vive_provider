#!/usr/bin/env python
import json
import math
import numpy as np
import pybullet as p
from time import sleep
from vive_provider import ViveProvider
from transforms3d import quaternions
import vive_utils


class BulletViewer:
    """
    Handles the rendering of the 3D environment using pyBullet
    """

    def __init__(self, vive: ViveProvider, trackers_calibration: bool = False):
        # Initialisation de pyBullet
        physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)

        # Loading the field
        field = p.loadURDF("assets/field/robot.urdf", [0, 0, -0.01])

        # Vive Provider
        self.vive: ViveProvider = vive

        # Should we tick physics ?
        self.physics: bool = False

        # Mapping serial numbers to pyBullet object indexes (created on first encounter)
        self.trackers: dict = {}
        self.references: dict = {}
        self.positions: dict = {}
        self.texts = {}
        
        
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)

        self.trackers_calibration = trackers_calibration
        self.calib_trackers = {}
        
        if trackers_calibration:
            f = open(vive_utils.FIELD_POINTS_TRACKERS_FILENAME, "r")
            self.calib_trackers = json.load(f)
            f.close()

    @staticmethod
    def add_urdf(path: str) -> int:
        """
        Adds an URDF file to the scene

        :param str path: path to the URDF file
        :return int: index of object in the pyBullet scene
        """
        return p.loadURDF(path, [0, 0, 0])

    @staticmethod
    def set_urdf_pose(urdf: int, position: list, orientation=None) -> None:
        """
        Sets the pose of an object

        :param int urdf: the object identifier
        :param list position:
        :param _type_ orientation: orientation, if None is given it will be 0, defaults to None
        """
        if orientation is None:
            orientation = p.getQuaternionFromEuler([0, 0, 0])

        p.resetBasePositionAndOrientation(urdf, position, orientation)

    def update(self):
        """
        Reads tracker infos and update the 3D object positions accordingly
        """
        # Calibrate tracker info
        if self.trackers_calibration:
            infos = self.vive.get_tracker_infos_without_calibration(tracker_calibration_name=self.calib_trackers)
            infos = vive_utils.calib_position(self.calib_trackers, infos)
        else:
            infos = self.vive.get_tracker_infos()

        # Quaternions are represented x, y, z, w in pyBullet
        def quaternions_flip(q):
            w, x, y, z = q
            return x, y, z, w

        # Updating tagged position arrows
        for index in range(len(infos["tagged_positions"])):
            position = infos["tagged_positions"][index]

            if index not in self.positions:
                self.positions[index] = self.add_urdf("assets/target/robot.urdf")

            self.set_urdf_pose(self.positions[index], position)
        # XXX: What happens if a position disappear ?

        # Updating tracker positions
        for serial_number in infos["trackers"]:
            info = infos["trackers"][serial_number]

            if serial_number not in self.trackers:
                startOrientation = p.getQuaternionFromEuler([0, 0, 0])
                startPos = [0, 0, 0]

                asset = "assets/tracker.urdf"
                if info["device_type"] == "controller":
                    asset = "assets/controller.urdf"
                tracker = p.loadURDF(asset, startPos, startOrientation)

                self.trackers[serial_number] = tracker

            p.resetBasePositionAndOrientation(
                self.trackers[serial_number], info["position"], quaternions_flip(info["orientation"])
            )

        # Updating reference lighthouses
        if "references" in infos:
            for serial_number in infos["references"]:
                if serial_number not in self.references:
                    self.references[serial_number] = p.loadURDF("assets/lighthouse/robot.urdf", [0, 0, 0])

                # We draw using the references from the calibration
                if serial_number in infos["calibration"]:
                    info = infos["calibration"][serial_number]
                else:
                    info = infos["references"][serial_number]

                p.resetBasePositionAndOrientation(
                    self.references[serial_number], info["position"], quaternions_flip(info["orientation"])
                )

                if serial_number not in self.texts:
                    self.texts[serial_number] = p.addUserDebugText(serial_number, info["position"])

    def execute(self):
        """
        Starts the rendering
        """
        p.setRealTimeSimulation(1)
        dt: float = 0.001
        t: float = 0
        p.setPhysicsEngineParameter(fixedTimeStep=dt)

        while True:
            self.update()
            sleep(dt)

            if self.physics:
                t += dt
                p.stepSimulation()


if __name__ == "__main__":
    vive = ViveProvider()
    viewer = BulletViewer(vive)
    viewer.physics = False
    viewer.execute()

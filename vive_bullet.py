#!/usr/bin/env python
import math
import numpy as np
import pybullet as p
from time import sleep
from vive_provider import Vive_provider
from utils import convert_to_euler

class BulletViewer:
    def __init__(self, vive):        
        # Initialisation de pyBullet
        physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        field = p.loadURDF("assets/field/robot.urdf", [0, 0, -0.01])

        self.vive = vive
        self.trackers = {}
        self.references = {}
        self.physics = False
        self.offset = None
        self.texts= {}

        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING,1)

        infos = self.vive.getTrackersInfos()

        for id in self.vive.trackers:
            tracker = self.vive.trackers[id]
            info = infos['trackers'][id]
            startOrientation = p.getQuaternionFromEuler([0, 0, 0])
            startPos = [0, 0, 0]
            
            asset = 'assets/tracker.urdf'
            # print(info)
            if info['device_type'] == 'controller':
                asset = 'assets/controller.urdf'
            tracker = p.loadURDF(asset, startPos, startOrientation)
            self.trackers[id] = tracker

        for position in infos['tagged_positions']:
            target = self.addUrdf('assets/target/robot.urdf')
            self.setUrdfPosition(target, position)

        for id in self.vive.references:
            reference = p.loadURDF('assets/lighthouse/robot.urdf', [0, 0, 0])
            self.references[id] = reference
    
    def addUrdf(self, path):
        return p.loadURDF(path, [0, 0, 0])

    def setUrdfPosition(self, urdf, position, orientation=[0, 0, 0]):
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(urdf, position, orientation)

    def update(self):
        infos = self.vive.getTrackersInfos()

        for id in self.vive.trackers:
            info = infos['trackers'][id]
            m = info['pose_matrix']
            position = np.array(m.T[3])[0][:3]

            # if info['device_type'] == 'controller' and self.offset is None and np.linalg.norm(position)>0.5:
            #     self.offset = position.copy()
            #     print('Defining')
            #     print(self.offset)
            
            # if self.offset is not None:
            #     position -= self.offset

            orientation = m[:3,:3]
            euler = convert_to_euler(orientation)
            orientation = p.getQuaternionFromEuler(euler)

            # fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
            # projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
            # tp = position.copy()
            # tp[2] += 0.1
            # view_matrix = p.computeViewMatrix(tp, tp+[-math.sin(euler[2]),math.cos(euler[2]),0], [0, 0, 1])
            # img = p.getCameraImage(1000, 1000, view_matrix, projection_matrix)

            # c = position.copy()
            #  p.resetDebugVisualizerCamera(0.2, cameraYaw=euler[2]*180/math.pi, cameraPitch=-120+euler[0]*180.0/math.pi, ,cameraTargetPosition=c)

            # print(position)
            p.resetBasePositionAndOrientation(self.trackers[id], position, orientation)
        
        for id in self.vive.references:
            m = infos['references_corrected'][id]
            position = np.array(m.T[3])[0][:3]
            orientation = m[:3,:3]
            euler = convert_to_euler(orientation)
            orientation = p.getQuaternionFromEuler(euler)

            if id not in self.texts:
                self.texts[id] = p.addUserDebugText(id, position)
            p.resetBasePositionAndOrientation(self.references[id], position, orientation)

    
    def execute(self):
        # Simulation en temps réel
        p.setRealTimeSimulation(1)
        dt = 0.001
        t = 0
        p.setPhysicsEngineParameter(fixedTimeStep=dt)

        while True:
            self.update()
            sleep(dt)

            if self.physics:
                t += dt
                p.stepSimulation()

if __name__ == '__main__':
    vive = Vive_provider()
    viewer = BulletViewer(vive)
    viewer.physics = False
    viewer.execute()

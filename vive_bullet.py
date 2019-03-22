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
        self.physics = False

        infos = vive.getTrackersInfos()
        for id in vive.trackers:
            tracker = vive.trackers[id]
            info = infos['trackers'][id]
            startOrientation = p.getQuaternionFromEuler([0, 0, 0])
            startPos = [0, 0, 0]
            
            asset = 'assets/tracker.urdf'
            if info['device_type'] == 'controller':
                asset = 'assets/controller.urdf'
            tracker = p.loadURDF(asset, startPos, startOrientation)
            self.trackers[id] = tracker
    
    def addUrdf(self, path):
        return p.loadURDF(path, [0, 0, 0])

    def setUrdfPosition(self, urdf, position, orientation=[0, 0, 0]):
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(urdf, position, orientation)

    def update(self):
        infos = vive.getTrackersInfos()
        for id in self.vive.trackers:
            info = infos['trackers'][id]
            m = info['pose_matrix']
            position = np.array(m.T[3])[0][:3]
            orientation = m[:3,:3]
            euler = convert_to_euler(orientation)
            orientation = p.getQuaternionFromEuler(euler)

            if id == '4':
                print("%f\t%f\t%f\t" % tuple(euler))

            p.resetBasePositionAndOrientation(self.trackers[id], position, orientation)
    
    def execute(self):
        # Simulation en temps réel
        p.setRealTimeSimulation(1)
        dt = 0.001
        t = 0
        p.setPhysicsEngineParameter(fixedTimeStep=dt)
        offset = None

        while True:
            self.update()

            if self.physics:
                sleep(dt)
                t += dt
                p.stepSimulation()

if __name__ == '__main__':
    vive = Vive_provider()
    viewer = BulletViewer(vive)
    viewer.physics = False
    viewer.execute()
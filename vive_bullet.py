import math
import numpy as np
import pybullet as p
from time import sleep
from vive_provider import Vive_provider
from utils import convert_to_euler

# Initializing vive, getting trackers
vive = Vive_provider()

# Initialisation de pyBullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
field = p.loadURDF("assets/field/robot.urdf", [0, 0, -0.01])

trackers = {}
offset = None
infos = vive.getTrackersInfos()
for id in vive.trackers:
    tracker = vive.trackers[id]
    info = infos['tracker_'+str(id)]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    startPos = [0, 0, 0]
    
    asset = 'assets/tracker.urdf'
    if info['device_type'] == 'controller':
        asset = 'assets/controller.urdf'
    tracker = p.loadURDF(asset, startPos, startOrientation)
    trackers[id] = tracker

# Simulation en temps réel
p.setRealTimeSimulation(0)
dt = 0.001
t = 0
p.setPhysicsEngineParameter(fixedTimeStep=dt)
offset = None

while True:
    infos = vive.getTrackersInfos()
    
    for id in vive.trackers:
        info = infos['tracker_'+str(id)]
        m = info['pose_matrix']
        position = np.array(m.T[3])[0][:3]

        if offset is None:
            offset = position.copy()

        if not (offset is None):
            position -= offset

        orientation = m[:3,:3]
        euler = convert_to_euler(orientation)
        orientation = p.getQuaternionFromEuler(euler)

        if id == '4':
            print("%f\t%f\t%f\t" % tuple(euler))

        p.resetBasePositionAndOrientation(trackers[id], position, orientation)

    # Uncomment if you want to enable physics simulation
    # sleep(dt)
    # t += dt
    # p.stepSimulation()

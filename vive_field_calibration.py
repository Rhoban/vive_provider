#!/usr/bin/env python

import json
from vive_provider import *
from vive_bullet import BulletViewer
from utils import rigid_transform_3D

vp = Vive_provider(enableButtons=True)

# Positions 
field_positions = [
    [-2.37, 0, 0],
    [0, 3, 0],
    [0, -3, 0]
]
        
data = []
viewer = BulletViewer(vp)
target = viewer.addUrdf('assets/target/robot.urdf')

controllersInfos = vp.getControllersInfos(raw=True)
if len(controllersInfos) != 1:
    print('ERROR: Calibration should have exactly one controller (found %d)' % len(controllersInfos))
    exit()

failed = True
positions = []
while failed:
    failed = False
    positions = []
    for field_position in field_positions:
        print('Place tracker on position '+str(field_position)+', then press the button ')
        viewer.setUrdfPosition(target, field_position)

        # Waiting for button to be released
        while vp.getControllersInfos(raw=True)[0]['buttonPressed']:
            time.sleep(0.01)
        # Waitng for button to be pressed
        while not vp.getControllersInfos(raw=True)[0]['buttonPressed']:
            time.sleep(0.01)

        pose = vp.getControllersInfos(raw=True)[0]['pose']

        # print(pose[0], pose[1], pose[2])
        position = pose[:3]

        hasError = False
        for k in range(len(positions)):
            other_field_position = field_positions[k]
            other_position = positions[k]

            expected_dist = np.linalg.norm(np.array(other_field_position) - np.array(field_position))
            dist = np.linalg.norm(np.array(other_position) - np.array(position))
            error = abs(dist-expected_dist)

            if error > 0.05:
                print('! ERROR: Expected distance is wrong (expected: %f m, got: %f m, error: %f m)!' % (expected_dist, dist, error))
                vp.vibrate(vp.getControllersInfos(raw=True)[0]['serial_number'])
                failed = True
                hasError = True
        if hasError:
            break
                
        positions.append(position)

# Computing worldToField matrix
R, t = rigid_transform_3D(np.mat(positions), np.mat(field_positions))
worldToField = R.copy()
worldToField = np.hstack((worldToField, t))
worldToField = np.vstack((worldToField, [0, 0, 0, 1]))

# XXX: References should be built all along the script
data = {}
data['worldToField'] = worldToField.tolist()
infos = vp.getTrackersInfos(raw=True)
for reference in infos['references']:
    referenceToWorld = infos['references'][reference]
    fieldToReference = np.linalg.inv(worldToField*referenceToWorld).tolist()
    data[reference] = fieldToReference

calibFile = open("calibFile.json", "w")
calibFile.write(json.dumps(data))
calibFile.close()

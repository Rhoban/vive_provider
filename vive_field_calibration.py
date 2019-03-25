#!/usr/bin/env python

import json
from vive_provider import *
from vive_bullet import BulletViewer

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

    print(pose[0], pose[1], pose[2])
    data.append({
        'field': field_position,
        'vive': pose[:3]
    })
    
calibFile = open("calibFile.json", "w")
calibFile.write(json.dumps(data))
calibFile.close()
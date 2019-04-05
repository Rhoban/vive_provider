#!/usr/bin/env python
import json, sys
from vive_provider import *
from vive_bullet import BulletViewer
from utils import rigid_transform_3D

# Erasing tagged positions
f = open('taggedPositions.json', 'w')
f.write(json.dumps([]))
f.close()

vp = Vive_provider(enableButtons=True)
        
# Creating bullet viewer and loading the field
viewer = BulletViewer(vp)

# Checking controller presence
controllersInfos = vp.getControllersInfos(raw=True)
if len(controllersInfos) != 1:
    print('ERROR: Tagging positions should have exactly one controller (found %d)' % len(controllersInfos))
    exit()

# Loop to retrieve the points
failed = True
positions = []
while failed:
    failed = False
    positions = []
    references = {}
    buttonState = 0
    print('Place the controller on tag position and press the button')
    while True:
        viewer.update()

        buttonPressed = vp.getControllersInfos()[0]['buttonPressed']
        if buttonState == 0:
            if buttonPressed:
                buttonState += 1
        elif buttonState == 1:
            if not buttonPressed:
                buttonState += 1
        elif buttonState == 2:
            buttonState = 0

            pose = vp.getControllersInfos()[0]['pose']
            position = pose[:3]

            target = viewer.addUrdf('assets/target/robot.urdf')
            viewer.setUrdfPosition(target, position)
                    
            positions.append(position)

            f = open('taggedPositions.json', 'w')
            f.write(json.dumps(positions))
            f.close()


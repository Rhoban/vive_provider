#!/usr/bin/env python
import json, sys
from vive_provider import *
from vive_bullet import BulletViewer

print("Erasing previously tagged positions...")
f = open(TAGGED_POSITIONS_FILENAME, "w")
f.write(json.dumps([]))
f.close()

# Create a Vive Povider
vp = ViveProvider(enable_buttons=True)

# Creating bullet viewer and loading the field
viewer = BulletViewer(vp)

# Checking controller presence
controllers = vp.get_controllers_infos()
if len(vp.get_controllers_infos()) != 1:
    print("ERROR: Tagging positions should have exactly one controller (found %d)" % len(controllers))
    exit()

# Loop to retrieve the points
positions: list = []
references: dict = {}
button_state: int = 0
print("Place the controller on tag position and press the button")

while True:
    viewer.update()

    button_pressed = vp.get_controllers_infos()[0]["button_pressed"]

    # Waiting for button to be relased and then pressed without blocking the viewer
    if button_state == 0:
        if button_pressed:
            button_state += 1
    elif button_state == 1:
        if not button_pressed:
            button_state += 1
    elif button_state == 2:
        button_state = 0

        pose = vp.get_controllers_infos(raw=False)[0]["pose"]
        position = pose[:3]

        # Showing the position in the viewer
        target = viewer.addUrdf("assets/target/robot.urdf")
        viewer.setUrdfPosition(target, position)

        positions.append(position)

        # Vibration indicates the point was added
        for k in range(len(positions)):
            vp.vibrate(vp.get_controllers_infos(raw=True)[0]["openvr_index"], 200)
            time.sleep(0.25)

        print("Tagged %d positions, updating %s" % (len(positions), TAGGED_POSITIONS_FILENAME))
        f = open(TAGGED_POSITIONS_FILENAME, "w")
        f.write(json.dumps(positions))
        f.close()

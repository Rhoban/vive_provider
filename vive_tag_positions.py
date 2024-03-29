#!/usr/bin/env python
import json, sys
from vive_provider import *
from vive_bullet import BulletViewer

print("Erasing previously tagged positions...")
f = open(TAGGED_POSITIONS_FILENAME, "w")
f.write(json.dumps([]))
f.close()

f = open(FIELD_POINTS_TRACKERS_FILENAME, "r")
calib_trackers = json.load(f)
f.close()

# Create a Vive Povider
vp = ViveProvider(enable_buttons=True)

# Creating bullet viewer and loading the field
viewer = BulletViewer(vp)

# Checking controller presence
# controllers = vp.get_controllers_infos()
controllers = vp.get_tracker_infos_without_calibration(tracker_calibration_name=calib_trackers)

if len(controllers) != 1:
    print("ERROR: Tagging positions should have exactly one controller (found %d)" % len(controllers))
    exit()

# Calibrate all trackers
controllers = vive_utils.calib_position(calib_trackers, controllers)

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

        position = vp.get_controllers_infos(raw=False)[0]["position"].tolist()

        # Showing the position in the viewer
        target = viewer.add_urdf("assets/target/robot.urdf")
        viewer.set_urdf_pose(target, position)

        positions.append(position)

        # Vibration indicates the point was added
        for k in range(len(positions)):
            vp.vibrate(vp.get_controllers_infos(raw=True)[0]["openvr_index"], 200)
            time.sleep(0.25)

        print("Tagged %d positions, updating %s" % (len(positions), TAGGED_POSITIONS_FILENAME))
        f = open(TAGGED_POSITIONS_FILENAME, "w")
        f.write(json.dumps(positions, indent=4))
        f.close()

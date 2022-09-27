#!/usr/bin/env python
import json, argparse
from vive_provider import *
from vive_bullet import BulletViewer
from vive_utils import rigid_transform_3D
import pickle

parser = argparse.ArgumentParser()
parser.add_argument("--points", "-p", type=str, default=FIELD_POINTS_FILENAME)
args = parser.parse_args()

vp = ViveProvider(enable_buttons=True)

# Creating bullet viewer and loading the field
viewer = BulletViewer(vp)

# Checking controller presence
controllers = vp.get_controllers_infos()
if len(controllers) != 1:
    print("ERROR: Calibration should have exactly one controller (found %d)" % len(controllers))
    exit()

# Loop to retrieve the points
failed = True
positions = []
while failed:
    failed = False
    positions = []
    references = {}

    for k in range(12):
        print(f"Capturing point {k}...")

        # Waiting for button to be released
        while vp.get_controllers_infos()[0]["button_pressed"]:
            time.sleep(0.01)
        # Waitng for button to be pressed
        while not vp.get_controllers_infos()[0]["button_pressed"]:
            time.sleep(0.01)

        position = vp.get_controllers_infos()[0]["position"]

        has_error: bool = False

        # Checking references
        infos = vp.get_tracker_infos(raw=True)
        for reference in infos["references"]:
            matrix = infos["references"][reference]
            if reference in references:
                if not np.allclose(references[reference], matrix):
                    print("! ERROR: References has moved!")
                    has_error = True
            else:
                references[reference] = matrix

        positions.append(position)

f = open('captured_positions.pickle', 'wb')
pickle.dump(positions, f)
f.close()


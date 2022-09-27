#!/usr/bin/env python
import json, argparse
from vive_provider import *
from vive_bullet import BulletViewer
from vive_utils import rigid_transform_3D

parser = argparse.ArgumentParser()
parser.add_argument("--points", "-p", type=str, default=FIELD_POINTS_FILENAME)
args = parser.parse_args()

vp = ViveProvider(enable_buttons=True)

# Loading field positions to use for calibration
f = open(args.points, "r")
field_positions = json.load(f)
f.close()

# Creating bullet viewer and loading the field
viewer = BulletViewer(vp)
target = viewer.add_urdf("assets/target/robot.urdf")

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

    for field_position in field_positions:
        print("Place tracker on position " + str(field_position) + ", then press the button ")
        viewer.set_urdf_pose(target, field_position)

        # Waiting for button to be released
        while vp.get_controllers_infos()[0]["button_pressed"]:
            time.sleep(0.01)
        # Waiting for button to be pressed
        while not vp.get_controllers_infos()[0]["button_pressed"]:
            time.sleep(0.01)

        position = vp.get_controllers_infos()[0]["position"]

        has_error: bool = False

        # Checking references
        infos = vp.get_tracker_infos(raw=True)
        for reference in infos["references"]:
            matrix = infos["references"][reference]
            # if the base station is not saved in references
            if reference in references:
                if not np.allclose(references[reference], matrix):
                    print("! ERROR: References has moved!")
                    has_error = True
            else:
                # We add the base station to the references list with his matrix
                references[reference] = matrix

        # Checking points distances consistency
        for k in range(len(positions)):
            other_field_position = field_positions[k]
            other_position = positions[k]

            expected_dist = np.linalg.norm(np.array(other_field_position) - np.array(field_position))
            dist = np.linalg.norm(np.array(other_position) - np.array(position))
            error = abs(dist - expected_dist)

            if error > 0.07:
                print(
                    "! ERROR: Expected distance is wrong (expected: %f m, got: %f m, error: %f m)!"
                    % (expected_dist, dist, error)
                )
                vp.vibrate(vp.get_controllers_infos()[0]["openvr_index"])
                failed = True
                has_error = True
        if has_error:
            break

        positions.append(position)

# Computing worldToField matrix
T_field_world = rigid_transform_3D(np.array(positions), np.array(field_positions))

# Adding field to reference to the calibration files
data = {}
for reference in references:
    T_world_reference = references[reference]
    T_field_reference = (T_field_world @ T_world_reference).tolist()
    data[reference] = T_field_reference

# Writing the calibration file
calibration = open("../calibration.json", "w")
calibration.write(json.dumps(data, indent=4))
calibration.close()

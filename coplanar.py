#!/usr/bin/env python
import argparse
import json, sys

from mpl_toolkits.mplot3d import Axes3D
from vive_provider import *
from vive_bullet import BulletViewer
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm

TAGGED_COPLANAR_POSITIONS_FILENAME: str = "tagged_positions_coplanar.json"


def drawing(positions):
    np_position = np.asarray(positions)

    fig = plt.figure("3D", figsize=plt.figaspect(0.5))

    ax = Axes3D(fig)
    min = np.min(np_position[:, 0])
    max = np.max(np_position[:, 0])

    surf = ax.plot_trisurf(np_position[:, 0], np_position[:, 1], np_position[:, 2], cmap=cm.jet, linewidth=0.1)
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlabel('X')

    color_map = cm.ScalarMappable(cmap=cm.jet)
    colo = np.array(np_position[:, 2])
    color_map.set_array(colo)

    ax.set_zlim3d(min, max)
    ax.set_ylim3d(min, max)
    ax.set_xlim3d(min, max)
    plt.colorbar(color_map)
    plt.show()



def tagging_position(total):
    print("Erasing previously tagged positions...")
    f = open(TAGGED_COPLANAR_POSITIONS_FILENAME, "w")
    f.write(json.dumps([]))
    f.close()

    # Create a Vive Povider
    vp = ViveProvider(enable_buttons=True)

    # Creating bullet viewer and loading the field
    viewer = BulletViewer(vp)

    # Checking controller presence
    controllers = vp.get_controllers_infos()
    if len(controllers) != 1:
        print("ERROR: Tagging positions should have exactly one controller (found %d)" % len(controllers))
        exit()

    # Loop to retrieve the points
    positions: list = []
    references: dict = {}
    button_state: int = 0
    print("Place the controller on tag position and press the button")

    nb_point = 0
    while nb_point < total:
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
            vp.vibrate(vp.get_controllers_infos(raw=True)[0]["openvr_index"], 200)
            time.sleep(0.25)

            print("Tagged %d positions, updating %s" % (len(positions), TAGGED_COPLANAR_POSITIONS_FILENAME))
            f = open(TAGGED_COPLANAR_POSITIONS_FILENAME, "w")
            f.write(json.dumps(positions, indent=4))
            f.close()
            nb_point += 1


parser = argparse.ArgumentParser()
parser.add_argument("--draw", "-d", type=str)
parser.add_argument("--total", "-t", type=int)
args = parser.parse_args()

if args.draw is not None:
    with open(args.draw, 'r') as f:
        data = json.load(f)
    drawing(data)
else:
    tagging_position(args.total)

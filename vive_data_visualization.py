import argparse
import errno
import os

import numpy as np
from transforms3d import euler
import matplotlib.pyplot as plt
import math
from vive_logs import ViveLog


def plot_n_trackers_3D(serial_numbers):
    """
    Draw in matplotlib the 3D representation of the data and all value

    Parameters
    ----------
    serial_numbers: set : Array of all serial numbers detected

    Returns
    -------
    3D plot and data plot
    """
    fig_3D = plt.figure("3D")
    ax_3D = fig_3D.add_subplot(projection='3d')

    for i in range(len(serial_numbers)):
        y = []
        x_position = []
        data = vive_log.get_data(serials[i])
        if data is not None:
            for line in data:
                if i == 0:
                    y_start = data[0][0]
                    y.append((line[0] - y_start) / 1000000)  # convert to second
                x_position.append([line[1].x, line[1].y, line[1].z])
                y.append((line[0] - y_start) / 1000000)

            position_xyz = np.multiply(np.asarray(x_position), 100)
            ## 3D projection
            ax_3D.scatter(position_xyz[:, 0], position_xyz[:, 1], position_xyz[:, 2], label=serials[i])
            plot_one_tracker(data, False, serials[i])
        else:
            print("ERROR, serial number " + serials[i] + " is None")

    ax_3D.set_xlabel('X')
    ax_3D.set_ylabel('Y')
    ax_3D.set_zlabel('Z')
    ax_3D.legend()
    plt.show()


def plot_one_tracker(data, is_tracker_alone, trackers: str = None):
    """
    Draw the evolution of position and orientation according to timestamps
    Parameters
    ----------
    data: list : list of position, orientation and timestamps link to a tracker
    is_tracker_alone: bool : Is data alone ?
    trackers: str : Name of the current tracker

    Returns plot :  Orientation and Position of one tracker
    -------

    """
    if data is None:
        raise ValueError("ERROR, tracker data are None !")
    y_start = data[0][0]
    y = []
    x_position = []
    x_orientation = []
    # x_velocity = []

    for line in data:
        y.append((line[0] - y_start) / 1000000)  # convert to second

        x_position.append([line[1].x, line[1].y, line[1].z])

        rpy = euler.quat2euler([line[2].qw, line[2].qx, line[2].qy, line[2].qz])
        rpy_deg_roll = rpy[0] * 180 / math.pi
        rpy_deg_pitch = rpy[1] * 180 / math.pi
        rpy_deg_yaw = rpy[2] * 180 / math.pi

        x_orientation.append([rpy_deg_roll, rpy_deg_pitch, rpy_deg_yaw])

        # x_velocity.append(line[3])

    y_np = np.asarray(y)

    position_xyz = np.multiply(np.asarray(x_position), 100)

    rotation_rpy = np.asarray(x_orientation)
    # x_velocity_np = np.asarray(x_velocity)

    fig, axs = plt.subplots(3, 2, sharex=True)
    fig.canvas.manager.set_window_title(trackers)

    axs[0, 0].plot(y_np, position_xyz[:, 0], 'r', label="position_x (cm)")
    axs[0, 0].set_title("tracker position x")

    axs[1, 0].plot(y_np, position_xyz[:, 1], 'g', label="position_y (cm)")
    axs[1, 0].set_title("tracker position y")

    axs[2, 0].plot(y_np, position_xyz[:, 2], 'b', label="position_z (cm)")
    axs[2, 0].set_title("tracker position z")

    axs[0, 1].plot(y_np, rotation_rpy[:, 0], 'r', label="rotation_roll (degrees)")
    axs[0, 1].set_title("tracker rotation roll")

    axs[1, 1].plot(y_np, rotation_rpy[:, 1], 'g', label="rotation_pitch (degrees)")
    axs[1, 1].set_title("tracker rotation pitch")

    axs[2, 1].plot(y_np, rotation_rpy[:, 2], 'b', label="rotation_yaw (degrees)")
    axs[2, 1].set_title("tracker rotation yaw")

    for plot_axis in axs[:, 0]:
        plot_axis.legend()
        plot_axis.grid()
        plot_axis.label_outer()
    for plot_axis in axs[:, 1]:
        plot_axis.legend()
        plot_axis.grid()
        plot_axis.label_outer()

    fig.tight_layout()

    if is_tracker_alone:
        plt.show()


parser = argparse.ArgumentParser()
parser.add_argument("--log_file", "-l", type=str)
parser.add_argument("--invert", "-i", action="store_true")
args = parser.parse_args()

if args.log_file is None or not os.path.isfile(args.log_file):
    raise ValueError(f"File {args.log_file} is not valid.")

serial_numbers = set()

vive_log = ViveLog(args.log_file)
for serial in vive_log.get_trackers_serial_numbers():
    serial_numbers.add(serial)
    print(serial)

serials = sorted(list(serial_numbers))

if len(serial_numbers) == 1:
    data = vive_log.get_data(serials[0])
    plot_one_tracker(data, True, serials[0])
elif len(serial_numbers) >= 1:
    plot_n_trackers_3D(serial_numbers)
else:
    raise ValueError("ERROR, no tracker to plot !")

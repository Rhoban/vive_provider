import argparse
import errno
import os

import numpy as np
from transforms3d import euler
import matplotlib.pyplot as plt
import math
from vive_logs import ViveLog
from matplotlib import cm
import json


def plot_two_trackers(data_1, data_2):
    if data_1 is None:
        raise ValueError("ERROR, tracker data are None !")
    y_start = data_1[0][0]
    y = []
    x_position_1 = []
    x_orientation_1 = []
    # x_velocity = []

    x_position_2 = []
    x_orientation_2 = []

    z_origin = 0

    for line in data_1:
        y.append((line[0] - y_start) / 1000000)  # convert to second

        x_position_1.append([line[1].x, line[1].y, line[1].z])

        rpy = euler.quat2euler([line[2].qw, line[2].qx, line[2].qy, line[2].qz])
        rpy_deg_roll = rpy[0] * 180 / math.pi
        rpy_deg_pitch = rpy[1] * 180 / math.pi
        rpy_deg_yaw = rpy[2] * 180 / math.pi

        x_orientation_1.append([rpy_deg_roll, rpy_deg_pitch, rpy_deg_yaw])

    for line in data_2:
        # z_origin = 1 - line[1].z
        z_origin = line[1].z

        x_position_2.append([line[1].x, line[1].y, z_origin])

        rpy = euler.quat2euler([line[2].qw, line[2].qx, line[2].qy, line[2].qz])
        rpy_deg_roll = rpy[0] * 180 / math.pi
        rpy_deg_pitch = rpy[1] * 180 / math.pi
        rpy_deg_yaw = rpy[2] * 180 / math.pi

        x_orientation_2.append([rpy_deg_roll, rpy_deg_pitch, rpy_deg_yaw])

    y_np = np.asarray(y)

    position_xyz_1 = np.multiply(np.asarray(x_position_1), 100)
    position_xyz_2 = np.multiply(np.asarray(x_position_2), 100)
    # position_xyz_diff = np.add(position_xyz_1, position_xyz_2)

    # calcul de la moyenne mobile
    factor = 10
    mobile_average_x = []
    mobile_average_y = []
    mobile_average_z = []
    for i in range(factor, len(position_xyz_1) - (factor - 1)):
        mobile_average_x.append(0)
        mobile_average_y.append(0)
        mobile_average_z.append(0)
        for j in range(0, factor):
            mobile_average_x[i - factor] += position_xyz_1[i * factor + j, 0]
            mobile_average_y[i - factor] += position_xyz_1[i * factor + j, 1]
            mobile_average_z[i - factor] += position_xyz_1[i * factor + j, 2]

    position_xyz_diff = np.array(mobile_average_x, mobile_average_y, mobile_average_z)
    # #calcul de l'écart type0
    # np_mobile_average_x = np.asarray(mobile_average_x)
    # np_mobile_average_y = np.asarray(mobile_average_y)
    # np_mobile_average_z = np.asarray(mobile_average_z)
    #

    rotation_rpy_1 = np.asarray(x_orientation_1)
    rotation_rpy_2 = np.asarray(x_orientation_2)
    rotation_rpy_diff = np.add(x_orientation_1, x_orientation_2)

    fig = plt.figure("Position")
    fig_orientation = plt.figure("Orientation")
    fig_3D = plt.figure("3D")

    gs = fig.add_gridspec(3, 3, hspace=0.05, wspace=0.2)
    gs_orientation = fig_orientation.add_gridspec(3, 3, hspace=0.05, wspace=0.2)

    ax_3D = fig_3D.add_subplot(projection='3d')
    axs = gs.subplots(sharex=True)
    axs_orientation = gs_orientation.subplots(sharex=True)

    ## 3D projection
    ax_3D.scatter(position_xyz_2[:, 0], position_xyz_2[:, 1], position_xyz_2[:, 2])
    ax_3D.set_xlabel('X')
    ax_3D.set_ylabel('Y')
    ax_3D.set_zlabel('Z')

    ## robot subplot position
    axs[0, 0].plot(y_np, position_xyz_1[:, 0], 'r', label="position_x")
    axs[0, 0].set_title("tracker robot")
    axs[0, 0].set_ylabel('Position (cm)')
    axs[1, 0].plot(y_np, position_xyz_1[:, 1], 'g', label="position_y")
    axs[1, 0].set_ylabel('Position (cm)')
    axs[2, 0].plot(y_np, position_xyz_1[:, 2], 'b', label="position_z")
    axs[2, 0].set_xlabel('timestamps (s)')
    axs[2, 0].set_ylabel('Position (cm)')
    for plot_axis in axs[:, 0]:
        plot_axis.legend()
        plot_axis.grid()

    ## robot subplot orientation
    axs_orientation[0, 0].plot(y_np, rotation_rpy_1[:, 0], 'r', label="rotation_roll")
    axs_orientation[0, 0].set_title("tracker robot")
    axs_orientation[0, 0].set_ylabel('Orientation (degrees)')
    axs_orientation[1, 0].plot(y_np, rotation_rpy_1[:, 1], 'g', label="rotation_pitch")
    axs_orientation[1, 0].set_ylabel('Orientation (degrees)')
    axs_orientation[2, 0].plot(y_np, rotation_rpy_1[:, 2], 'b', label="rotation_yaw")
    axs_orientation[2, 0].set_ylabel('Orientation (degrees)')
    axs_orientation[2, 0].set_xlabel('timestamps (s)')
    for plot_axis in axs_orientation[:, 0]:
        plot_axis.legend()
        plot_axis.grid()

    ## tracker subplot position
    axs[0, 1].plot(y_np, position_xyz_2[:, 0], 'r', label="position_x")
    axs[0, 1].set_title("tracker ground")
    axs[1, 1].plot(y_np, position_xyz_2[:, 1], 'g', label="position_y")
    axs[2, 1].plot(y_np, position_xyz_2[:, 2], 'b', label="position_z")
    axs[2, 1].set_xlabel('timestamps (s)')
    for plot_axis in axs[:, 1]:
        plot_axis.legend()
        plot_axis.grid()

    ## tracker subplot orientation
    axs_orientation[0, 1].plot(y_np, rotation_rpy_2[:, 0], 'r', label="rotation_roll")
    axs_orientation[0, 1].set_title("tracker ground")
    axs_orientation[1, 1].plot(y_np, rotation_rpy_2[:, 1], 'g', label="rotation_pitch")
    axs_orientation[2, 1].plot(y_np, rotation_rpy_2[:, 2], 'b', label="rotation_yaw")
    axs_orientation[2, 1].set_xlabel('timestamps (s)')
    for plot_axis in axs_orientation[:, 1]:
        plot_axis.legend()
        plot_axis.grid()

    ## Difference subplot orientation
    axs[0, 2].plot(y_np, position_xyz_diff[:, 0], 'r', label="diff_x")
    axs[0, 2].set_title("robot + ground x")
    axs[1, 2].plot(y_np, position_xyz_diff[:, 1], 'g', label="diff_y")
    axs[2, 2].plot(y_np, position_xyz_diff[:, 2], 'b', label="diffz")
    axs[2, 2].set_xlabel('timestamps (s)')
    for plot_axis in axs[:, 2]:
        plot_axis.legend()
        plot_axis.grid()

    axs_orientation[0, 2].plot(y_np, rotation_rpy_diff[:, 0], 'r', label="diff_roll")
    axs_orientation[0, 2].set_title("robot + ground x")
    axs_orientation[1, 2].plot(y_np, rotation_rpy_diff[:, 1], 'g', label="diff_pitch")
    axs_orientation[2, 2].plot(y_np, rotation_rpy_diff[:, 2], 'b', label="diff_yaw")
    axs_orientation[2, 2].set_xlabel('timestamps (s)')
    for plot_axis in axs_orientation[:, 2]:
        plot_axis.legend()
        plot_axis.grid()

    plt.show()


def plot_three_trackers(data_1, data_2, data_3):
    if data_1 is None:
        raise ValueError("ERROR, tracker data are None !")
    y_start = data_1[0][0]
    y = []
    x_position_1 = []
    x_orientation_1 = []

    x_position_2 = []
    x_orientation_2 = []

    x_position_3 = []
    x_orientation_3 = []

    for line in data_1:
        y.append((line[0] - y_start) / 1000000)  # convert to second

        x_position_1.append([line[1].x, line[1].y, line[1].z])

        rpy = euler.quat2euler([line[2].qw, line[2].qx, line[2].qy, line[2].qz])
        rpy_deg_roll = rpy[0] * 180 / math.pi
        rpy_deg_pitch = rpy[1] * 180 / math.pi
        rpy_deg_yaw = rpy[2] * 180 / math.pi

        x_orientation_1.append([rpy_deg_roll, rpy_deg_pitch, rpy_deg_yaw])

    for line in data_2:
        x_position_2.append([line[1].x, line[1].y, line[1].z])

        rpy = euler.quat2euler([line[2].qw, line[2].qx, line[2].qy, line[2].qz])
        rpy_deg_roll = rpy[0] * 180 / math.pi
        rpy_deg_pitch = rpy[1] * 180 / math.pi
        rpy_deg_yaw = rpy[2] * 180 / math.pi

        x_orientation_2.append([rpy_deg_roll, rpy_deg_pitch, rpy_deg_yaw])

    for line in data_3:
        x_position_3.append([line[1].x, line[1].y, line[1].z])

        rpy = euler.quat2euler([line[2].qw, line[2].qx, line[2].qy, line[2].qz])
        rpy_deg_roll = rpy[0] * 180 / math.pi
        rpy_deg_pitch = rpy[1] * 180 / math.pi
        rpy_deg_yaw = rpy[2] * 180 / math.pi

        x_orientation_3.append([rpy_deg_roll, rpy_deg_pitch, rpy_deg_yaw])

    y_np = np.asarray(y)

    position_xyz_1 = np.multiply(np.asarray(x_position_1), 100)
    position_xyz_2 = np.multiply(np.asarray(x_position_2), 100)
    position_xyz_3 = np.multiply(np.asarray(x_position_3), 100)

    rotation_rpy_1 = np.asarray(x_orientation_1)
    rotation_rpy_2 = np.asarray(x_orientation_2)
    rotation_rpy_3 = np.asarray(x_orientation_3)

    fig = plt.figure("Position")
    fig_orientation = plt.figure("Orientation")
    fig_3D = plt.figure("3D")

    gs = fig.add_gridspec(3, 3, hspace=0.05, wspace=0.2)
    gs_orientation = fig_orientation.add_gridspec(3, 3, hspace=0.05, wspace=0.2)

    ax_3D = fig_3D.add_subplot(projection='3d')
    axs = gs.subplots(sharex=True)
    axs_orientation = gs_orientation.subplots(sharex=True)

    ## 3D projection
    ax_3D.scatter(position_xyz_1[:, 0], position_xyz_1[:, 1], position_xyz_1[:, 2])
    ax_3D.scatter(position_xyz_2[:, 0], position_xyz_2[:, 1], position_xyz_2[:, 2])
    ax_3D.scatter(position_xyz_3[:, 0], position_xyz_3[:, 1], position_xyz_3[:, 2])
    ax_3D.set_xlabel('X')
    ax_3D.set_ylabel('Y')
    ax_3D.set_zlabel('Z')

    ## robot subplot position
    axs[0, 0].plot(y_np, position_xyz_1[:, 0], 'r', label="position_x")
    axs[0, 0].set_title("tracker 0")
    axs[0, 0].set_ylabel('Position (cm)')
    axs[1, 0].plot(y_np, position_xyz_1[:, 1], 'g', label="position_y")
    axs[1, 0].set_ylabel('Position (cm)')
    axs[2, 0].plot(y_np, position_xyz_1[:, 2], 'b', label="position_z")
    axs[2, 0].set_xlabel('timestamps (s)')
    axs[2, 0].set_ylabel('Position (cm)')
    for plot_axis in axs[:, 0]:
        plot_axis.legend()
        plot_axis.grid()

    ## robot subplot orientation
    axs_orientation[0, 0].plot(y_np, rotation_rpy_1[:, 0], 'r', label="rotation_roll")
    axs_orientation[0, 0].set_title("tracker 0")
    axs_orientation[0, 0].set_ylabel('Orientation (degrees)')
    axs_orientation[1, 0].plot(y_np, rotation_rpy_1[:, 1], 'g', label="rotation_pitch")
    axs_orientation[1, 0].set_ylabel('Orientation (degrees)')
    axs_orientation[2, 0].plot(y_np, rotation_rpy_1[:, 2], 'b', label="rotation_yaw")
    axs_orientation[2, 0].set_ylabel('Orientation (degrees)')
    axs_orientation[2, 0].set_xlabel('timestamps (s)')
    for plot_axis in axs_orientation[:, 0]:
        plot_axis.legend()
        plot_axis.grid()

    ## tracker subplot position
    axs[0, 1].plot(y_np, position_xyz_2[:, 0], 'r', label="position_x")
    axs[0, 1].set_title("tracker 1")
    axs[1, 1].plot(y_np, position_xyz_2[:, 1], 'g', label="position_y")
    axs[2, 1].plot(y_np, position_xyz_2[:, 2], 'b', label="position_z")
    axs[2, 1].set_xlabel('timestamps (s)')
    for plot_axis in axs[:, 1]:
        plot_axis.legend()
        plot_axis.grid()

    ## tracker subplot orientation
    axs_orientation[0, 1].plot(y_np, rotation_rpy_2[:, 0], 'r', label="rotation_roll")
    axs_orientation[0, 1].set_title("tracker 1")
    axs_orientation[1, 1].plot(y_np, rotation_rpy_2[:, 1], 'g', label="rotation_pitch")
    axs_orientation[2, 1].plot(y_np, rotation_rpy_2[:, 2], 'b', label="rotation_yaw")
    axs_orientation[2, 1].set_xlabel('timestamps (s)')
    for plot_axis in axs_orientation[:, 1]:
        plot_axis.legend()
        plot_axis.grid()

    ## Difference subplot orientation
    axs[0, 2].plot(y_np, position_xyz_3[:, 0], 'r', label="position_x")
    axs[0, 2].set_title("tracker 2")
    axs[1, 2].plot(y_np, position_xyz_3[:, 1], 'g', label="position_y")
    axs[2, 2].plot(y_np, position_xyz_3[:, 2], 'b', label="position_z")
    axs[2, 2].set_xlabel('timestamps (s)')
    for plot_axis in axs[:, 2]:
        plot_axis.legend()
        plot_axis.grid()

    axs_orientation[0, 2].plot(y_np, rotation_rpy_3[:, 0], 'r', label="roll")
    axs_orientation[0, 2].set_title("tracker 2")
    axs_orientation[1, 2].plot(y_np, rotation_rpy_3[:, 1], 'g', label="pitch")
    axs_orientation[2, 2].plot(y_np, rotation_rpy_3[:, 2], 'b', label="yaw")
    axs_orientation[2, 2].set_xlabel('timestamps (s)')
    for plot_axis in axs_orientation[:, 2]:
        plot_axis.legend()
        plot_axis.grid()

    plt.show()


def plot_n_trackers_3D(serial_numbers):
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
            position_xyz = np.multiply(np.asarray(x_position), 100)
            ## 3D projection
            ax_3D.scatter(position_xyz[:, 0], position_xyz[:, 1], position_xyz[:, 2])
        else:
            print("ERROR, serial number " + serials[i] + " is None")

    ax_3D.set_xlabel('X')
    ax_3D.set_ylabel('Y')
    ax_3D.set_zlabel('Z')
    plt.show()


def plot_one_tracker(data):
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

    axs[0, 0].plot(y_np, position_xyz[:, 0], 'r', label="position_x")
    axs[0, 0].set_title("tracker position x")

    axs[1, 0].plot(y_np, position_xyz[:, 1], 'g', label="position_y")
    axs[1, 0].set_title("tracker position y")

    axs[2, 0].plot(y_np, position_xyz[:, 2], 'b', label="position_z")
    axs[2, 0].set_title("tracker position z")

    axs[0, 1].plot(y_np, rotation_rpy[:, 0], 'r', label="rotation_roll")
    axs[0, 1].set_title("tracker rotation roll")

    axs[1, 1].plot(y_np, rotation_rpy[:, 1], 'g', label="rotation_pitch")
    axs[1, 1].set_title("tracker rotation pitch")

    axs[2, 1].plot(y_np, rotation_rpy[:, 2], 'b', label="rotation_yaw")
    axs[2, 1].set_title("tracker rotation yaw")

    for plot_axis in axs[:, 0]:
        plot_axis.legend()
        # plot_axis.xlabel("Timestamps")
        plot_axis.grid()
        plot_axis.label_outer()

    fig.tight_layout()
    # plt.ylabel("Distance (m)")

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
    plot_one_tracker(data)
elif len(serial_numbers) == 2:
    if not args.invert:
        data_tracker_1 = vive_log.get_data(serials[0])
        data_tracker_2 = vive_log.get_data(serials[1])
    else:
        data_tracker_1 = vive_log.get_data(serials[1])
        data_tracker_2 = vive_log.get_data(serials[0])
elif len(serial_numbers) >= 3:
    data_tracker_1 = vive_log.get_data(serials[0])
    data_tracker_2 = vive_log.get_data(serials[1])
    data_tracker_3 = vive_log.get_data(serials[2])
    plot_three_trackers(data_tracker_1, data_tracker_2, data_tracker_3)
elif len(serial_numbers) >= 3:
    plot_n_trackers_3D(serial_numbers)
else:
    raise ValueError("ERROR, no tracker to plot !")

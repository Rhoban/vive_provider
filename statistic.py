import argparse
import errno
import os

import numpy as np
from transforms3d import euler
import matplotlib.pyplot as plt
import math
from vive_logs import ViveLog


def plot_two_trackers(data_1, data_2):
    y_start = data_1[0][0]
    y = []
    x_position_1 = []
    x_orientation_1 = []
    #x_velocity = []
    
    x_position_2 = []
    x_orientation_2 = []

    for line in data_1:
        y.append((line[0]-y_start)/1000000) # convert to second
        
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
      
         
    y_np = np.asarray(y)

    position_xyz_1 = np.asarray(x_position_1)
    position_xyz_2 = np.asarray(x_position_2)
    position_xyz_diff = np.subtract(position_xyz_1, position_xyz_2)

    rotation_rpy_1 = np.asarray(x_orientation_1)
    rotation_rpy_2 = np.asarray(x_orientation_2)
    rotation_rpy_diff = np.subtract(x_orientation_1, x_orientation_2)


    fig = plt.figure()
    gs = fig.add_gridspec(6,3, hspace=0)
    axs = gs.subplots(sharex=True)

    axs[0,0].plot(y_np, position_xyz_1[:,0], 'r',label ="position_x")
    axs[0,0].set_title("tracker robot")
    axs[1,0].plot(y_np, position_xyz_1[:,1], 'g', label ="position_y")
    axs[2,0].plot(y_np, position_xyz_1[:,2], 'b', label ="position_z")
    axs[3,0].plot(y_np, rotation_rpy_1[:,0], 'r', label ="rotation_roll")
    axs[4,0].plot(y_np, rotation_rpy_1[:,1], 'g', label ="rotation_pitch")
    axs[5,0].plot(y_np, rotation_rpy_1[:,2], 'b', label ="rotation_yaw")
    for plot_axis in axs[:,0]:
        plot_axis.legend()
        # plot_axis.xlabel("Timestamps")
        plot_axis.grid()
        
    axs[0,1].plot(y_np, position_xyz_2[:,0], 'r',label ="position_x")
    axs[0,1].set_title("tracker ground")
    axs[1,1].plot(y_np, position_xyz_2[:,1], 'g', label ="position_y")
    axs[2,1].plot(y_np, position_xyz_2[:,2], 'b', label ="position_z")
    axs[3,1].plot(y_np, rotation_rpy_2[:,0], 'r', label ="rotation_roll")
    axs[4,1].plot(y_np, rotation_rpy_2[:,1], 'g', label ="rotation_pitch")
    axs[5,1].plot(y_np, rotation_rpy_2[:,2], 'b', label ="rotation_yaw")
    for plot_axis in axs[:,1]:
        plot_axis.legend()
        # plot_axis.xlabel("Timestamps")
        plot_axis.grid()
        
    axs[0,2].plot(y_np, position_xyz_diff[:,0], 'r',label ="diff_x")
    axs[0,2].set_title("robot - ground x")
    axs[1,2].plot(y_np, position_xyz_diff[:,1], 'g', label ="diff_y")
    axs[2,2].plot(y_np, position_xyz_diff[:,2], 'b', label ="diffz")
    axs[3,2].plot(y_np, rotation_rpy_diff[:,0], 'r', label ="diff_roll")
    axs[4,2].plot(y_np, rotation_rpy_diff[:,1], 'g', label ="diff_pitch")
    axs[5,2].plot(y_np, rotation_rpy_diff[:,2], 'b', label ="diff_yaw")
    for plot_axis in axs[:,2]:
        plot_axis.legend()
        # plot_axis.xlabel("Timestamps")
        plot_axis.grid()
        
    fig.tight_layout()
    # plt.ylabel("Distance (m)")

    plt.show()
    
def plot_one_tracker(data):
    y_start = data[0][0]
    y = []
    x_position = []
    x_orientation = []
    #x_velocity = []

    for line in data:
        y.append((line[0]-y_start)/1000000) # convert to second
        
        x_position.append([line[1].x, line[1].y, line[1].z])
        
        rpy = euler.quat2euler([line[2].qw, line[2].qx, line[2].qy, line[2].qz])
        rpy_deg_roll = rpy[0] * 180 / math.pi
        rpy_deg_pitch = rpy[1] * 180 / math.pi
        rpy_deg_yaw = rpy[2] * 180 / math.pi
        
        x_orientation.append([rpy_deg_roll, rpy_deg_pitch, rpy_deg_yaw])
        
        
        #x_velocity.append(line[3])
        
    y_np = np.asarray(y)

    position_xyz = np.asarray(x_position)

    rotation_rpy = np.asarray(x_orientation)
    #x_velocity_np = np.asarray(x_velocity)

    fig, axs = plt.subplots(3, 2, sharex=True)

    axs[0,0].plot(y_np, position_xyz[:,0], 'r',label ="position_x")
    axs[0,0].set_title("tracker position x")

    axs[1,0].plot(y_np, position_xyz[:,1], 'g', label ="position_y")
    axs[1,0].set_title("tracker position y")

    axs[2,0].plot(y_np, position_xyz[:,2], 'b', label ="position_z")
    axs[2,0].set_title("tracker position z")

    axs[0,1].plot(y_np, rotation_rpy[:,0], 'r', label ="rotation_roll")
    axs[0,1].set_title("tracker rotation roll")

    axs[1,1].plot(y_np, rotation_rpy[:,1], 'g', label ="rotation_pitch")
    axs[1,1].set_title("tracker rotation pitch")

    axs[2,1].plot(y_np, rotation_rpy[:,2], 'b', label ="rotation_yaw")
    axs[2,1].set_title("tracker rotation yaw")

    for plot_axis in axs[:,0]:
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

serials = list(serial_numbers)

if len(serial_numbers) == 1:
    data = vive_log.get_data(serials[0])
    plot_one_tracker(data)
elif len(serial_numbers) == 2:
    if(args.invert):
        data_tracker_1 = vive_log.get_data(serials[0])
        data_tracker_2 = vive_log.get_data(serials[1])
    else:
        data_tracker_1 = vive_log.get_data(serials[1])
        data_tracker_2 = vive_log.get_data(serials[0])
        
    plot_two_trackers(data_tracker_1, data_tracker_2)
else :
    raise ValueError("ERROR, no tracker to plot !")
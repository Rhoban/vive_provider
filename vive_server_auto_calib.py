#!/usr/bin/env python
import json
from gc import collect

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from vive_pb2 import *
from transforms3d import euler
from scipy.spatial.transform import Rotation as R

import argparse
from datetime import datetime
import socket
import time
from vive_utils import *
from vive_provider import *

parser = argparse.ArgumentParser()
parser.add_argument("--broadcast", "-b", type=str, default="192.168.0.255")
parser.add_argument("--max_freq", "-f", type=int, default=250)
parser.add_argument("--points", "-p", type=str, default=FIELD_POINTS_TRACKERS_FILENAME)
args = parser.parse_args()

collection = GlobalCollection()
corrected_collection = GlobalCollection()

try:
    vp = ViveProvider()

    # XXX: This should be parametrized with args
    address = args.broadcast
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(0.2)
    server.bind(("", VIVE_SERVER_PORT))

    pb_msg: GlobalMsg = GlobalMsg()
    last_broadcast: float = time.time()
    sequence: int = 0
    sequence_calibrate: int = 0
    period: float = 1.0 / args.max_freq
    last: float = 0

    list_field_position = []
    list_position_trackers = []
    list_to_calibrate = []

    f = open(FIELD_POINTS_TRACKERS_FILENAME, "r")
    test_positions = json.load(f)
    f.close()

    while True:
        elapsed = time.time() - last
        if elapsed < period:
            time.sleep(period - elapsed)
        last = time.time()

        # Get world data from all trackers (balise and tracker robot)
        T_world_trackers = vp.get_tracker_infos_without_calibration(raw=True, tracker_calibration_name=test_positions)
        pb_msg.Clear()
        pb_msg = tracker_infos_to_GlobalMsg(T_world_trackers)
        pb_msg.seq = sequence
        sequence += 1
        collection.messages.extend([pb_msg])

        if len(collection.tagged_positions) == 0:
            tagged_positions_to_message(T_world_trackers, collection)

        # Only sending network messages at ~100Hz
        if time.time() - last_broadcast > 0.01:
            tagged_positions_to_message(T_world_trackers, pb_msg)
            last_broadcast = time.time()

            references = {}

            # Calibrate all trackers
            T_field_trackers = vive_utils.calib_position(test_positions, T_world_trackers)

            print("---")
            print("* Tracking %d devices (%d detections made)" % (
                len(T_world_trackers["trackers"]), len(collection.messages)))

            pb_msg.Clear()
            pb_msg = tracker_infos_to_GlobalMsg(T_field_trackers)
            pb_msg.seq = sequence_calibrate
            sequence_calibrate += 1
            corrected_collection.messages.extend([pb_msg])

            if address is not None:
                bytes_sent = server.sendto(pb_msg.SerializeToString(), (address, VIVE_SERVER_PORT))

            # Clear each data
            pb_msg.ClearField("tagged_positions")
            list_field_position.clear()
            list_position_trackers.clear()
            list_to_calibrate.clear()

except KeyboardInterrupt:
    # Writing logs to binary file
    fname = datetime.now().strftime("%Y_%m_%d-%Hh%Mm%Ss") + "_vive.bin"
    print("Interrupted, saving the collection to %s ..." % fname)
    f = open("logs/" + fname, "wb")
    s = collection.SerializeToString()
    f.write(s)
    f.close()

    fname = datetime.now().strftime("%Y_%m_%d-%Hh%Mm%Ss") + "_vive_auto_calib.bin"
    print("Interrupted, saving the collection to %s ..." % fname)
    f = open("logs/" + fname, "wb")
    s = corrected_collection.SerializeToString()
    f.write(s)
    f.close()

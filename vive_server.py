#!/usr/bin/env python

from gc import collect
from vive_pb2 import *
from transforms3d import euler

import argparse
from datetime import datetime
import socket
import time
from vive_utils import *
from vive_provider import *

parser = argparse.ArgumentParser()
parser.add_argument("--broadcast", "-b", type=str, default="192.168.0.255")
parser.add_argument("--max_freq", "-f", type=int, default=250)
args = parser.parse_args()

collection = GlobalCollection()

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
    period: float = 1.0 / args.max_freq
    last: float = 0

    while True:
        # Limitting frequency
        elapsed = time.time() - last
        if elapsed < period:
            time.sleep(period - elapsed)
        last = time.time()

        # Collecting messages at maximum speed
        trackers = vp.get_tracker_infos()
        pb_msg.Clear()
        pb_msg = tracker_infos_to_GlobalMsg(trackers)
        pb_msg.seq = sequence
        sequence += 1
        collection.messages.extend([pb_msg])

        if len(collection.tagged_positions) == 0:
            tagged_positions_to_message(trackers, collection)

        # Only sending network messages at ~100Hz
        if time.time() - last_broadcast > 0.01:
            tagged_positions_to_message(trackers, pb_msg)
            last_broadcast = time.time()

            # Output debug infos
            print("---")
            print("* Tracking %d devices (%d detections made)" % (len(trackers["trackers"]), len(collection.messages)))
            for id in trackers["trackers"]:
                p = trackers["trackers"][id]["position"]
                rpy = euler.quat2euler(trackers["trackers"][id]["orientation"])
                print("- %s (%s)" % (id, trackers["trackers"][id]["device_type"]))
                print("  - x: %g, y: %g, z: %g" % (p[0], p[1], p[2]))
                print("  - roll: %g, pitch: %f, yaw: %g" % tuple(rpy))
            print()

            if address is not None:
                bytes_sent = server.sendto(pb_msg.SerializeToString(), (address, VIVE_SERVER_PORT))

            pb_msg.ClearField("tagged_positions")

except KeyboardInterrupt:
    # Writing logs to binary file
    fname = datetime.now().strftime("%Y_%m_%d-%Hh%Mm%Ss") + "_vive.bin"
    print("Interrupted, saving the collection to %s ..." % fname)
    f = open("logs/" + fname, "wb")
    s = collection.SerializeToString()
    f.write(s)
    f.close()

#!/usr/bin/env python

from vive_pb2 import *
from transforms3d import euler

from datetime import datetime
import socket
import time
from utils import *
from vive_provider import *

collection = GlobalCollection()

try:
    vp = ViveProvider()

    # XXX: This should be parametrized with args
    # addr = None
    # addr = '<broadcast>'
    address = "192.168.0.255"
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(0.2)
    server.bind(("", VIVE_SERVER_PORT))

    pb_msg = GlobalMsg()
    last_broadcast: float = time.time()
    sequence: int = 0

    while True:
        # Collecting messages at maximum speed
        trackers = vp.getTrackersInfos()
        pb_msg.Clear()
        pb_msg = tracker_infos_to_GlobalMsg(trackers, sequence)
        pb_msg.seq = sequence
        sequence += 1
        collection.messages.extend([pb_msg])

        # Only sending network messages at ~100Hz
        if time.time() - last_broadcast > 0.01:
            last_broadcast = time.time()

            # Output debug infos
            print("---")
            print("* Tracking %d devices (%d detections made)" % (len(trackers["trackers"]), len(collection)))
            for id in trackers["trackers"]:
                p = trackers["trackers"][id]["pose"]
                rpy = euler.mat2euler(trackers["trackers"][id]["pose_matrix"]) * 180.0 / math.pi
                print("- %s (%s)" % (id, trackers["trackers"][id]["device_type"]))
                print("  - x: %g, y: %g, z: %g" % (p[0], p[1], p[2]))
                print("  - roll: %g, pitch: %f, yaw: %g" % tuple(rpy))
            print()

            if address is not None:
                bytes_sent = server.sendto(pb_msg.SerializeToString(), (address, 37020))

except KeyboardInterrupt:
    # Writing logs to binary file
    fname = datetime.now().strftime("%Y_%m_%d-%Hh%Mm%Ss") + "_vive.bin"
    print("Interrupted, saving the collection to %s ..." % fname)
    f = open("logs/" + fname, "wb")
    s = collection.SerializeToString()
    f.write(s)
    f.close()

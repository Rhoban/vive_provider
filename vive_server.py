#!/usr/bin/env python

from vive_pb2 import *

from datetime import datetime
import socket
import time
from utils import *
import sys
from vive_provider import *

collection = GlobalCollection()

try:
    vp = Vive_provider()

    addr = None
    # addr = '<broadcast>'
    # addr = '10.0.0.255'
    server = socket.socket(
        socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(0.2)
    server.bind(("", 44444))

    pb_msg = GlobalMsg()
    last = time.time()
    i = 0
    while True:
        # Collecting messages at maximum speed
        trackers = vp.getTrackersInfos()
        pb_msg.Clear()
        pb_msg = trackersInfos_to_GlobalMsg(trackers, i)
        collection.messages.extend([pb_msg])
        i += 1

        # Can be removed to have full logs, to test
        time.sleep(0.01);

        # Only sending network messages at ~100Hz
        if time.time()-last > 0.01:
            last = time.time()

            # Converting message to bytes for network
            trackersInfos = pb_msg.SerializeToString()

            # Output debug infos
            print('---')
            print('* Tracking %d devices' % len(trackers['trackers']))
            for id in trackers['trackers']:
                p = trackers['trackers'][id]['pose']
                rpy = np.array(convert_to_euler(
                    trackers['trackers'][id]['pose_matrix']))*180.0/math.pi
                print('- %s (%s)' %
                      (id, trackers['trackers'][id]['device_type']))
                print('  - x: %g, y: %g, z: %g' % (p[0], p[1], p[2]))
                print('  - roll: %g, pitch: %f, yaw: %g' % tuple(rpy))
            print()

            if addr is not None:
                bytes_sent = server.sendto(trackersInfos, (addr, 37020))

except KeyboardInterrupt:
    fname = datetime.now().strftime('%Y_%m_%d-%Hh%Mm%Ss')+'_vive.bin'
    print('Interrupted, saving the collection to %s ...' % fname)
    f = open('logs/'+fname, 'wb')
    s = collection.SerializeToString()
    f.write(s)
    f.close()

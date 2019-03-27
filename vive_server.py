#!/usr/bin/env python

from vive_pb2 import *

import socket
import time
from utils import *
import sys
from vive_provider import *

vp = Vive_provider()


server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
server.settimeout(0.2)
server.bind(("", 44444))

pb_msg = GlobalMsg()
i = 0
while True:
    trackers = vp.getTrackersInfos()
    pb_msg.Clear()
    pb_msg = trackersInfos_to_GlobalMsg(trackers, i)
    # pb_msg = trackersInfos_to_GlobalMsg(get_dummy_trackerInfos())
    
    # temporary, converting to bytes
    trackersInfos = pb_msg.SerializeToString()

    print('---')
    print('* Tracking %d devices' % len(trackers['trackers']))
    for id in trackers['trackers']:
        p = trackers['trackers'][id]['pose']
        print('- %s (%g, %g, %g)' % (id, p[0], p[1], p[2]))
    print()
    
    bytes_sent = server.sendto(trackersInfos, ('<broadcast>', 37020))
    # print("bytes_sent: {}".format(bytes_sent))
    time.sleep(0.01)
    i+=1

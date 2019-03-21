#!/usr/bin/python3

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
while True:

    pb_msg.Clear()
    pb_msg = trackersInfos_to_GlobalMsg(vp.getTrackersInfos())
    # pb_msg = trackersInfos_to_GlobalMsg(get_dummy_trackerInfos())
    
    # temporary, converting to bytes
    trackersInfos = pb_msg.SerializeToString()
    
    bytes_sent = server.sendto(trackersInfos, ('<broadcast>', 37020))
    print("bytes_sent: {}".format(bytes_sent))
    time.sleep(0.1)

#!/usr/bin/python3

from vive_pb2 import *

import socket
import time


server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
server.settimeout(0.2)
server.bind(("", 44444))

pb_msg = GlobalMsg()
while True:

    pb_msg.Clear()

    pb_msg.vive_timestamp = int(time.perf_counter()* 10 ** 6)
    pb_msg.time_since_epoch = int(time.time()*1000000)

    for i in range(0,2):
        pb_msg.trackers.add()
        pb_msg.trackers[i].tracker_idx = i+1
    
    # temporary, converting to bytes
    trackersInfos = pb_msg.SerializeToString()
    
    bytes_sent = server.sendto(trackersInfos, ('<broadcast>', 37020))
    print("bytes_sent: {}".format(bytes_sent))
    time.sleep(0.5)

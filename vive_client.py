#!/usr/bin/python3

from vive_pb2 import *
import socket

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
client.bind(("", 37020))

while True:
    pb_msg = GlobalMsg()
    data, addr = client.recvfrom(1024)
    pb_msg.ParseFromString(data)
    
    nb_trackers = len(pb_msg.trackers)
    print("Received a message: source:{}, len:{}".format(addr,len(data)))
    print("\ttimestamp: {}".format(pb_msg.vive_timestamp))
    print("\ttime_since_epoch: {}".format(pb_msg.time_since_epoch))
    print("\ttrackers: {}.format(nb_trackers)")
    for i in range(nb_trackers):
        print("\t\tidx: {}".format(pb_msg.trackers[i].tracker_idx))
        print("\t\tidx: {}".format(pb_msg.trackers[i].tracker_idx))
    

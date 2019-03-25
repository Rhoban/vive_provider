#!/usr/bin/env python

from vive_pb2 import *
import socket
from utils import *
from vive_provider import *

vp = Vive_provider(clientMode=True)

while True:
    
    trackers = vp.getTrackersInfos()
    
    # for t in trackers:
    #     print(t)
    

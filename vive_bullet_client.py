#!/usr/bin/env python
from vive_bullet import *

vive = ViveProvider(clientMode=True)
viewer = BulletViewer(vive)
viewer.physics = False
viewer.execute()

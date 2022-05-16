#!/usr/bin/env python
from vive_bullet import *

vive = ViveProvider(client_mode=True)
viewer = BulletViewer(vive)
viewer.physics = False
viewer.execute()

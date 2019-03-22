#!/usr/bin/python3

from vive_provider import *

vp = Vive_provider()

halfField = False
if(len(sys.argv)>1):
    halfField = True    
    

def displayFieldPositions(half):

    if not half:
        print("       0-------------------------+")
        print("       |                         |")
        print("       |            +            |")
        print("tables |          +   +          |  mur")
        print("       |            +            |")
        print("       |                         |")
        print("       1-------------------------2")
    else:
        print("       0-------------------------+")
        print("       |                         |")
        print("       |            +            |")
        print("tables |          +   +          |  mur")
        print("       |            +            |")
        print("       |                         |")
        print("       1------------2------------+")
        
data = {}
positions = {}

displayFieldPositions(halfField)

controllersInfos = vp.getControllersInfos(raw=True)
if len(controllersInfos) != 1:
    print('ERROR: Calibration should have exactly one controller (found %d)' % len(controllersInfos))
    exit()

for i in range(0, 3):
    print('Place tracker on position '+str(i)+', then press the button ')

    # Waiting for button to be released
    while vp.getControllersInfos(raw=True)[0]['buttonPressed']:
        time.sleep(0.01)
    # Waitng for button to be pressed
    while not vp.getControllersInfos(raw=True)[0]['buttonPressed']:
        time.sleep(0.01)

    pose = vp.getControllersInfos(raw=True)[0]['pose']

    print(pose[0], pose[1], pose[2])
    positions[i] = [pose[0], pose[1], pose[2]]

data["halfField"] = halfField
data["positions"] = positions
    
calibFile = open("calibFile.txt", "w")
calibFile.write(str(data))
calibFile.close()

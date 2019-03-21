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

for i in range(0, 3):
    input('Place tracker on position '+str(i)+', then press enter: ')
    pose = vp.getTrackersInfos(raw=True)
    print(pose[0], pose[1], pose[2])
    positions[i] = [pose[0], pose[1], pose[2]]

data["halfField"] = halfField
data["positions"] = positions
    
calibFile = open("calibFile.txt", "w")
calibFile.write(str(data))
calibFile.close()

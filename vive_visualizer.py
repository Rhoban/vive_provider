#!/usr/bin/python3

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from vive_provider import *
import sys
import socket
from vive_pb2 import *
from utils import * 


clientMode = False
if (len(sys.argv)>1):
    clientMode = True
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client.bind(("", 37020))
else:
    vp = Vive_provider()
    

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(700,700)
    glutCreateWindow(b"Trackers visualizer")

    glClearColor(1.,1.,1.,1.)
    glShadeModel(GL_SMOOTH)
    
    # glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    
    lightZeroPosition = [10.,4.,10.,1.]
    lightZeroColor = [0.8,1.0,0.8,1.0] #green tinged
    
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
    
    glEnable(GL_LIGHT0)
    
    glutDisplayFunc(display)
    
    glMatrixMode(GL_PROJECTION)
    
    gluPerspective(40., 1. ,1. ,40.)
    
    glMatrixMode(GL_MODELVIEW)
    
    gluLookAt(10, 10, 10,
              0, 0, 0,
              0, 1, 0)
    
    glPushMatrix()
    glutMainLoop()
    
    return

def displayAxis(xRot, yRot, zRot):
    glPushMatrix()
    quad = gluNewQuadric()
    
    color = [1., 0., 0., 1.]
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    gluCylinder(quad, 0.05, 0.05, 3, 30, 30);

    color = [0., 1., 0., 1.]    
    glRotatef(90, 0, 1, 0);
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    gluCylinder(quad, 0.05, 0.05, 3, 30, 30);

    color = [0., 0., 1., 1.]
    glRotatef(90, 0, 0, 1);
    glRotatef(90, 0, 1, 0);
    
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    gluCylinder(quad, 0.05, 0.05, 3, 30, 30);

    glPopMatrix()


def displayTracker(pose, color):
    glPushMatrix()
    glTranslatef(pose[0]*2, pose[1]*2, pose[2]*2)
    
    qw = pose[3]
    qx = pose[4]
    qy = pose[5]
    qz = pose[6]
            
    angle = 2 * (math.acos(qw)*180)/math.pi
    x = qx / (math.sqrt(1-qw*qw) + 0.000001)
    y = qy / (math.sqrt(1-qw*qw) + 0.000001)
    z = qz / (math.sqrt(1-qw*qw) + 0.000001)
            
    glRotatef(angle, x, y, z);
    
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)

    thickness = 0.3
    radius = 0.5
    quad = gluNewQuadric()
    glRotatef(180, 1, 0, 0)
    gluDisk(quad, 0, radius, 20, 1)
    glRotatef(-180, 1, 0, 0)
    gluCylinder(quad, radius, radius, thickness, 20, 10);
    glTranslatef(0, 0, thickness)
    gluDisk(quad, 0, radius, 20, 1)

    glMaterialfv(GL_FRONT,GL_DIFFUSE,[1, 1, 1, 1])
    cubeSize = 0.2
    glTranslatef(radius, 0 , -0.4)
    glutSolidCube(cubeSize)
    glTranslatef(-radius, 0 , 0.4)

    
    
    glPopMatrix()
    glutSwapBuffers()
    glutPostRedisplay()    
            

def display():
    
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    displayAxis(0, 0, 0)
    
    if(clientMode):
        pb_msg = GlobalMsg()
        data, addr = client.recvfrom(1024)
        trackersInfo = GlobalMsg_to_trackersInfos(data)
    else:
        trackersInfo = vp.getTrackersInfos()    
    
    for t in range(0, 1):
    # for t in vp.trackers:
        if(clientMode):
            tracker = trackersInfo["tracker_"+str(t)]
        else:
            tracker = trackersInfo["tracker_"+str(t+1)]
        if(tracker['time_since_last_tracked'] == 0):
            pose = tracker['pose']
            displayTracker(pose, [0.0,0.,1.,1.])            
        else:
            print(tracker['time_since_last_tracked'])
            print("tracker not visible")
        

    
main()

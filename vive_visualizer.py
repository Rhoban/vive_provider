#!/usr/bin/python3

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from vive_provider import *
import sys
import socket
from vive_pb2 import *
from utils import * 
from objloader import *


texture = load_texture('assets/black.png')
obj = ObjLoader('assets/HTC_Vive_Tracker.obj')


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
    
    gluPerspective(60., 1. ,1. ,40.)
    
    glMatrixMode(GL_MODELVIEW)
    
    gluLookAt(2, 2, 2,
              0, 0, 0,
              0, 0, 1)
    
    glPushMatrix()
    glutMainLoop()
    
    return

def displayAxis(center):
    glPushMatrix()
    
    glDisable(GL_LIGHTING)
    glLineWidth(6);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1., 0, 0);
    glEnd();

    glLineWidth(6);
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0., 1, 0);
    glEnd();

    glLineWidth(6);
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0., 0, 1);
    glEnd();

    glEnable(GL_LIGHTING)

    glPopMatrix()


def displayTracker(pose, color):
    glPushMatrix()

    glMultMatrixd(np.mat(pose).T)
    
    glMaterialfv(GL_FRONT,GL_DIFFUSE, color)
    glScale(0.005, 0.005, 0.005) # TODO determine scale
    glRotatef(-90, 1, 0, 0)
    obj.render_scene()

    
    
    glPopMatrix()
    glutSwapBuffers()
    glutPostRedisplay()    
            

def display():
    
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    if(clientMode):
        pb_msg = GlobalMsg()
        data, addr = client.recvfrom(1024)
        trackersInfo, nbTrackers = GlobalMsg_to_trackersInfos(data)
    else:
        trackersInfo = vp.getTrackersInfos()
        nbTrackers = len(vp.trackers)

    displayAxis([0, 0, 0])
    
    # glMatrixMode(GL_MODELVIEW)

    
    # for t in range(0, len(vp.trackers)):
    
    for t in range(1, nbTrackers+1):
        if(clientMode):
            tracker = trackersInfo["tracker_"+str(int(t)-1)]
        else:
            tracker = trackersInfo["tracker_"+str(int(t))]
        if(tracker['time_since_last_tracked'] == 0):
            pose = tracker['pose_matrix']
            ppose = tracker['pose']

            # make the camera follow the object
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            gluLookAt(2, 2, 2,
              tracker['pose'][0], tracker['pose'][1], tracker['pose'][2],
              0, 0, 1)

            
            displayTracker(pose, [0.0,0.,1.,1.])
        else:
            print(tracker['time_since_last_tracked'])
            print("tracker not visible")
        

    
main()

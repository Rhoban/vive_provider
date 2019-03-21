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
    
    # m = np.matrix([list(pose[0]), list(pose[1]), list(pose[2])])
    # m = np.vstack((m, [0, 0, 0, 1]))

    glMultMatrixd(pose.T)
    

    # glMaterialfv(GL_FRONT,GL_DIFFUSE,[1, 1, 1, 1])
    # glutSolidCube(0.2)
    # sys.exit()

    
    # glTranslatef(pose[0], pose[1], pose[2])
    
    # qw = pose[3]
    # qx = pose[4]
    # qy = pose[5]
    # qz = pose[6]
            
    # angle = 2 * (math.acos(qw)*180)/math.pi
    # x = qx / (math.sqrt(1-qw*qw) + 0.000001)
    # y = qy / (math.sqrt(1-qw*qw) + 0.000001)
    # z = qz / (math.sqrt(1-qw*qw) + 0.000001)
    
    # glRotatef(angle, x, y, z)
    
    glMaterialfv(GL_FRONT,GL_DIFFUSE, color)
    glScale(0.005, 0.005, 0.005)
    # pushMatrix()
    glRotatef(-90, 1, 0, 0)
    obj.render_scene()
    # popMatrix()


    
    # glMaterialfv(GL_FRONT,GL_DIFFUSE,color)

    # thickness = 0.3
    # radius = 0.5
    # quad = gluNewQuadric()
    # # glRotatef(180, 1, 0, 0)
    # gluDisk(quad, 0, radius, 20, 1)
    # # glRotatef(-180, 1, 0, 0)
    # gluCylinder(quad, radius, radius, thickness, 20, 10);
    # glTranslatef(0, 0, thickness)
    # gluDisk(quad, 0, radius, 20, 1)


    
    # glMaterialfv(GL_FRONT,GL_DIFFUSE,[1, 1, 1, 1])
    # cubeSize = 0.2
    # glTranslatef(radius, 0 , -0.4)
    # glutSolidCube(cubeSize)
    # glTranslatef(-radius, 0 , 0.4)

    
    
    glPopMatrix()
    glutSwapBuffers()
    glutPostRedisplay()    
            

def display():
    
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    
    if(clientMode):
        pb_msg = GlobalMsg()
        data, addr = client.recvfrom(1024)
        trackersInfo = GlobalMsg_to_trackersInfos(data)
    else:
        trackersInfo = vp.getTrackersInfos()    

    displayAxis([0, 0, 0])


    # fakeTrackerPos = [center[0], center[1], center[2], 0, 0, 0, 0]
    # displayTracker(fakeTrackerPos, [1., 0., 0., 1])
    # sys.exit()

    
    # glMatrixMode(GL_MODELVIEW)
    # gluLookAt(center[0], center[1], center[2],
    #           0, 0, 0,
    #           0, 1, 0)
    # glPushMatrix()
    
    # for t in range(0, len(vp.trackers)):
    for t in vp.trackers:
        if(clientMode):
            tracker = trackersInfo["tracker_"+str(int(t)-1)]
        else:
            tracker = trackersInfo["tracker_"+str(int(t))]
        if(tracker['time_since_last_tracked'] == 0):
            # pose = tracker['raw_pose']
            pose = tracker['pose_matrix']
            # pose = tracker['pose']
            displayTracker(pose, [0.0,0.,1.,1.])
            # print(abs(center[0] - pose[0]), abs(center[1] - pose[1]), abs(center[2] - pose[2]))
        else:
            print(tracker['time_since_last_tracked'])
            print("tracker not visible")
        

    
main()

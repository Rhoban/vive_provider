from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from vive_provider import *
import sys

vp = Vive_provider(1)
# vp.scanForTrackers()

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(700,700)
    glutCreateWindow(b"azeaze")

    glClearColor(1.,1.,1.,1.)
    glShadeModel(GL_SMOOTH)
    
    glEnable(GL_CULL_FACE)
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


def display():
    
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    
    displayAxis(0, 0, 0)

    glPushMatrix()
    color = [0.0,0.,1.,1.]

    trackerInfo = vp.getTrackerInfo(1)
    if(trackerInfo is not None):
        pose = trackerInfo['pose']
        
        glTranslatef(pose[0], pose[1], pose[2])
        
        qw = pose[3]
        qx = pose[4]
        qy = pose[5]
        qz = pose[6]
        
        angle = 2 * (math.acos(qw)*180)/math.pi
        x = qx / math.sqrt(1-qw*qw) + 0.000001
        y = qy / math.sqrt(1-qw*qw)
        z = qz / math.sqrt(1-qw*qw)
        
        glRotatef(angle, x, y, z);
    else:
        print("tracker not visible")
        
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    glutSolidCube(0.5)
    glPopMatrix()
    glutSwapBuffers()
    glutPostRedisplay()

    
main()

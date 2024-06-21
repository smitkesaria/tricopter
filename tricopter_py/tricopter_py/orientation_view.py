from __future__ import print_function
import time
import sys
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node

useSerial = True # set true for using serial for data transmission, false for wifi
useQuat = True   # set true for using quaternions, false for using y,p,r angles

class DisplayAngles(Node):

    def __init__(self):
        super().__init__('display_angles')

        self.subscription = self.create_subscription(
            Imu,
            '/tricopter/imu',
            self.imu_callback,
            1)
        self.subscription  # prevent unused variable warning
        video_flags = OPENGL | DOUBLEBUF
        pygame.init()
        screen = pygame.display.set_mode((640, 480), video_flags)
        pygame.display.set_caption("PyTeapot IMU orientation visualization")
        self.resizewin(640, 480)
        self.init()
        ticks = pygame.time.get_ticks()
        self.imu_msg=Imu()
        self.timer = self.create_timer(0.01, self.update_cube)


    def resizewin(self,width, height):
        """
        For resizing window
        """
        if height == 0:
            height = 1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0*width/height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def init(self):
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    def cleanSerialBegin(self):
        if(useQuat):
            try:
                line = ser.readline().decode('UTF-8').replace('\n', '')
                w = float(line.split('w')[1])
                nx = float(line.split('a')[1])
                ny = float(line.split('b')[1])
                nz = float(line.split('c')[1])
            except Exception:
                pass
        else:
            try:
                line = ser.readline().decode('UTF-8').replace('\n', '')
                yaw = float(line.split('y')[1])
                pitch = float(line.split('p')[1])
                roll = float(line.split('r')[1])
            except Exception:
                pass

    def draw(self,w, nx, ny, nz):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)

        self.drawText((-2.6, 1.8, 2), "PyTeapot", 18)
        self.drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
        self.drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

        if(useQuat):
            [yaw, pitch , roll] = self.quat_to_ypr([w, nx, ny, nz])
            # roll=-roll
            self.drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
            # glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
        # else:
            # yaw = nx
            # pitch = ny
            # roll = nz
            # self.drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
            glRotatef(-roll, 0.00, 0.00, 1.00)
            glRotatef(-pitch, 1.00, 0.00, 0.00)
            glRotatef(yaw, 0.00, 1.00, 0.00)

        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

    def drawText(self,position, textString, size):
        font = pygame.font.SysFont("Courier", size, True)
        textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
        textData = pygame.image.tostring(textSurface, "RGBA", True)
        glRasterPos3d(*position)
        glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

    def quat_to_ypr(self,q):
        yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
        pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
        roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
        pitch *= 180.0 / math.pi
        yaw   *= 180.0 / math.pi
        yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
        roll  *= 180.0 / math.pi
        return [yaw, pitch, roll]

    def imu_callback(self, msg):
        self.imu_msg=msg
        # print(self.imu_msg)

    def update_cube(self):
        self.draw(self.imu_msg.orientation.w, self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z)
        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)

    display_angles = DisplayAngles()

    rclpy.spin(display_angles)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    display_angles.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
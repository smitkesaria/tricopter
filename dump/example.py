from __future__ import print_function
import qwiic_icm20948
import time
import sys
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q, q2euler
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import threading


ACC_FSR = 4                     # Full scale Range of accelerometer in g
GYR_FSR = 4000                   # Full scale Range of gyroscope in dps
MAG_FSR = 9800                  # Full scale Range of accelerometer in uT

BIT_RESOLUTION = 65536          # Resolution (2^16)
N_GYR_CAL = 1000
q = acc2q(np.zeros([1,3]))

useSerial = True # set true for using serial for data transmission, false for wifi
useQuat = True   # set true for using quaternions, false for using y,p,r angles
ekf = EKF()


def resizewin(width, height):
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


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def cleanSerialBegin():
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


def read_data():
    if(useSerial):
        ser.reset_input_buffer()
        cleanSerialBegin()
        line = ser.readline().decode('UTF-8').replace('\n', '')
        print(line)
    else:
        # Waiting for data from udp port 5005
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        line = data.decode('UTF-8').replace('\n', '')
        print(line)
                
    if(useQuat):
        w = float(line.split('w')[1])
        nx = float(line.split('a')[1])
        ny = float(line.split('b')[1])
        nz = float(line.split('c')[1])
        return [w, nx, ny, nz]
    else:
        yaw = float(line.split('y')[1])
        pitch = float(line.split('p')[1])
        roll = float(line.split('r')[1])
        return [yaw, pitch, roll]


def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "PyTeapot", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    if(useQuat):
        [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
    else:
        yaw = nx
        pitch = ny
        roll = nz
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(-roll, 0.00, 0.00, 1.00)
        glRotatef(pitch, 1.00, 0.00, 0.00)
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


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]

def run_ekf():
    global q
    time_now = 0
    last_read = 0
    acc_data = np.zeros([1,3])
    gyr_data = np.zeros([1,3])
    mag_data = np.zeros([1,3])
    gyr_sample = np.zeros([N_GYR_CAL,3])
    ekf.a_noise = 0.001
    ekf.g_noise = 0.1
    q = acc2q(acc_data)       # First sample of tri-axial accelerometer
    draw(q[0], q[1], q[2], q[3])
    pygame.display.flip()
    print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    IMU.begin()
    IMU.enableDlpfAccel(True)
    IMU.enableDlpfGyro(True)
    IMU.setFullScaleRangeGyro(0x03)

    print("Calibrating Gyro")
    for i in range(N_GYR_CAL):
        if IMU.dataReady():
            IMU.getAgmt()
            gyr_sample[i-1] = np.array([float(IMU.gxRaw),float(IMU.gyRaw),float(IMU.gzRaw)]) * GYR_FSR/BIT_RESOLUTION
            # print(gyr_sample[i-1])
    print("Gyro calibarion done with bias as ")
    gyr_bias=gyr_sample.mean(axis=0)
    print(gyr_bias)
    while True:
        if IMU.dataReady():
            time_now=time.time_ns()
            if time_now - last_read >= 100000000:
                IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
                ekf.Dt=(time_now-last_read)/1000000000
                last_read = time_now
                acc_data = np.array([float(IMU.axRaw),float(IMU.ayRaw),float(IMU.azRaw)]) * ACC_FSR/BIT_RESOLUTION
                gyr_data = (np.array([float(IMU.gxRaw),float(IMU.gyRaw),float(IMU.gzRaw)]) * GYR_FSR/BIT_RESOLUTION) - (gyr_bias)
                gyr_data = gyr_data*np.pi/180
                mag_data = (np.array([float(IMU.mxRaw),float(IMU.myRaw),float(IMU.mzRaw)]) * MAG_FSR/BIT_RESOLUTION)
                # print(1/ekf.Dt)
                q = ekf.update( q=q,
                                gyr=gyr_data, 
                                acc=acc_data,
                                # mag=mag_data,
                                # mag=np.array([ mag_data[0] , -mag_data[1] , -mag_data[2] ]),
                                )
                # print(q2euler(q)*180/np.pi)
                print('{: .2f}'.format(mag_data[0]), '{: .2f}'.format(mag_data[1]),'{: .2f}'.format(mag_data[2]),'{: .2f}'.format(acc_data[0]),'{: .2f}'.format(acc_data[1]),'{: .2f}'.format(acc_data[2]), end = "\r")
                # print("")
                # print(mag_data, end = "\r")
def update_cube():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    # pygame.display.set_caption("PyTeapot IMU orientation visualization")
    resizewin(640, 480)
    init()
    ticks = pygame.time.get_ticks()
    while True:
        global q 
        draw(q[0], q[1], q[2], q[3])
        pygame.display.flip()
        time.sleep(1/100
        
        
)


if __name__ == '__main__':
    ekf__thread = threading.Thread(target=run_ekf)
    display_thread = threading.Thread(target=update_cube)
    try:
        ekf__thread.start()
        display_thread.start()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)

from __future__ import print_function
import qwiic_icm20948
import time
import sys
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q, q2euler


ACC_FSR = 4                     # Full scale Range of accelerometer in g
GYR_FSR = 2000                   # Full scale Range of gyroscope in dps
MAG_FSR = 9800                  # Full scale Range of accelerometer in uT

BIT_RESOLUTION = 65536          # Resolution (2^16)

ekf = EKF()


def runExample():
    time_now = 0
    time_prev = 0
    acc_data = np.zeros([1,3])
    gyr_data = np.zeros([1,3])
    mag_data = np.zeros([1,3])
    gyr_sample = np.zeros([1000,3])

    q = acc2q(acc_data)       # First sample of tri-axial accelerometer
    print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    IMU.begin()
    IMU.enableDlpfAccel(True)
    IMU.enableDlpfGyro(True)
    IMU.setFullScaleRangeGyro(0x02)

    print("Calibrating Gyro")
    for i in range(1000):
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
            if time_now - time_prev >= 4800000:
                IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
                acc_data = np.array([float(IMU.axRaw),float(IMU.ayRaw),float(IMU.azRaw)]) * ACC_FSR/BIT_RESOLUTION
                gyr_data = (np.array([float(IMU.gxRaw),float(IMU.gyRaw),float(IMU.gzRaw)]) * GYR_FSR/BIT_RESOLUTION) - (gyr_bias)
                gyr_data = gyr_data*np.pi/180
                mag_data = (np.array([float(IMU.mxRaw),float(IMU.myRaw),float(IMU.mzRaw)]) * MAG_FSR/BIT_RESOLUTION)
                ekf.Dt=(time_now-time_prev)/1000000000
                # print(gyr_data)
                q = ekf.update(q=q, gyr=gyr_data, acc=acc_data)
                print(q2euler(q)*180/np.pi)
                time_prev = time_now
    


if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)

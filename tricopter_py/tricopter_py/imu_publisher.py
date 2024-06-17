from __future__ import print_function
import qwiic_icm20948
import time
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q 
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class IMUPublisher(Node):


    def __init__(self):

        self.imu_msg=Imu()  # Message container for self.IMU getting updated by the self.IMU
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'tricopter/imu', 1)
        timer_period = 0.005  # seconds

        
        self.ACC_FSR = 4                     # Full scale Range of accelerometer in g
        self.GYR_FSR = 4000                   # Full scale Range of gyroscope in dps
        self.MAG_FSR = 9800                  # Full scale Range of accelerometer in uT

        self.BIT_RESOLUTION = 65536          # Resolution (2^16)
        self.N_GYR_CAL = 1000

        self.ekf = EKF()

        self.acc_data = np.zeros([1,3])
        self.gyr_data = np.zeros([1,3])
        self.mag_data = np.zeros([1,3])
        self.gyr_sample = np.zeros([self.N_GYR_CAL,3])
        self.ekf.a_noise = 0.001
        self.ekf.g_noise = 0.1
        self.q = acc2q(self.acc_data)       # First sample of tri-axial accelerometer
        self.time_now = 0
        self.last_read = 0

        self.IMU = qwiic_icm20948.QwiicIcm20948()

        if self.IMU.connected == False:
            self.get_logger().info("The Qwiic ICM20948 device isn't connected to the system. Please check your connection")
            return

        self.IMU.begin()
        self.IMU.enableDlpfAccel(True)
        self.IMU.enableDlpfGyro(True)
        self.IMU.setFullScaleRangeGyro(0x03)

        self.get_logger().info("Calibrating Gyro")
        for i in range(self.N_GYR_CAL):
            if self.IMU.dataReady():
                self.IMU.getAgmt()
                self.gyr_sample[i-1] = np.array([float(self.IMU.gxRaw),float(self.IMU.gyRaw),float(self.IMU.gzRaw)]) * self.GYR_FSR/self.BIT_RESOLUTION
        self.gyr_bias=self.gyr_sample.mean(axis=0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Calibrated Gyro")


            

    def timer_callback(self):

        if self.IMU.dataReady():
            self.time_now=time.time_ns()
            self.IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
            self.ekf.Dt=(self.time_now-self.last_read)/1000000000
            self.last_read = self.time_now
            self.acc_data = np.array([float(self.IMU.axRaw),float(self.IMU.ayRaw),float(self.IMU.azRaw)]) * self.ACC_FSR/self.BIT_RESOLUTION
            self.gyr_data = (np.array([float(self.IMU.gxRaw),float(self.IMU.gyRaw),float(self.IMU.gzRaw)]) * self.GYR_FSR/self.BIT_RESOLUTION) - (self.gyr_bias)
            self.gyr_data = self.gyr_data*np.pi/180
            self.mag_data = (np.array([float(self.IMU.mxRaw),float(self.IMU.myRaw),float(self.IMU.mzRaw)]) * self.MAG_FSR/self.BIT_RESOLUTION)
            self.q = self.ekf.update( q=self.q,
                            gyr=self.gyr_data, 
                            acc=self.acc_data,
                            # mag=self.mag_data,
                            # mag=np.array([ self.mag_data[0] , -self.mag_data[1] , -self.mag_data[2] ]),
                            )
            # print('{: .2f}'.format(self.mag_data[0]), '{: .2f}'.format(self.mag_data[1]),'{: .2f}'.format(self.mag_data[2]),'{: .2f}'.format(self.acc_data[0]),'{: .2f}'.format(self.acc_data[1]),'{: .2f}'.format(self.acc_data[2]), end = "\r")
            # print("")
            # print(self.mag_data, end = "\r")
            self.imu_msg.orientation.w=self.q[0]
            self.imu_msg.orientation.x=self.q[1]
            self.imu_msg.orientation.y=self.q[2]
            self.imu_msg.orientation.z=self.q[3]
            self.publisher_.publish(self.imu_msg)



def main(args=None):
    rclpy.init(args=args)

    imu_publlisher = IMUPublisher()

    rclpy.spin(imu_publlisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publlisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    










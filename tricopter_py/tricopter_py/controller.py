import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tricopter_msgs.msg import Motors, RCMessage, Servo
import numpy as np
import math

class Controller(Node):

    def __init__(self):

        super().__init__('controller')

        self.loop_freq = 200            # Control Loop frequency in Hz
        ctf = 1                         # Coefficient of thrust for forward motors
        ctb = 1                         # Coefficient of thrust for back motor
        cq = 1                          # Coefficient of torque for back motor
        ls = 1                          # Side distance of left and right motors from center of mass
        lb = 1                          # Distance of back motor from center of mass
        lf = 1                          # Distance of front motor from center of mass

        self.servo_speed = 2000         # Servo rotation speed  
        self.servo_acc = 40             # Servo acceleration

        self.is_armed = False

        self.imu_subscription_ = self.create_subscription(Imu, '/tricopter/imu', self.imu_callback, 1)  # Imu subscriber object
        self.imu_subscription_          # prevent unused variable warning
        self.imu_msg = Imu()

        self.rc_subscription_ = self.create_subscription(RCMessage, '/tricopter/rc_message', self.rc_callback, 1)   # Radio controller subscriber object 
        self.rc_subscription_           # prevent unused variable warning
        self.rc_msg = RCMessage()

        self.motor_publisher_ = self.create_publisher(Motors, '/tricopter/motor_cmd',1) # Motor publisher object
        self.motor_msg = Motors()       # Message container for motors
        
        self.servo_publisher_ = self.create_publisher(Servo, '/tricopter/servo_control',1)  # Servo publisher object
        self.servo_msg = Servo()        # Message container for servo

        self.controller_timer = self.create_timer(1/self.loop_freq, self.controller_callback)     # Timer loop for sending controller commands
        
        self.f_n_tau_des = np.zeros([5,1])  # [Fx Fz Tau_Φ Tau_θ Tau_Ψ]
        self.actuators = np.zeros([5,1])    # [Sαrωr^2 Cαrωr^2 Sαlωl^2 Cαlωl^2 ω5^2]


        self.A = np.array([[ 2.*ctf,        0.,             2.*ctf,         0.,             0.      ],
                           [ 0.,            -2.*ctf,        0.,             -2.*ctf,        -ctb    ],
                           [ 0.,            -2.*ctf*ls,     0.,             2.*ctf*ls,      0.0     ],
                           [ 0.,            2.*ctf*lf,      0.,             2.*ctf*lf,     -ctb*lb  ],
                           [ -2.*ctf*ls,    0.,             2.*ctf*ls,      0.,             cq      ]])            # Control allocation matrix

        self.A_inv = np.linalg.inv(self.A)  # Inverse of coltrol allocation matrix

        self.roll , self.pitch , self.yaw = [0., 0., 0.]
        self.roll_error , self.pitch_error , self.yaw_error = [0., 0., 0.]
        self.roll_des , self.pitch_des , self.yaw_des = [0., 0., 0.]
        self.roll_error_prev , self.pitch_error_prev , self.yaw_error_prev = [0., 0., 0.]
        self.roll_error_sum , self.pitch_error_sum , self.yaw_error_sum = [0., 0., 0.]

        self.Kp_roll, self.Kd_roll, self.Ki_roll = [0., 0., 0.]        
        self.Kp_pitch, self.Kd_pitch, self.Ki_pitch = [0., 0., 0.]
        self.Kp_yaw, self.Kd_yaw, self.Ki_yaw = [0., 0., 0.]

    def imu_callback(self,msg):
        # Copy imu message  
        self.imu_msg = msg

    def rc_callback(self,msg):
        # Copy the rc message
        self.rc_msg = msg

        # Check for arming switch and throtlle low
        if not self.is_armed:
            if self.rc_msg.aux1>1000 and self.rc_msg.rc_throttle<1000:
                self.is_armed = True
        # Check for disarm
        if self.is_armed and self.rc_msg.aux1<1200:
            self.disarm()
            

    def controller_callback(self):
        if self.is_armed:

            # Convert quaternion to euler angles 
            self.yaw, self.pitch, self.roll = self.quat_to_ypr([self.imu_msg.orientation.w, self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z])
            self.pitch = -self.pitch        # Allign to IMU orientation 
            self.roll = -self.roll          # Allign to IMU orientation 

            # Calculate error 

            self.roll_error = self.roll_des - self.roll
            self.pitch_error = self.pitch_des - self.pitch
            self.yaw_error = self.yaw_des - self.yaw

            # PID loop
            self.f_n_tau_des[3][0] = self.Kp_pitch * self.pitch_error + self.Kd_pitch * (self.pitch_error - self.pitch_error_prev) + self.Ki_pitch * self.pitch_error_sum
            self.f_n_tau_des[2][0] = self.Kp_roll * self.roll_error + self.Kd_roll * (self.roll_error - self.roll_error_prev) + self.Ki_roll * self.roll_error_sum
            self.f_n_tau_des[4][0] = self.Kp_yaw * self.yaw_error + self.Kd_yaw * (self.yaw_error - self.yaw_error_prev) + self.Ki_yaw * self.yaw_error_sum


            # Find the actuator commands to realize desired forces and torques
            self.actuators = self.A_inv * self.f_n_tau_des

            self.ωr = np.sqrt(np.sqrt( np.squar(self.actuators[0][0]), np.square(self.actuators[1][0]) ) )
            self.ωl = np.sqrt(np.sqrt( np.squar(self.actuators[2][0]), np.square(self.actuators[3][0]) ) )
            self.ω5 = np.sqrt(self.actuators[4][0])
            self.ar = np.arcsin(self.actuators[0][0]/self.ωr)
            self.al = np.arcsin(self.actuators[2][0]/self.ωl)

            # Publish motors commands
            self.motor_publisher_.publish(Motors(motor_lu=float(self.rc_msg.rc_throttle),
                                                motor_ru=float(self.rc_msg.rc_throttle),
                                                motor_ld=float(self.rc_msg.rc_throttle),
                                                motor_rd=float(self.rc_msg.rc_throttle),
                                                motor_b=float(self.rc_msg.rc_throttle)
                                                ))
            
            # Publish servo commands
            self.servo_msg.left_pose = int( (self.rc_msg.rc_pitch - 1500) * 512/500 + 2048)
            self.servo_msg.right_pose = int( (self.rc_msg.rc_pitch - 1500) * 512/500 + 2048)
            self.servo_msg.left_speed = self.servo_speed
            self.servo_msg.right_speed = self.servo_speed
            self.servo_msg.left_acc = self.servo_acc
            self.servo_msg.right_acc = self.servo_acc
            self.servo_publisher_.publish(self.servo_msg)
    
    def disarm (self):
        self.is_armed = False
        self.motor_publisher_.publish(Motors(motor_lu = 900.0 ,motor_ru = 900.0 ,motor_ld = 900.0 ,motor_rd = 900.0 ,motor_b = 900.0))
        self.servo_publisher_.publish(Servo(left_pose=2048, right_pose = 2048))

    def quat_to_ypr(self,q):
        yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
        pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
        roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
        pitch *= 180.0 / math.pi
        yaw   *= 180.0 / math.pi
        # yaw   -= -0.13          # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
        roll  *= 180.0 / math.pi
        return [yaw, pitch, roll]


def main(args=None):
    
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.disarm()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






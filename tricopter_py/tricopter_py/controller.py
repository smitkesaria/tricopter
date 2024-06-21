import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tricopter_msgs.msg import Motors, RCMessage, Servo

class Controller(Node):

    def __init__(self):

        super().__init__('controller')

        self.loop_freq = 200            # Control Loop frequency

        self.servo_speed = 2000         # Servo rotation speed
        self.servo_acc = 40           # Servo acceleration

        self.is_armed = False

        # self.imu_subscription_ = self.create_subscription(Imu, '/tricopter/imu', self.imu_callback, 1)
        # self.imu_subscription_        # prevent unused variable warning
        self.imu_msg = Imu()

        self.rc_subscription_ = self.create_subscription(RCMessage, '/tricopter/rc_message', self.rc_callback, 1)   # Radio controller subscriber object 
        self.rc_subscription_           # prevent unused variable warning
        self.rc_msg = RCMessage()

        self.motor_publisher_ = self.create_publisher(Motors, '/tricopter/motor_cmd',1) # Motor publisher object
        self.motor_msg = Motors()       # Message container for motors
        
        self.servo_publisher_ = self.create_publisher(Servo, '/tricopter/servo_control',1)  # Servo publisher object
        self.servo_msg = Servo()        # Message container for servo

        self.motot_timer = self.create_timer(1/self.loop_freq, self.motor_callback)     # Timer loop for sending motor commands
        self.servo_timer = self.create_timer(1/self.loop_freq, self.servo_callback)     # Timer loop for sending servo commands

    
    # def imu_callback(self,msg):
        # Copy imu message  
    #     self.imu_msg = msg

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
            

    def motor_callback(self):
        if self.is_armed:
            self.motor_publisher_.publish(Motors(motor_lu=float(self.rc_msg.rc_throttle),
                                                motor_ru=float(self.rc_msg.rc_throttle),
                                                motor_ld=float(self.rc_msg.rc_throttle),
                                                motor_rd=float(self.rc_msg.rc_throttle),
                                                motor_b=float(self.rc_msg.rc_throttle)
                                                ))

    def servo_callback(self):
        if self.is_armed:
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


def main(args=None):
    
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.disarm()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






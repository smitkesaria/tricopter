from scservo_sdk.servo_utils import ServoUtils
import rclpy
import time
from rclpy.node import Node

from tricopter_msgs.msg import Servo


class ServoController(Node):

    def __init__(self):
        super().__init__('servo_controller')

        self.servo_utils=ServoUtils()
        self.subscription = self.create_subscription(
            Servo,
            '/tricopter/servo_control',
            self.servo_controll_callback,
            1)
        self.subscription  # prevent unused variable warning

    def servo_controll_callback(self, msg):
        self.servo_utils.move_servo(id=1, position=msg.left_pose, speed=msg.left_speed, acc=msg.left_acc)       #Assuming left servo has id=1 
        self.servo_utils.move_servo(id=2, position=msg.right_pose, speed=msg.right_speed, acc=msg.right_acc)    #Assuming right servo has id=2 



def main(args=None):
    rclpy.init(args=args)

    servo_controller = ServoController()

    rclpy.spin(servo_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
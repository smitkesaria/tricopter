import rclpy
from rclpy.node import Node
import serial

import tricopter_py.crsf as crsf

from tricopter_msgs.msg import RCMessage

SERIAL_PORT = "/dev/ttyAMA10"
BAUDRATE = 420000


DEFAULT_ROLL_VALUE = 1500
DEFAULT_PITCH_VALUE = 1500
DEFAULT_YAW_VALUE = 1500
DEFAULT_THROTTLE_VALUE = 988


class RosCsrf(Node):

    def __init__(self):
        super().__init__('rc_command')

        timer_period = 1/1000  # seconds

        self.get_logger().info("Ros2_CSRF node started")

        self.rc_pub = self.create_publisher(RCMessage, 'tricopter/rc_message', 1)

        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
        self.input = bytearray()
        self.unique = []

        self.timer = self.create_timer(timer_period, self.timer_callback)

    
    def timer_callback(self):

        if self.ser.in_waiting > 0:
            self.input.extend(self.ser.read(self.ser.in_waiting))
                
        if len(self.input) > 2:
            # This simple parser works with malformed CRSF streams
            # it does not check the first byte for SYNC_BYTE, but
            # instead just looks for anything where the packet length
            # is 4-64 bytes, and the CRC validates
            expected_len = self.input[1] + 2
            if expected_len > 64 or expected_len < 4:
                self.input = []
            elif len(self.input) >= expected_len:
                single_frame = self.input[:expected_len] # copy out this whole packet
                self.input = self.input[expected_len:] # and remove it from the buffer

                if not crsf.crsf_validate_frame(single_frame): # single_frame[-1] != crc:
                    # packet = ' '.join(map(hex, single_frame))
                    # print(f"crc error: {packet}")
                    pass
                else:
                    # crsf.handleCrsfPacket(single_frame[2], single_frame)
                    # if single_frame[2] not in self.unique:
                    #     self.unique.append(single_frame[2])
                    if single_frame[2]==crsf.PacketsTypes.RC_CHANNELS_PACKED:
                        self.pwm=crsf.crsf2pwm(single_frame[3:25])
                        # print(self.pwm)
                        self.rc_pub.publish(RCMessage(rc_roll=int(self.pwm[0]), rc_pitch=int(self.pwm[1]), rc_throttle=int(self.pwm[2]), rc_yaw=int(self.pwm[3]), aux1=int(self.pwm[4]), aux2=int(self.pwm[5]), aux3=int(self.pwm[6]), aux4=int(self.pwm[7])))

def main(args=None):
    rclpy.init()
    ros2_csrf = RosCsrf()  
    rclpy.spin(ros2_csrf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

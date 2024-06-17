# Uses SC Servo SDK library
from .port_handler import *
from .protocol_packet_handler import *
from .group_sync_write import *
from .group_sync_read import *
from .sms_sts import *
from .scscl import *     

import time

class ServoUtils():

    def __init__(self) -> None:

        self.baudrate = 115200           # SC Servo default baudrate : 115200
        self.devicename = '/dev/ttyUSB0'    # Check which port is being used on your controller
    
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.devicename)

        # Initialize PacketHandler instance
        # Get methods and members of Protocol
        self.packetHandler = sms_sts(self.portHandler)
    
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the Servo port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def move_servo (self,id,position, speed, acc):

        # Write SC Servo goal position/moving speed/moving acc
        time_now = time.time_ns()
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(id, position, speed, acc)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
        print((time.time_ns()-time_now)/1000000000)

    def close_port(self):
        # Close port
        self.portHandler.closePort()
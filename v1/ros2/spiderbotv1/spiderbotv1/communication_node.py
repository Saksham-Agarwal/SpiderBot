#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from serial import Serial, SerialException
from serial.tools import list_ports

ARDUINO_SERIAL_NUMBERS = [
    "557373130313519040B1",
    "8513332303635160F041"
]

# TODO:
# 1. Test is arduinos are connected and communication is working by sending a set sequence and receiving a set sequence
# 2. Subscribe to servo angles and send to arduino

class CommunicationNode(Node):

    def __init__(self):
        super().__init__("communication_node")
        self.get_logger().info("Starting communication node")

        rospy
        
        devices = list_ports.comports()
        ports = []
        for device in devices:
            #print(device)
            port, desc, hwid = device
            serial_number = hwid.split()[2].split("=")[1]
            
            if serial_number in ARDUINO_SERIAL_NUMBERS:
                #print(serial_number)
                ports.append(port)
        
        for port in ports:
            try:
                self.arduino = Serial(port, baudrate=9600, timeout = 1)
                time.sleep(1)
                self.arduino.flush()
                self.get_logger().info("Connection to arduino on port {}".format(port))
            except SerialException as e:
                self.get_logger().info(f"There was a problem connecting to arduino on port {port}.\n{e.with_traceback()}")
                
    def send_data(self,msg):
        if  self.arduino.is_open:
            data = bytearray([70,19,65])
            print(data)
            self.arduino.write(data[1])
            self.arduino.reset_input_buffer()
            time.sleep(.5)


    
    def receive_data(self):
        if self.arduino.is_open:
            
            data = self.arduino.readline()
            print("here")
            print(data)
       
def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    while True:
        node.receive_data()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
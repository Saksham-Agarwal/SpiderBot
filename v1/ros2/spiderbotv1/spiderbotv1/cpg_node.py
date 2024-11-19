#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from .cpg import CPG
import numpy as np

class CPG_Node(Node):
    def __init__(self):
        super().__init__("cpg_node")
        self.get_logger().info("Starting CPG Node")
        
        # Parameters
        self.default_amp_offset = [0,0,0,0,0,0,90,90,90,90,90,90,90,90,90,90,90,90,0,0] #ask tubki
        self.amp_offset = self.default_amp_offset[:]
        # self.amp_offset = [30,-30,30,30,30,30,0,20,0,20,-60,20,60,20,0,20,0,20,1,1]
        self.update_period = 0.05 # publishing rate
        self.n = 1 # ask tubki
        
        self.joint_angles = [0 for _ in range(18)]
        
        # Setting up timer
        self.create_timer(self.update_period, self.timer_callback) # pubishes at evry 50ms

        # Initialize publisher
        self.publisher123 = self.create_publisher(Float32MultiArray, "/leg123", 10) 
        self.publisher456 = self.create_publisher(Float32MultiArray, "/leg456", 10)
        # Subscribe to joy_node
        self.joy_node_subscriber = self.create_subscription(Joy, "/joy", self.joystick_input_callback, 10)
        self.cpg = CPG()

    def timer_callback(self):
        computed_cpg = self.cpg.simulate(self.update_period) # updating x and y
        lower = computed_cpg[:6] # J1 of each leg
        upper = computed_cpg[6:] #J2 of each leg
                
        for i in range(6):
            self.joint_angles[3*i] = 0
            if (i % 2 == 0):
                self.joint_angles[3*i] += self.amp_offset[i] * (np.cos(np.arctan2(upper[i], lower[i]) + np.pi) + 1) #  angles calculating for L2,4,6(J1)
            else:
                self.joint_angles[3*i] -= self.amp_offset[i] * (np.cos(np.arctan2(upper[i], lower[i]) + np.pi) + 1) #   angles calculating for L1,3,5(J1)
            self.joint_angles[3*i+1] = min(25*upper[i], 0) * self.amp_offset[19] # ANGLES CALCULATING for J2(clipping)
            
            self.joint_angles[3*i] += (90 * self.n) + self.amp_offset[6 + 2*i] # ask tubki

            self.joint_angles[3*i+1] += 90.0 # offset
            self.joint_angles[3*i+2] = 145.0 # offset
            
        msg123 = Float32MultiArray()
        msg456 = Float32MultiArray()
        
        print(self.joint_angles)
        
        msg123.data = self.joint_angles[:9]
        msg456.data = self.joint_angles[9:]
        self.publisher123.publish(msg123)
        self.publisher456.publish(msg456)
        
    def joystick_input_callback(self, msg: Joy): #ask tubki regarding states
        if np.count_nonzero(msg.buttons) != 0:
            self.n = 1
            # Left
            if msg.buttons[0]:
                self.amp_offset = [-30,-30,30,-30,30,30,0,20,0,20,0,20,-60,20,0,20,0,20,1,1]
            # Back
            elif msg.buttons[1]:
                self.amp_offset = [-30,30,-30,-30,-30,-30,60,20,0,20,0,20,0,20,0,20,0,20,1,1]
            # Right    
            elif msg.buttons[2]:
                self.amp_offset = [30,30,-30,30,-30,-30,-60,20,60,20,0,20,0,20,0,20,0,20,1,1]
            # Forward    
            elif msg.buttons[3]:
                self.amp_offset = [30,-30,30,30,30,30,0,20,0,20,-60,20,60,20,0,20,0,20,1,1]
            # Rotate    
            elif msg.buttons[4]:
                self.amp_offset = [30,-30,30,-30,-30,-30,0,20,0,20,0,20,0,20,0,20,0,20,1,1]
            # Rotate    
            elif msg.buttons[5]:
                self.amp_offset = [-30,30,-30,30,30,30,0,20,0,20,0,20,0,20,0,20,0,20,1,1]
        # Standing case     
        else:
            self.amp_offset = self.default_amp_offset[:]
            self.n = 0


def main(args=None):
    rclpy.init(args=args)
    node = CPG_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy


class StepperNode(Node): 
    def __init__(self):
        super().__init__("stepper_node")   

        #publisher for controlling the stepper motor
        self.stepper_publisher = self.create_publisher(Int32, "stepper_control", 10)  

        #subscriber to receive joystick input
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.joy_callback, 10) 

        self.get_logger().info("Stepper node has been started")

        # Initialize the motor position in degrees
        self.degrees = 0 

        #last buttons state
        self.last_x =  0
        self.last_o =  0

    def joy_callback(self, msg):

        # Get the current state of the X and O buttons from the joystick
        ccw_x = msg.buttons[0]
        cw_o = msg.buttons[1]
        msg = Int32()

        
        # Check if both buttons are pressed
        if ccw_x == 1 and cw_o == 1:  
            self.get_logger().info("Both buttons pressed, no movement.")

        #X button is pressed, move cw
        elif ccw_x == 1 and self.last_x == 0:
            msg.data  = 1
        
            
        #O button is pressed, move ccw
        elif cw_o == 1 and self.last_o == 0:
            msg.data = -1
        

        #update last button state
        self.last_x = ccw_x
        self.last_o = cw_o
        self.stepper_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StepperNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
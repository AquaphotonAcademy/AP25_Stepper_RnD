#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy


class StepperNode(Node): 
    def __init__(self):
        super().__init__("stepper_node")   

        # Publisher for controlling the stepper motor
        self.stepper_publisher = self.create_publisher(Int32, "stepper_control", 10)  

        # Publisher for test signal from Triangle button
        self.test_publisher = self.create_publisher(Int32, "test_first", 10)

        # Subscriber to receive joystick input
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.joy_callback, 10) 

        self.get_logger().info("Stepper node has been started")

        # Initialize last button states
        self.last_x = 0
        self.last_o = 0
        self.last_t = 0  # Track last state of Triangle button

    def joy_callback(self, msg):
        # Get button states
        ccw_x = msg.buttons[0]  # X button
        cw_o = msg.buttons[1]   # O button
        triangle = msg.buttons[2]  # Triangle button

        stepper_msg = Int32()
        test_msg = Int32()

        # Check if both X and O buttons are pressed
        if ccw_x == 1 and cw_o == 1:  
            self.get_logger().info("Both buttons pressed, no movement.")

        # X button is pressed, move CW
        elif ccw_x == 1 and self.last_x == 0:
            stepper_msg.data = 1
        
        # O button is pressed, move CCW
        elif cw_o == 1 and self.last_o == 0:
            stepper_msg.data = -1

        # Triangle button test signal
        if triangle == 1:
        # and self.last_t == 0:
            test_msg.data = 1  # Publish 1 when Triangle is pressed
        # elif triangle == 0 and self.last_t == 1:
        else:
            test_msg.data = 0  # Publish 0 when Triangle is released
        
        # Update last button states
        self.last_x = ccw_x
        self.last_o = cw_o
        self.last_t = triangle

        # Publish messages
        self.stepper_publisher.publish(stepper_msg)
        self.test_publisher.publish(test_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StepperNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

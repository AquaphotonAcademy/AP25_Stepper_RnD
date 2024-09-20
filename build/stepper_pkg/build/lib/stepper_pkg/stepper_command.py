#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class StepperNode(Node): 
    def __init__(self):
        super().__init__("stepper_command")   
        self.stepper_publisher = self.create_publisher(Float32MultiArray, "stepper_command", 10)
        self.timer = self.create_timer(1.0, self.publish_command)

        self.dir = 1.0  # 1.0 for clockwise, -1.0 for counterclockwise
        self.step_angle = 1.8 # Step angle depends on motor, e.g., 1.8 or 0.9
        self.resolution = 1.0  # Resolution for stepping, full step, half step, etc.

        self.get_logger().info("Stepper node has been started")


    def publish_command(self):
        msg = Float32MultiArray()
        msg.data = [self.dir, self.step_angle, self.resolution]
        self.stepper_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = StepperNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

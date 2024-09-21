#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class StepperNode(Node): 
    def __init__(self):
        super().__init__("stepper_node")   
        self.stepper_publisher = self.create_publisher(Int32, "stepper_control", 10)
        self.timer = self.create_timer(1.0, self.publish_command)


        self.get_logger().info("Stepper node has been started")


    def publish_command(self):
        degrees= 20
        msg = Int32()
        msg.data = degrees

        self.stepper_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = StepperNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

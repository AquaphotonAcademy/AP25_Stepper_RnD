#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from rclpy.duration import Duration

class StepperNode(Node): 
    def __init__(self):
        super().__init__("stepper_node")   
        self.stepper_publisher = self.create_publisher(Int32, "stepper_control", 10)
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        # self.timer = self.create_timer(1.0, self.publish_command)
        self.get_logger().info("Stepper node has been started")
        self.degrees = 0
        self.last_press_time = self.get_clock().now()  
        self.debounce_duration = Duration(seconds=0.5) 

    def joy_callback(self, msg):
         
        current_time = self.get_clock().now()

        # Debouncing logic using ROS time
        if current_time - self.last_press_time >= self.debounce_duration:

            if msg.buttons[0] == 1 and msg.buttons[1] == 1:  
                self.get_logger().info("Both buttons pressed, no movement.")

            elif msg.buttons[0] == 1:
                self.degrees += 10
                self.last_press_time = current_time
            

            elif msg.buttons[1] == 1:
                self.degrees -= 10
                self.last_press_time = current_time

            msg_angle = Int32()
            msg_angle.data = self.degrees
            self.stepper_publisher.publish(msg_angle)
            # self.get_logger().info(f"Stepper motor position: {self.degrees} ")



    # def publish_command(self):
    #     degrees= 20
    #     msg = Int32()
    #     msg.data = degrees

    #     self.stepper_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = StepperNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

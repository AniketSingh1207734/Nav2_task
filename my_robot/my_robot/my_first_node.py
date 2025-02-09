#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("First_Node")
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback (self):
        self.get_logger().info("Hello from First Node " + str(self.counter))
        self.counter += 1

def main(args=None) :
    rclpy.init(args=args)
    Node = MyNode()
    rclpy.spin(Node)


    rclpy.shutdown()


if(__name__)=='__main__':
    main()

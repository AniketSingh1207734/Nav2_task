#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import numpy as np
from transforms3d.euler import euler2quat, quat2euler

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('initial_pose_x', 0.0),
                ('initial_pose_y', 0.0),
                ('initial_pose_theta', 0.0)
            ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.set_initial_pose()

    def set_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        initial_pose.pose.pose.position.y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
        quaternion = euler2quat(0, 0, self.get_parameter('initial_pose_theta').get_parameter_value().double_value)
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(initial_pose)
        self.get_logger().info("Initial pose set.")

    def send_navigation_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        quaternion = euler2quat(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quaternion[0]
        goal_msg.pose.pose.orientation.y = quaternion[1]
        goal_msg.pose.pose.orientation.z = quaternion[2]
        goal_msg.pose.pose.orientation.w = quaternion[3]

        self.get_logger().info("Sending goal ...")

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            self.prompt_for_next_goal()
            return

        self.get_logger().info('Goal accepted.')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached the target position.')
            self.activate_marker(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
            self.prompt_for_next_goal()
        else:
            self.get_logger().info('Failed to reach the target position.')
            self.prompt_for_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def activate_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'marker_light'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        self.get_logger().info("Marker light activated.")

    def prompt_for_next_goal(self):
        x = float(input("Enter the x coordinate for the next goal: "))
        y = float(input("Enter the y coordinate for the next goal: "))
        theta = float(input("Enter the orientation (theta) for the next goal: "))
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        quaternion = euler2quat(0, 0, theta)
        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]
        self.send_navigation_goal(x, y, theta)

def main(args=None):
    rclpy.init(args=args)
    navigator = RobotNavigator()
    navigator.prompt_for_next_goal()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
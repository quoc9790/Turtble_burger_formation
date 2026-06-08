#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class SimpleNavClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 0.5
        goal_msg.pose.pose.orientation.w = 1.0

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal bị từ chối 😢')
            return

        self.get_logger().info('Goal được chấp nhận 🎯')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Vị trí hiện tại: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}')

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Thành công: {result.success}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

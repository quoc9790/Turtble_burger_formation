#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # Thông số điều khiển
        self.L = 0.05
        self.Kp = 1    
        self.Ki = 0.0   
        self.integral_err_2 = np.zeros((2, 1))

        self.last_time = time.time()
        
        # toa do leader
        self.leader_x = 1
        self.leader_y = 0.5

        # vi tri tuong doi mong muon so voi leader
      
        self.h_2_0_des = np.array([[0.433],
                                   [0.25]])

        # Subscriber

        self.odom_sub_2 = self.create_subscription(
            Odometry, "/robot2/odom", self.callback_odom2, 10)

        # Publisher
        self.pub_2 = self.create_publisher(Twist, "/robot2/cmd_vel", 10)

    
    def callback_odom2(self, msg2: Odometry):
        # now = time.time()
        # dt = now - self.last_time
        # self.last_time = now
        dt=0.01

        follower2_x = msg2.pose.pose.position.x
        follower2_y = msg2.pose.pose.position.y
        h_2_0 = np.array([[self.leader_x - follower2_x],
                          [self.leader_y - follower2_y]])

        q = msg2.pose.pose.orientation
        list=[q.x,q.y,q.z,q.w]
        roll, pitch, theta_2 = euler_from_quaternion(list)

        e_2 = h_2_0 - self.h_2_0_des

        u, r, self.integral_err_2 = self.control_law(theta_2, e_2, self.integral_err_2, dt)

        # Publish vận tốc
        cmd_2 = Twist()
        cmd_2.linear.x = float(np.clip(u * 2.0, -0.3, 0.3))   # nhân hệ số để dễ vượt ngưỡng ma sát
        cmd_2.angular.z = float(np.clip(r, -1.0, 1.0))        # giảm giới hạn góc
        self.pub_2.publish(cmd_2)

        self.get_logger().info(f"Robot2 -> e:{e_2.flatten()} u:{cmd_2.linear.x:.2f}, r:{cmd_2.angular.z:.2f}")

    def control_law(self, theta, e, integral_err, dt):
        # Ma tran B
        g1 = np.array([[math.cos(theta)],
                       [math.sin(theta)]])
        g2 = np.array([[-math.sin(theta)],
                       [math.cos(theta)]])
        B = np.hstack((g1, self.L * g2))

        # tich phan (*ki)
        integral_err += e * dt

        # uat dieu khien
        control = np.linalg.inv(B) @ (self.Kp * e + self.Ki * integral_err)
        u = control[0, 0]
        r = control[1, 0]

        return u, r, integral_err


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

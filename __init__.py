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
        self.Kp = 2.0   
        self.Ki = 0.0   
        self.integral_err_1 = np.zeros((2, 1))
        self.integral_err_2 = np.zeros((2, 1))

        self.last_time = time.time()
        self.start_time=time.time()
        
        # toa do leader
        self.leader_x = 1
        self.leader_y = 0.5

        # vi tri tuong doi mong muon so voi leader
        self.h_1_0_des = np.array([[0.433],
                                   [-0.25]])
        self.h_2_0_des = np.array([[0.433],
                                   [0.25]])

        # Subscriber
        self.odom_sub_1 = self.create_subscription(
            Odometry, "/robot1/odom", self.callback_odom1, 10)
        self.odom_sub_2 = self.create_subscription(
            Odometry, "/robot2/odom", self.callback_odom2, 10)

        # Publisher
        self.pub_1 = self.create_publisher(Twist, "/robot1/cmd_vel", 10)
        self.pub_2 = self.create_publisher(Twist, "/robot2/cmd_vel", 10)
        #tao timer update leader position
        self.timer_leader = self.create_timer(0.1, self.update_leader_position)

    def update_leader_position(self):
        
        t = time.time() - self.start_time
        self.leader_x = 1.0 + 0.05* t   # doc truc X 10cm/1s
        self.leader_y = 0.5 
    def callback_odom1(self, msg1: Odometry):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        follower1_x = msg1.pose.pose.position.x
        follower1_y = msg1.pose.pose.position.y
        h_1_0 = np.array([[self.leader_x - follower1_x],
                          [self.leader_y - follower1_y]])
        
        #  chuyen tu quaternion-> theta
        q = msg1.pose.pose.orientation
        list=[q.x,q.y,q.z,q.w]
        roll, pitch, theta_1 = euler_from_quaternion(list)


        # sai so
        e_1 = h_1_0 - self.h_1_0_des

        # luat dieu khien
        u, r, self.integral_err_1 = self.control_law(theta_1, e_1, self.integral_err_1, dt)

        # Publish van toc
        cmd_1 = Twist()
        cmd_1.linear.x = float(max(min(u, 0.2), -0.2))     # giới hạn vận tốc tuyến tính
        cmd_1.angular.z = float(max(min(r, 1.5), -1.5))    # giới hạn vận tốc góc
        self.pub_1.publish(cmd_1)

        self.get_logger().info(f"Robot1 -> e:{e_1.flatten()} u:{cmd_1.linear.x:.2f}, r:{cmd_1.angular.z:.2f}")

    def callback_odom2(self, msg2: Odometry):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        follower2_x = msg2.pose.pose.position.x
        follower2_y = msg2.pose.pose.position.y
        h_2_0 = np.array([[self.leader_x - follower2_x],
                          [self.leader_y - follower2_y]])

        q = msg2.pose.pose.orientation
        list=[q.x,q.y,q.z,q.w]
        roll, pitch, theta_2 = euler_from_quaternion(list)

        e_2 = h_2_0 - self.h_2_0_des

        u, r, self.integral_err_2 = self.control_law(theta_2, e_2, self.integral_err_2, dt)

        cmd_2 = Twist()
        cmd_2.linear.x = float(max(min(u, 0.2), -0.2))
        cmd_2.angular.z = float(max(min(r, 1.5), -1.5))
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

        # luat dieu khien
        #control = np.linalg.inv(B) @ (self.Kp * e + self.Ki * integral_err)
        control = np.linalg.pinv(B) @ (self.Kp * e + self.Ki * integral_err)
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

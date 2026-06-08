#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import time
from my_robot_interfaces.msg import EstimationError
class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # thong so ban dau
        self.k_heading= 4.0
        self.L = 0.05
        self.integral_err_1 = np.zeros((2, 1))
        self.integral_err_2 = np.zeros((2, 1))
        self.dt=0.1
        self.start_time = time.time()
        #parametter
        self.declare_parameter("velo", 0.05)
        self.declare_parameter("Kp", 2.0)
        self.declare_parameter("Ki", 0.4)

        self.velo_ = self.get_parameter("velo").value
        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        
        # toa do leader
        self.leader_x = 1.0
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

        self.estimation_error_1=self.create_publisher(EstimationError,"/robot1/error",10)
        self.estimation_error_2=self.create_publisher(EstimationError,"/robot2/error",10)

        self.total_error_pub=self.create_publisher(EstimationError,"/total_error",10)
        #vantoc dau
        self.cmd_1=Twist()
        self.cmd_1.linear.x=0.0

        self.cmd_2=Twist()
        self.cmd_2.linear.x=0.0
        #sai so ban dau
        self.e_1=None
        self.e_2=None
        #bien phu
        self.init_velo_1=False
        self.init_velo_2=False

        self.epsilon_1 = np.array([[1],
                                   [0]])
        self.epsilon_2 = np.array([[1],
                                   [0]])

        self.M = np.array([[0, -1],
                          [1,  0]])
        #tao timer 
        self.timer_leader = self.create_timer(0.1, self.update_leader_position)     #update vi tri leader
        self.timer_error = self.create_timer(0.1, self.public_tracking_error)       #publich lich tong sai so vi tri
    def update_leader_position(self):
        
        t = time.time() - self.start_time
        self.leader_x = 1.0 + self.velo_ * t   # doc truc X 5cm/1s
        self.leader_y = 0.5        
    def callback_odom1(self, msg1: Odometry):
       
        if not (self.init_velo_1):
            self.pub_1.publish(self.cmd_1)
            self.init_velo_1=True
        
        else:
            follower1_x = msg1.pose.pose.position.x
            follower1_y = msg1.pose.pose.position.y

            h_1_0 = np.array([[self.leader_x - follower1_x],
                            [self.leader_y - follower1_y]])
            #estimate heading
            p_1=np.array([[follower1_x],
                        [follower1_y]])
            g1_1=self.epsilon_1+self.k_heading*p_1

            epsilon_dot = msg1.twist.twist.angular.z * self.M @ g1_1 - self.k_heading * msg1.twist.twist.linear.x * g1_1

            self.epsilon_1 = self.epsilon_1 +epsilon_dot*self.dt
            # sai so vitri
            self.e_1 = h_1_0 - self.h_1_0_des
            #tinh toan sai so huong
            q = msg1.pose.pose.orientation
            list=[q.x,q.y,q.z,q.w]
            roll, pitch, theta_real_1 = euler_from_quaternion(list)
            
            error_theta_1=EstimationError()
            error_theta_1.error=self.estimation_error(g1_1,theta_real_1)

            #publish sai so huong
            self.estimation_error_1.publish(error_theta_1)
            self.get_logger().info(f"Robot1 heading error: {math.degrees(error_theta_1.error):.2f}°")

            # luat dieu khien
            u, r, self.integral_err_1 = self.control_law(g1_1, self.e_1, self.integral_err_1, self.dt)

            # Publish van toc
            
            self.cmd_1.linear.x = float(max(min(u, 0.2), -0.2))     # giới hạn vận tốc tuyến tính
            self.cmd_1.angular.z = float(max(min(r, 1.5), -1.5))    # giới hạn vận tốc góc
            self.pub_1.publish(self.cmd_1)

            self.get_logger().info(f"Robot1 -> e:{self.e_1.flatten()} u:{self.cmd_1.linear.x:.2f}, r:{self.cmd_1.angular.z:.2f}")

    def callback_odom2(self, msg2: Odometry):

        if not self.init_velo_2:
            self.pub_2.publish(self.cmd_2)
            self.init_velo_2 = True
        
        else:
            follower2_x = msg2.pose.pose.position.x
            follower2_y = msg2.pose.pose.position.y
            h_2_0 = np.array([[self.leader_x - follower2_x],
                              [self.leader_y - follower2_y]])
                
            #estimate haeding
            p_2 = np.array([[follower2_x],
                        [follower2_y]])
            g1_2 = self.epsilon_2 + self.k_heading * p_2
            epsilon_dot = msg2.twist.twist.angular.z * self.M @ g1_2 - self.k_heading * msg2.twist.twist.linear.x * g1_2
            self.epsilon_2 = self.epsilon_2 + epsilon_dot * self.dt

            # Sai so
            self.e_2 = h_2_0 - self.h_2_0_des

            #tinh toan sai so huong
            q = msg2.pose.pose.orientation
            list=[q.x,q.y,q.z,q.w]
            roll, pitch, theta_real_2 = euler_from_quaternion(list)
            
            error_theta_2=EstimationError()
            error_theta_2.error=self.estimation_error(g1_2,theta_real_2)
            #publish sai so huong
            self.estimation_error_2.publish(error_theta_2)
            self.get_logger().info(f"Robot2 heading error: {math.degrees(error_theta_2.error):.2f}°")
            # dieukhien
            u, r, self.integral_err_2 = self.control_law(g1_2, self.e_2, self.integral_err_2, self.dt)

            # Publish vantoc
            self.cmd_2.linear.x = float(max(min(u, 0.2), -0.2))
            self.cmd_2.angular.z = float(max(min(r, 1.5), -1.5))
            self.pub_2.publish(self.cmd_2)

            self.get_logger().info(f"Robot2 -> e:{self.e_2.flatten()} u:{self.cmd_2.linear.x:.2f}, r:{self.cmd_2.angular.z:.2f}")
    def control_law(self,g1, e, integral_err, dt):
        # Ma tran B
        g2 =self.M@g1
        B = np.hstack((g1, self.L * g2))

        # tich phan (*ki)
        integral_err += e * dt

        # luat dieu khien
                #control = np.linalg.inv(B) @ (self.Kp * e + self.Ki * integral_err)
        control = np.linalg.pinv(B) @ (self.Kp * e + self.Ki * integral_err)
        u = control[0, 0]
        r = control[1, 0] 
        if abs(u) < 0.01 and abs(r) < 0.1:
             u = 0.02
             r = 0.1

        return u, r, integral_err
    
    def estimation_error(self,g1,theta_real):
        theta_est = math.atan2(g1[1,0], g1[0,0])
        error = theta_est - theta_real
        return error
    def public_tracking_error(self):
        if self.e_1 is not None and self.e_2 is not None:
            total_error = float(np.linalg.norm(self.e_1) + np.linalg.norm(self.e_2))
            msg=EstimationError()
            msg.error=total_error
            self.total_error_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
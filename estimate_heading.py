#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class Turtlebot3ControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("controller") # MODIFY NAME
        self.cmd_vel_publisher_=self. create_publisher(Twist,"/cmd_vel",10)
        self.timer=self.create_timer(0.5,self.callPublisher)
        self.lidar_subscriber=self.create_subscription(LaserScan,"/scan",self.callback_scan,10)
        self.impact=False
        self.get_logger().info("started")
        
    def callback_scan(self, msg:LaserScan ):
        ranges=msg.ranges
        ranges__=ranges[(len(ranges)//2-15):(len(ranges)+15)]
        dist= min(ranges__)
        if dist<=0.5 :
             self.impact=True
        else:
             self.impact=False
    def callPublisher(self):
        msg=Twist()   
        if self.impact:
            msg.linear.x=0.0
            msg.angular.z=0.15
        
        else:
            msg.linear.x=0.4
            msg.angular.z=0.0
        self.cmd_vel_publisher_.publish(msg)

     
     
def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3ControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
     
if __name__ == "__main__":
        main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import sys
import numpy as np
from math import pow, atan2, sqrt

class GControl(Node):
    def __init__(self):
        super().__init__('lab05')

        self.cmdvel_pub = self.create_publisher(Twist, '/cmd_vel',10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.point = 0
        self.v_max = 0.5
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.move2goal)

        self.pose=Pose()
        self.flag=False

    
    def odom_callback(self, odom: Odometry):
       
        self.pose.x = odom.pose.pose.position.x
        self.pose.y = odom.pose.pose.position.y
        self.poset1 = odom.pose.pose.orientation.z
        self.poset2 = odom.pose.pose.orientation.w
        self.pose.theta = atan2(2 * ((self.poset1*self.poset2) + 0),1 - (2*(0+(self.poset1*self.poset1))))
        msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(self.pose.x,self.pose.y,self.pose.theta)
        self.get_logger().info(msg)

    def euclidien_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2) + pow((goal_pose.y - self.pose.y),2))
    

    def liner_vel(self,goal_pose,constant=2):
        return constant*self.euclidien_distance(goal_pose)
    

    def steering_angle(self,goal_pose):
        return atan2(goal_pose.y-self.pose.y,goal_pose.x-self.pose.x)
    
    
    def angular_vel(self,goal_pose,constant=2):
        return constant*(self.steering_angle(goal_pose)-self.pose.theta)


    def move2goal(self):

        positions_x = [-1.0, -0.4,0.2,0.6,1.0]
        positions_y = [-0.5, 0.4,0.4,1.4,0.5]
        theta = [0.0, 0.0, 0.0, 0.0, 0.0]

        goal_pose=Pose()
        
        if self.point <= 4:
            goal_pose.x = positions_x[self.point]
            goal_pose.y = positions_y[self.point]
            goal_pose.theta = theta[self.point]

        distance_tolerance = 0.1
        angular_tolerance = 0.01

        vel_msg=Twist()

        if abs(self.steering_angle(goal_pose)-self.pose.theta)>angular_tolerance:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)
        else:
            vel_msg.angular.z = 0.0
            if self.point > 4:
                vel_msg.linear.x = 0.0
                self.flag = True
            elif self.euclidien_distance(goal_pose)>=distance_tolerance:
                vel_msg.linear.x = self.v_max*self.liner_vel(goal_pose)
            else:
                self.point += 1

        if self.flag:
            vel_msg.angular.z = goal_pose.theta - self.pose.theta
            if abs(goal_pose.theta - self.pose.theta) <= angular_tolerance:
                quit()
        
        self.cmdvel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

from turtle import right
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()

class Tb3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data):
        left_arc = scan_data.ranges[0:16]
        right_arc = scan_data.ranges[-15:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        right = np.array(scan_data.ranges[-45:-91])
        left = np.array(scan_data.ranges[46:91])
        back = np.array(scan_data.ranges[91:271])

        self.min_distance = front_arc.min()
        self.right_min = right.min()
        self.left_min = left.min()
        self.back_min = back.min()


        arc_angles = np.arange(-15, 16)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]

        right_angles = np.arange(-46, -91)
        self.right_object_position = right_angles[np.argmin(right)]

        left_angles = np.arange(46, 91)
        self.left_object_position = left_angles[np.argmin(left)]

        back_angles = np.arange(91, 271)
        self.back_object_position = back_angles[np.argmin(back)]


    def __init__(self):
        self.min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb) 

#!/usr/bin/env python3

from sys import is_finalizing
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math 
import numpy as np
import time
from math import pi
from tf.transformations import euler_from_quaternion
from tb3 import Tb3LaserScan, Tb3Move, Tb3Odometry

def movingForward():
    print('Is moving forward')
    scan_msg = rospy.wait_for_message("/scan", LaserScan)
    frontPoint = scan_msg.ranges[1]
    while frontPoint > 0.35:
            scan_msg = rospy.wait_for_message("/scan", LaserScan)
            frontPoint = scan_msg.ranges[1]
            robot_controller.set_move_cmd(0.25, 0)  
            robot_controller.publish()
    print('stopped moving forward')
    robot_controller.set_move_cmd(0, 0)  
    robot_controller.publish()
        

def turnCW():
    theta_z0 = robot_odometry.yaw
    print('Turning CW')
    turning = True
    while turning:
        if abs(theta_z0 - robot_odometry.yaw) >= 90:
            print('finished CW turn')
            robot_controller.set_move_cmd(0.0, 0.0)
            robot_controller.publish()
            turning = False
        else:
            robot_controller.set_move_cmd(0, -0.3)
            robot_controller.publish()
    scan_msg = rospy.wait_for_message("/scan", LaserScan)
    frontPoint = scan_msg.ranges[1]
    print('forward after turn')
    while frontPoint > 0.35:
            scan_msg = rospy.wait_for_message("/scan", LaserScan)
            frontPoint = scan_msg.ranges[1]
            robot_controller.set_move_cmd(0.25, 0)  
            robot_controller.publish()
    print('stopped moving forward')
    robot_controller.set_move_cmd(0, 0)  
    robot_controller.publish()

def turnCCW():
    theta_z0 = robot_odometry.yaw
    print('Turning CCW')
    turning = True
    while turning:
        if abs(theta_z0 - robot_odometry.yaw) >= 90:
            print('finished CCW turn')
            robot_controller.set_move_cmd(0.0, 0.0)
            robot_controller.publish()
            turning = False
        else:
            robot_controller.set_move_cmd(0, 0.3)
            robot_controller.publish()
    scan_msg = rospy.wait_for_message("/scan", LaserScan)
    frontPoint = scan_msg.ranges[1]
    print('forward after turn')
    while frontPoint > 0.35:
            scan_msg = rospy.wait_for_message("/scan", LaserScan)
            frontPoint = scan_msg.ranges[1]
            robot_controller.set_move_cmd(0.25, 0)  
            robot_controller.publish()
    print('stopped moving forward')
    robot_controller.set_move_cmd(0, 0)  
    robot_controller.publish()

def escapeMaze():
    #initialize the node   
    rospy.init_node('robot_maze', anonymous=True)
    global robot_controller
    robot_controller = Tb3Move()
    global robot_odometry
    robot_odometry = Tb3Odometry()
    global robot_lidar
    robot_lidar = Tb3LaserScan()
    
    while not rospy.is_shutdown():
        no_right_wall = None
        no_front_wall = None
        t0 = rospy.Time.now().to_sec()
        scan_msg = rospy.wait_for_message("/scan", LaserScan)
        front_left = scan_msg.ranges[0:5]
        front_right = scan_msg.ranges[-5:]  
        front = np.array(front_left[::-1] + front_right[::-1])
        frontPoint = scan_msg.ranges[1]
        left = scan_msg.ranges[90]  
        top_left = scan_msg.ranges[45]  
        right = scan_msg.ranges[270]  
        top_right = scan_msg.ranges[315]
        global executingTurn
        if right < 0.5:
            no_right_wall = False
            print('Right wall detected')
        else:
            no_right_wall = True
            print('Right wall not detected')
        if frontPoint < 0.35:
            no_front_wall = False
            print('Front wall detected')
        else:
            no_front_wall = True
            print('Front wall not detected')
        
        if not no_right_wall and no_front_wall:
            movingForward()
        if no_right_wall:
            turnCW()
        if not no_right_wall and not no_front_wall:
            turnCCW()
    rospy.spin()

if __name__ == '__main__':
    escapeMaze()
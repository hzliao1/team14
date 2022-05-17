#!/usr/bin/env python3

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
from tb3 import Tb3Move, Tb3Odometry

def movingForward(robot_controller, forwardSpeed, front):  
    robot_controller.set_move_cmd(forwardSpeed, 0)  
    robot_controller.publish()
    #vel_msg.angular.z = 0 #initialize angular z to zero   
    print('Is moving forward')
    front = 1
    while(front > 0.5):
        scan_msg = rospy.wait_for_message("/scan", LaserScan)
        front = min(min(scan_msg.ranges[0:20]), min(scan_msg.ranges[339:359]))
        robot_controller.publish()  
    print('stopped moving forward')
    robot_controller.set_move_cmd(0, 0)  
    robot_controller.publish()
    return

def turnCW(robot_controller, t0, current_angle, turningSpeed, angle):
    #converting from angle to radian
    angular_speed = round(turningSpeed*2*pi/360, 1)
    #converting from angle to radian
    relative_angle = round((angle*2*pi)/360, 1)
    robot_controller.set_move_cmd(0, -abs(angular_speed))
    front = 0.2
    print('Turning')
    while(current_angle < relative_angle):
        robot_controller.publish()#Publish the velocity  
        #Take actual time to vel calculation
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)#calculates distance
    while(front > 0.1):
        print('forward after turning')
        scan_msg = rospy.wait_for_message("/scan", LaserScan)
        front = scan_msg.ranges[1]
        robot_controller.set_move_cmd(0.2, 0)
        robot_controller.publish()
        if front < 0.5:
            break
    print('stop after turn')
    robot_controller.set_move_cmd(0, 0)  
    robot_controller.publish()
    return

def turnCCW(robot_controller, t0, current_angle, turningSpeed, angle):
    #converting from angle to radian
    angular_speed = round(turningSpeed*2*pi/360, 1)
    #converting from angle to radian
    relative_angle = round(angle*2*pi/360, 1)  
    robot_controller.set_move_cmd(0, abs(angular_speed))
    print('Turning')
    while(current_angle < relative_angle - 0.1):
        robot_controller.publish()#Publish the velocity  
        #Take actual time to vel calculation
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)#calculates distance
    return

def escapeMaze():
    #initialize the node   
    rospy.init_node('robot_cleaner', anonymous=True)
    global robot_controller
    robot_controller = Tb3Move()
    global robot_odometry
    robot_odometry = Tb3Odometry()
    #set topic for publisher
    velocity_publisher = rospy.Publisher('cmd_vel', Twist,queue_size=10)
    vel_msg = Twist()  
    print("Let's move the robot")  
    #define the local speed 
    speed = 0.1
    #set all the linear and angular motion of each dimension to zero        
    vel_msg.linear.x = 0  
    vel_msg.angular.z = 0  
    #Execute the movement when the robot is active
    while not rospy.is_shutdown():
        no_right_wall = None
        no_front_wall = None
        t0 = rospy.Time.now().to_sec()
        scan_msg = rospy.wait_for_message("/scan", LaserScan)
        front = scan_msg.ranges[1]  
        left = scan_msg.ranges[90]  
        top_left = scan_msg.ranges[45]  
        right = scan_msg.ranges[270]  
        top_right = scan_msg.ranges[315]
        if right < 0.85:
            no_right_wall = False
            print('Right wall detected')
        else:
            no_right_wall = True
            print('Right wall not detected')
        if front < 0.5:
            no_front_wall = False
            print('Front wall detected')
        else:
            no_front_wall = True
            print('Front wall not detected')

        if not no_right_wall and no_front_wall:
            movingForward(robot_controller, 0.35, front)
        elif no_right_wall:
            print('need to turn CW')
            turnCW(robot_controller, t0, 0, 15, 90)
        elif not no_right_wall and not no_front_wall:
            turnCCW(robot_controller, t0, 0, 15, 90)
    rospy.spin()

if __name__ == '__main__':
    escapeMaze()
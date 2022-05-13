#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
from asyncore import loop
from random import random
from re import A
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

# Import the tb3 modules from tb3.py
from tb4 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import pi, sqrt, pow
import numpy as np

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]



    def turn_left(self):
        left_distance = self.tb3_lidar.left_min
        right_distance = self.tb3_lidar.right_min
        if left_distance > right_distance:
            return True


    def turn_right(self):
        left_distance = self.tb3_lidar.left_min
        right_distance = self.tb3_lidar.right_min
        if left_distance < right_distance:
            return True

    
    def action_server_launcher(self, goal: SearchGoal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid velocity.  Select a value between 0 and 0.26 m/s.")
            success = False
        if goal.approach_distance <= 0.2:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print(f"Request to move at {goal.fwd_velocity:.3f}m/s "
                f"and stop {goal.approach_distance:.2f}m "
                f"infront of any obstacles")

        # Get the current robot odometry:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:
        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
        
        while self.tb3_lidar.min_distance > goal.approach_distance:
            self.vel_controller.publish()
            #check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the camera sweep.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.vel_controller.stop()  
                success = False
                # exit the loop:
                break
            
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            # populate the feedback message and publish it:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)


        if self.tb3_lidar.closest_object_position > 0 and self.tb3_lidar.back_min > 0.3 and self.turn_right:
            self.vel_controller.set_move_cmd(0.0, -1.2)
            self.vel_controller.publish()
        elif self.tb3_lidar.closest_object_position < 0 and self.tb3_lidar.back_min > 0.3 and self.turn_left:
            self.vel_controller.set_move_cmd(0.0, 1.2)
            self.vel_controller.publish()
        elif self.tb3_lidar.closest_object_position == 0:
            if self.turn_right: 
                self.vel_controller.set_move_cmd(0.0, -1.2)
                self.vel_controller.publish()
            else:
                self.vel_controller.set_move_cmd(0.0, 1.2)
                self.vel_controller.publish()  




        if success:
            rospy.loginfo("approach completed sucessfully.")
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.tb3_lidar.min_distance
            self.result.closest_object_angle = self.tb3_lidar.closest_object_position

            self.actionserver.set_succeeded(self.result)
            # self.vel_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
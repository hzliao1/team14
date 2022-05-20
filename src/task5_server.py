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
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Import some other useful Python Modules
from math import pi, sqrt, pow
import numpy as np
import argparse
from pathlib import Path
from std_msgs.msg import String

path = Path(__file__).parent / "../snaps"
base_image_path = Path(path)



class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        cli = argparse.ArgumentParser(description="Command-line interface for the client node.")
        cli.add_argument("-colour", metavar="COL", type=String, default="Blue", help="The name of a colour (for example)")
        self.args = cli.parse_args(rospy.myargv()[1:])
       
        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        
        self.actionserver.start()
        self.search_colour = str(self.args.colour.data).lower()
        print("THIS IS THE COLOUR:", self.search_colour)
        
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        self.cvbridge_interface = CvBridge()

        self.blue_lower = np.array([115, 224, 100])
        self.blue_upper = np.array([130, 255, 255])
        self.red_lower = np.array([0, 185, 100])
        self.red_upper = np.array([10, 255, 255])
        self.green_lower = np.array([35, 150, 100])
        self.green_higher = np.array([70, 255, 255])
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])

        self.blue_mask = 0
        self.red_mask = 0
        self.green_mask = 0
        self.yellow_mask = 0 
        
        self.beaconFound = 0
        self.img_to_save = None
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
            self.img_to_save = cv_img
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # colour masks
        self.blue_mask = cv2.inRange(hsv_img, self.blue_lower, self.blue_upper)
        self.red_mask = cv2.inRange(hsv_img, self.red_lower, self.red_upper)
        self.green_mask = cv2.inRange(hsv_img, self.green_lower, self.green_higher)
        self.yellow_mask = cv2.inRange(hsv_img, self.yellow_lower, self.yellow_upper)
     
        
        cv2.imshow("img", cv_img)
        cv2.waitKey(1)

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

        print(f"Request to move at {goal.fwd_velocity:.3f}m/s ")

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
                rospy.loginfo("Cancelling...")
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
            self.vel_controller.set_move_cmd(0.0, -1.3)
            self.vel_controller.publish()
        elif self.tb3_lidar.closest_object_position < 0 and self.tb3_lidar.back_min > 0.3 and self.turn_left:
            self.vel_controller.set_move_cmd(0.0, 1.3)
            self.vel_controller.publish()
        elif self.tb3_lidar.closest_object_position == 0:
            if self.turn_right and self.tb3_lidar.back_min > 0.3: 
                self.vel_controller.set_move_cmd(0.0, -1.3)
                self.vel_controller.publish()
            else:
                self.vel_controller.set_move_cmd(0.0, 1.3)
                self.vel_controller.publish()  

        if self.beaconFound != -1:
            #yellow red green blue
            searchBlue = np.sum(self.blue_mask)
            if self.search_colour == "blue" and searchBlue > 0:
                self.beaconFound = 1
            searchRed = np.sum(self.red_mask)
            if self.search_colour == "red" and searchRed > 0:
                self.beaconFound = 1
            searchGreen = np.sum(self.green_mask)
            if self.search_colour == "green" and searchGreen > 0:
                self.beaconFound = 1
            searchYellow = np.sum(self.yellow_mask)
            if self.search_colour == "yellow" and searchYellow > 0:
                self.beaconFound = 1

            if self.beaconFound == 1:
                full_image_path = base_image_path.joinpath('the_beacon.jpg')
                cv2.imwrite(str(full_image_path), self.img_to_save)
                self.beaconFound = -1

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
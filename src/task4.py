#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import time 

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image, LaserScan
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

# Import the tb3 modules (which needs to exist within the "week6_vision" package)
from tb3 import Tb3Move, Tb3LaserScan, Tb3Odometry
# from tb4 import Tb3LaserScan, Tb3Odometry
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sqrt, pow


class colour_search(object):

    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        node_name = "beaconing"
        rospy.init_node(node_name)
        self.startup = True
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.robot_controller = Tb3Move()
        self.robot_odometry = Tb3Odometry()
        self.robot_lidar = Tb3LaserScan()
        self.cvbridge_interface = CvBridge()
        
        #self.rate = rospy.Rate(1) # hz
        self.vel = Twist()
        self.start_colour = ""
        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0
        
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.3
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000

        # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Yellow", "Purple"]
        # self.lower = [(115, 224, 100), (0, 185, 100), (35, 150, 100), (75, 150, 100), (20, 100, 100), (135, 100, 100)]
        # self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (30, 255, 255), (160, 255, 255)]
        # self.lower = {"Blue" : (115, 224, 100), "Red" : (0, 185, 100), "Green" : (35, 150, 100), "Turquoise" : (75, 150, 100), "Yellow" : (20, 100, 100), "Purple" : (135, 100, 100)}
        # self.upper = {"Blue" : (130, 255, 255), "Red" : (10, 255, 255), "Green" : (70, 255, 255), "Turquoise" : (100, 255, 255), "Yellow" : (30, 255, 255), "Purple" : (160, 255, 255)}
        
        self.blue_lower = np.array([115, 224, 100])
        self.blue_upper = np.array([130, 255, 255])
        self.red_lower = np.array([0, 185, 100])
        self.red_upper = np.array([10, 255, 255])
        self.green_lower = np.array([35, 150, 100])
        self.green_higher = np.array([70, 255, 255])
        self.turquoise_lower = np.array([75, 150, 100])
        self.turquoise_upper = np.array([100, 255, 255])
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        self.purple_lower = np.array([135, 100, 100])
        self.purple_upper = np.array([160, 255, 255])

        self.blue_mask = 0
        self.red_mask = 0
        self.green_mask = 0
        self.turquoise_mask = 0
        self.yellow_mask = 0 
        self.purple_mask = 0

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
        #     SearchAction, self.main, auto_start=False)
        # self.actionserver.start()

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)


        # create a single mask to accommodate all six dectection colours:
        # for i in range(6):
        #     if i == 0:
        #         mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        #     else:
        #         mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        # colour = cv2.bitwise_and(hsv_img, hsv_img, mask = mask)

        # m = cv2.moments(mask)

        # colour masks
        self.blue_mask = cv2.inRange(hsv_img, self.blue_lower, self.blue_upper)
        self.red_mask = cv2.inRange(hsv_img, self.red_lower, self.red_upper)
        self.green_mask = cv2.inRange(hsv_img, self.green_lower, self.green_higher)
        self.turquoise_mask = cv2.inRange(hsv_img, self.turquoise_lower, self.turquoise_upper)
        self.yellow_mask = cv2.inRange(hsv_img, self.yellow_lower, self.yellow_upper)
        self.purple_mask = cv2.inRange(hsv_img, self.purple_lower, self.purple_upper)
            
        # self.m00 = m["m00"]
        # self.cy = m["m10"] / (m["m00"] + 1e-5)

        # if self.m00 > self.m00_min:
        #     cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow("cropped image", crop_img)
        cv2.waitKey(1)

    def odom_callback(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        #initialising and storing starting odom readings
        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z
    
    def turn_left(self):
        left_distance = self.robot_lidar.left_arc.min()
        right_distance = self.robot_lidar.right_arc.min()
        if left_distance > right_distance:
            return True


    def turn_right(self):
        left_distance = self.robot_lidar.left_arc.min()
        right_distance = self.robot_lidar.right_arc.min()
        if left_distance < right_distance:
            return True

    def main(self):
        searchInitiated = False
        beaconingInitiated = False
        searchColour = ''
        beaconTarget = 0 # will store current value of the mask we are searching for
        beaconColor = 0 # will store value of mask that needs to be found
        searching = False # flag for searching for beacon
        # beaconFound = True # flag for once beacon is found
        beaconingDone = True # flag for beaconing sequence
        detectingColor = True
        pastHalfway = False
        startTime = time.time()
        while not self.ctrl_c:
            # detect beacon colour
            while detectingColor:
                currentTime = time.time()
                if int(currentTime - startTime) > 22:
                    detectingColor = False
                else:
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                    self.robot_controller.publish()
                
                # if self.theta_z <= (((self.theta_z0 - pi)) - 0.1):
                if int(currentTime - startTime) > 9 and int(currentTime - startTime) < 11:
                    pastHalfway = True
                    if searchColour == '':
                        
                        searchBlue = np.sum(self.blue_mask)
                        if searchBlue > 0:
                            searchColour = 'Blue'
                            beaconTarget = self.blue_mask
                            beaconColor = searchBlue
                        searchRed = np.sum(self.red_mask)
                        if searchRed > 0:
                            searchColour = 'Red'
                            beaconTarget = self.red_mask
                            beaconColor = searchRed
                        searchGreen = np.sum(self.green_mask)
                        if searchGreen > 0:
                            searchColour = 'Green'
                            beaconTarget = self.green_mask
                            beaconColor = searchGreen
                        searchTurquoise = np.sum(self.turquoise_mask)
                        if searchTurquoise > 0:
                            searchColour = 'Turquoise'
                            beaconTarget = self.turquoise_mask
                            beaconColor = searchTurquoise
                        searchYellow = np.sum(self.yellow_mask)
                        if searchYellow > 0:
                            searchColour = 'Yellow'
                            beaconTarget = self.yellow_mask
                            beaconColor = searchYellow
                        searchPurple = np.sum(self.purple_mask)
                        if searchPurple > 0:
                            searchColour = 'Purple'
                            beaconTarget = self.purple_mask
                            beaconColor = searchPurple
            
                if not searchInitiated and searchColour != '':
                    print('SEARCH INITIATED: The target beacon colour is ' + searchColour + '.')
                    #self.robot_controller.set_move_cmd(0.0, 0.0)
                    searchInitiated = True
                    searching = True
                    # beaconFound = False
                    # detectingColor = False

            self.robot_controller.set_move_cmd(0.0, 0.0)

            # search for beacon of colour 'searchColour'
            while searching:
                self.posx0 = self.robot_odometry.posx
                self.posy0 = self.robot_odometry.posy
                
                self.distance = sqrt(pow(self.posx0 - self.robot_odometry.posx, 2) + pow(self.posy0 - self.robot_odometry.posy, 2))
                
                if self.robot_lidar.closest_object_position > 0 and self.turn_right:
                    self.robot_controller.set_move_cmd(0.0, -1.2)
                    self.robot_controller.publish()
                elif self.robot_lidar.closest_object_position < 0 and self.turn_left:
                    self.robot_controller.set_move_cmd(0.0, 1.2)
                    self.robot_controller.publish()
                elif self.robot_lidar.closest_object_position == 0:
                    if self.turn_right: 
                        self.robot_controller.set_move_cmd(0.0, -1.2)
                        self.robot_controller.publish()
                    else:
                        self.robot_controller.set_move_cmd(0.0, 1.2)
                        self.robot_controller.publish()
                else:
                    self.robot_controller.set_move_cmd(0.25, 0.0)
                    self.robot_controller.publish()
                         
                if np.sum(beaconTarget) == beaconColor and int(currentTime - startTime) > 20:
                    print('TARGET DETECTED: Beaconing Initiated.')
                    self.robot_controller.set_move_cmd(0, 0)
                    self.robot_controller.publish()
                    searching = False
                    beaconingInitiated = True

                        
                # initiate beaconing
                # while not beaconingDone:
                #     while self.robot_lidar.min_distance > goal.approach_distance:
                #         self.robot_controller.set_move_cmd(0.25, 0)
                #         self.robot_controller.publish()
                #     self.robot_controller.set_move_cmd(0, 0)
                #     self.robot_controller.publish()
                #     print("BEACONING COMPLETE: The robot has now stopped.")
                #     self.result.total_distance_travelled = self.distance
                #     self.result.closest_object_distance = self.robot_lidar.min_distance
                #     self.result.closest_object_angle = self.robot_lidar.closest_object_position
                #     beaconingDone = True

                # self.actionserver.set_succeeded(self.result)
                # searchEnd = True





            # if beaconTarget > 0 and not beaconingInitiated: 
            #     print('TARGET DETECTED: Beaconing Initiated.')
            #     beaconingInitiated = True


            # if self.stop_counter > 0:
            #     self.stop_counter -= 1

            # if self.m00 > self.m00_min:
            #     # blob detected
            #     if self.cy >= 560-100 and self.cy <= 560+100:
            #         if self.move_rate == "slow":
            #             self.move_rate = "stop"
            #             self.stop_counter = 20
            #     else:
            #         self.move_rate = "slow"
            # else:
            #     self.move_rate = "fast"
                
            # if self.move_rate == "fast":
            #     print(f"MOVING FAST: I can't see anything at the moment (blob size = {self.m00:.0f}), scanning the area...")
            #     self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            # elif self.move_rate == "slow":
            #     print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
            #     self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            # elif self.move_rate == "stop" and self.stop_counter > 0:
            #     print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
            #     self.robot_controller.set_move_cmd(0.0, 0.0)
            # else:
            #     print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
            #     self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
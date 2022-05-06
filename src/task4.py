#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import the tb3 modules (which needs to exist within the "week6_vision" package)
from tb3 import Tb3Move

import numpy as np

class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.start_colour = ""


        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000

        # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Yellow", "Purple"]
        self.lower = [(115, 224, 100), (0, 185, 100), (35, 150, 100), (75, 150, 100), (20, 100, 100), (135, 100, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (30, 255, 255), (160, 255, 255)]
        
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
        for i in range(6):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        colour = cv2.bitwise_and(hsv_img, hsv_img, mask = mask)

        m = cv2.moments(mask)

        # colour masks
        self.blue_mask = cv2.inRange(hsv_img, self.blue_lower, self.blue_upper)
        self.red_mask = cv2.inRange(hsv_img, self.red_lower, self.red_upper)
        self.green_mask = cv2.inRange(hsv_img, self.green_lower, self.green_higher)
        self.turquoise_mask = cv2.inRange(hsv_img, self.turquoise_lower, self.turquoise_upper)
        self.yellow_mask = cv2.inRange(hsv_img, self.yellow_lower, self.yellow_upper)
        self.purple_mask = cv2.inRange(hsv_img, self.purple_lower, self.purple_upper)
            
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow("cropped image", crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            
            self.move_rate = "stop"
            
            # detect colour
            searchColour = ''
            searchBlue = np.sum(self.blue_mask)
            if searchBlue > 0:
                searchColour = 'Blue'
            searchRed = np.sum(self.red_mask)
            if searchRed > 0:
                searchColour = 'Red'
            searchGreen = np.sum(self.green_mask)
            if searchGreen > 0:
                searchColour = 'Green'
            searchTurquoise = np.sum(self.turquoise_mask)
            if searchTurquoise > 0:
                searchColour = 'Turquoise'
            searchYellow = np.sum(self.yellow_mask)
            if searchYellow > 0:
                searchColour = 'Yellow'
            searchPurple = np.sum(self.purple_mask)
            if searchPurple > 0:
                searchColour = 'Purple'
            
            print('SEARCH INITIATED: The target beacon colour is {search_colour}')
            

            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == "slow":
                        self.move_rate = "stop"
                        self.stop_counter = 20
                else:
                    self.move_rate = "slow"
            else:
                self.move_rate = "fast"
                
            if self.move_rate == "fast":
                print(f"MOVING FAST: I can't see anything at the moment (blob size = {self.m00:.0f}), scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == "slow":
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == "stop" and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi

class Publisher():

    def callback_function(self, odom_data):
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
    
    def __init__(self):
        self.node_name = "figure_8"
        self.startup = True
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz
        self.vel = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.pub.publish(Twist())
        self.ctrl_c = True

    def print_data(self, a_message):
        #formats data and prints to console
        print(a_message)
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        print(f"current odometry: x = {self.x:.2f}, y = {self.y:.2f}, theta_z = {(self.theta_z*(180/pi)):.1f}")

    def main_loop(self):
        #output of initial data
        print(f"Initial odometry: x={self.x:.2f}, y = {self.y:.2f}, theta_z = {(self.theta_z*(180/pi)):.1f}")
        #initialising flags
        path_rad = 0.5
        is_past_halfway = False
        status = "First Loop"
        #start of main loop
        while not self.ctrl_c:
            #setting the speed
            lin_vel = pi/30 # m/s

            #setting linear and angular velocities
            self.vel.linear.x = lin_vel # m/s (v)
            self.vel.angular.z = lin_vel / path_rad # rad/s (w)

            #checks to see if robot has met halfway point
            if not is_past_halfway:
                if self.theta_z > (((self.theta_z0 + pi)) - 0.1):
                    is_past_halfway = True
            #if past halfway on first loop, start checking if nearing origin
            if is_past_halfway and status == "First Loop":
                if (self.theta_z >= self.theta_z0 - 0.4 and self.theta_z <= self.theta_z0+0.4):
                    path_rad = -0.5
                    status = "Second Loop"
                    is_past_halfway = False

            #if past halfway on second loop, start checking if nearing origin
            if is_past_halfway and status == "Second Loop":
                if (self.theta_z >= self.theta_z0 - 0.13 and self.theta_z <= self.theta_z0+0.13):
                    #stops robot based on position to origin
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    rospy.signal_shutdown("We are done here!")
                    break
                    #exit(0)
            #output of odom data
            self.pub.publish(self.vel)
            self.print_data(status)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
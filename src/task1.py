#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

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

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z
    
    def __init__(self):
        self.node_name = "figure 8"

        self.startup = True
        self.turn = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.vel = Twist()

                
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.pub.publish(Twist())
        self.ctrl_c = True

    def print_stuff(self, a_message):
        print(a_message)
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")

    def main_loop(self):
        while not self.ctrl_c:
            #publisher_message = f"rospy time is: {rospy.get_time()}"
            path_rad = 1 # m
            lin_vel = 0.2 # m/s

            # v = r * w
            self.vel.linear.x = lin_vel # m/s (v)
            self.vel.angular.z = -(lin_vel / path_rad) # rad/s (w)

            if abs(self.theta_z0 - self.theta_z) >= 2 * pi:
                self.turn = False
                self.vel.linear.x = lin_vel # m/s (v)
                self.vel.angular.z = lin_vel / path_rad # rad/s (w)
            
            self.pub.publish(self.vel)
            self.print_stuff(status)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
#! /usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class action_client(object):
   
    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0
            print(f"FEEDBACK: Currently travelled {self.distance:.3f} m")

    def __init__(self):
        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)
        self.goal = SearchGoal()
        self.client = actionlib.SimpleActionClient("/search_action_server", 
                    SearchAction)


        self.client.wait_for_server()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.distance = 0.0
        self.i = 0

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
        self.ctrl_c = True    
            
    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        while not self.ctrl_c:
            self.send_goal(velocity = 0.25, approach = 0.5)
            prempt = False
            while self.client.get_state() < 2:
                print(f"FEEDBACK: Currently travelled {self.distance:.3f} m, "
                        f"STATE: Current state code is {self.client.get_state()}")
                if self.distance >= 5:
                    rospy.logwarn("Cancelling goal now...")
                    self.client.cancel_goal()
                    rospy.logwarn("Goal Cancelled")
                    prempt = True
                    break

                self.rate.sleep()
            
            self.action_complete = True
            print(f"RESULT: Action State = {self.client.get_state()}")
            if prempt:
                print("RESULT: Action preempted after travelling 2 meters")
            else:
                result = self.client.get_result()
                print(f"RESULT: closest object {result.closest_object_distance:.3f} m away "
                        f"at a location of {result.closest_object_angle:.3f} degrees")

if __name__ == '__main__':
    client_instance = action_client()
    try:
        client_instance.main()
    except rospy.ROSInterruptException:
        pass
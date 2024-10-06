#!/usr/bin/env python
import rospy
from turtlebot3_msgs.msg import RoverActionMsg,RoverStateMsg
from std_msgs.msg import String


class status_handler:
    def __init__(self):
 
        self.action = RoverActionMsg()

    def start(self):
    	self.state_pub = rospy.Publisher("/smach/output", RoverStateMsg, queue_size=1)
        rospy.Subscriber("/smach/input",RoverActionMsg,self.state_callback)

    def state_callback(self,data):
    	self.action.ACTION = data.ACTION

    def publishRoverState(self, state_msg):		
        self.state_pub.publish(state_msg)






        





    



















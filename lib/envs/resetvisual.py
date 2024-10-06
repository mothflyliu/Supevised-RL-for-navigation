import rospy
from turtlebot3_msgs.msg import VisualReset

class ResetVisual():
    def __init__(self,):
        self.visualreset = VisualReset()
        self.pub_visual_reset = rospy.Publisher('visual_reset', VisualReset, queue_size=1)

    def reset(self):
        self.visualreset.reset = True
        self.pub_visual_reset.publish(self.visualreset)
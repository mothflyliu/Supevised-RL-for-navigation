#!/usr/bin/env python
# encoding: utf-8

import rospy
from tf.transformations import euler_from_quaternion
from math import sqrt,pi,pow, atan2
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import RoverStateMsg,RoverActionMsg
from turtlebot3_msgs.msg import VisualReset
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import numpy as np
import time

class Move_Handler():
    def __init__(self):
      self.angular_speed = 1
      self.linear_speed = 1
      self.done =False
      self.state_able = False
      self.stateMsg = RoverStateMsg()
      self.actionMsg = RoverActionMsg()
      self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
      # 重置时将发布消息将行驶轨迹清空

      self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
      self.sub_scan = rospy.Subscriber('scan',LaserScan, self.getState)
      self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
      self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
      self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
      rospy.Subscriber("/smach/output",RoverStateMsg,self.state_callback)
      self.action_pub = rospy.Publisher("/smach/input", RoverActionMsg, queue_size=1)
      # self.do_done = True
      self.rate = rospy.Rate(10)
      while not rospy.is_shutdown():
        if(not self.state_able):
          continue
        print(self.stateMsg.state)
        if( self.stateMsg.state == self.stateMsg.S0):
          print("starting first goal, press & enter s ")
          user_input = raw_input()
          if(user_input == 's' and self.pub_twist((4.5,0.5),0)):
            self.actionMsg.ACTION = self.actionMsg.FIRST
            self.action_pub.publish(self.actionMsg)
            time.sleep(1
            )
        if(self.stateMsg.state == self.stateMsg.S1):
          if(self.pub_twist((4.5,3.5),90)):
            self.actionMsg.ACTION = self.actionMsg.SECOND
            self.action_pub.publish(self.actionMsg)
            self.start_timing()

        if(self.stateMsg.state == self.stateMsg.S2):

          if(self.pub_twist((5.5,3.5),0)):
            self.actionMsg.ACTION = self.actionMsg.THIRD
            self.action_pub.publish(self.actionMsg)
            self.start_timing()
            
        if(self.stateMsg.state == self.stateMsg.S3):
          if(self.pub_twist((5.5,4.5),90)):
            self.actionMsg.ACTION = self.actionMsg.FORTH
            self.action_pub.publish(self.actionMsg)
            self.start_timing()
  
        if(self.stateMsg.state == self.stateMsg.S4):
          if(self.pub_twist((0.5,4.5),180)):
            self.actionMsg.ACTION = self.actionMsg.FIFTH
            self.action_pub.publish(self.actionMsg)
            self.start_timing()       

        if(self.stateMsg.state == self.stateMsg.S5):
          if(self.pub_twist((0.5,0.5),-90)):
            self.actionMsg.ACTION = self.actionMsg.SIXTH
            self.action_pub.publish(self.actionMsg)
            self.start_timing()
            
          self.state_able = False
          self.rate.sleep()

    def start_timing(self):
      self.starting = rospy.Time.now().to_sec()

    def state_callback(self,msg):
      self.stateMsg = msg
      self.state_able = True

    # 实时获取机器人位置
    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw =yaw

    def getState(self, scan):
        scan_range = []
        min_range = 0.13
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0:
          self.done = True

    def reset(self):
      rospy.wait_for_service('gazebo/reset_simulation')
      try:
          self.reset_proxy()

      except (rospy.ServiceException) as e:
          print("gazebo/reset_simulation service call failed")

    def pub_twist(self,state,angle):
        move_cmd = Twist()
        last_rotation = self.yaw
        time.sleep(0.5)  
      # 计算与下个目标点的欧式距离
        distance = sqrt(pow(state[0] - self.position.x, 2) + pow(state[1] - self.position.y, 2))
        while distance > 0.05 and not self.done:
            path_angle = atan2(state[1] - self.position.y, state[0]- self.position.x)
            print("path_angle",path_angle)
            if path_angle < -pi/4 *3 or path_angle > pi/4 *3 :
                if state[1] < 0 and  self.position.y < state[1]:
                    path_angle = -2*pi + path_angle
                elif state[1] >= 0 and  self.position.y > state[1]:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and self.yaw <= 0:
                self.yaw = 2*pi + self.yaw
            elif last_rotation < -pi+0.1 and self.yaw > 0:
                self.yaw = -2*pi + self.yaw
            move_cmd.angular.z = self.angular_speed * path_angle-self.yaw
            print(move_cmd.angular.z)
            distance = sqrt(pow((state[0] - self.position.x), 2) + pow((state[1] -  self.position.y), 2))
            move_cmd.linear.x = min(self.linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = self.yaw
            self.pub_cmd_vel.publish(move_cmd)
            self.rate.sleep()  
        self.pub_cmd_vel.publish(Twist())
        if self.done:
          self.reset()
          return False
        return True

  
if __name__ == '__main__':
  rospy.init_node('robot_move')
  move_handler = Move_Handler()





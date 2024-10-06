#!/usr/bin/env python
# encoding: utf-8
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #
import time
from re import S
import rospy
import numpy as np
# from lib.envs.XMLReader import parse
import math
from math import hypot, sqrt, pow, pi, atan2,hypot
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
# from resetvisual import ResetVisual

class Env():
    def __init__(self, ):
        self.goal_x = 5.5
        self.goal_y = 4.5
        self.angular_speed = 1
        self.linear_speed = 1
        self.min_distance = 0.5
        self.scan_min_range = 0.13
        self.action_yaw = [-0.5*pi, pi,0,0.5*pi]
        self.get_goalbox = False
        self.position = Pose()  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.r = rospy.Rate(10)
        # 机器人方位
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        # 雷达数据
        self.sub_odom = rospy.Subscriber('scan', LaserScan, self.getLaserScan)
        # 重置机器人
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        # self.resetvisual = ResetVisual()
        self.yaw =0
        self.done = False
        self.goal_distance= 0


    def getGoalDistace(self):
        goal_distance  = round(self.goal_x - self.position.x+ self.goal_y - self.position.y, 2)
        return goal_distance

    def getLaserScan(self,scan):
        for i in scan.ranges:
            if i <self.scan_min_range:
                print(i)
                self.done = True

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
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
        # 撞墙，回合结束
        if min_range > min(scan_range) > 0:
            done = True
        # 接近奖励，赢得游戏
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.4:
            self.get_goalbox = True

        return done

    def setReward(self,goal_distance):
        reward = -10*(goal_distance - self.goal_distance)
        # reward = -1
        if self.done:
            rospy.loginfo("Collision!!")
            reward = -50
            self.pub_cmd_vel.publish(Twist())

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.4:
            self.get_goalbox = True

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 100
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
    
        return reward

    def restriction(self,action, state):
        x = [1.5, 2.5,4.5,5.5]
        y = [1.5,2.5,3.5]
        if action == 0:
            if state[1] < 0:
                state[1] += 1
            elif state[1] ==3.5 and state[0] in x:
                state[1] += 1
        elif action == 1 :
            if state[0] <0:
                state[0] += 1
            elif state[0] ==2.5 and state[1] in y:
                state[0] += 1
            elif state[0] ==5.5 and state[1] in y:
                state[0] += 1
        elif action ==2 :
            if state[0] > 7:
                state[0] -= 1
            elif state[0] ==1.5 and state[1] in y:
                state[0] -= 1
            elif state[0] ==4.5 and state[1] in y:
                state[0] -= 1
        elif action == 3 :
            if state[1] >5:
                state[1] -= 1
            elif state[1] ==1.5 and state[0] in x:
                state[1] -= 1
        return state

    def step(self, action, pre_state):
        state = [0,0]
        if action == 0 :
            state[0] = pre_state[0]
            state[1] = pre_state[1]-1
        elif action ==1 :
            state[0] = pre_state[0]-1
            state[1] = pre_state[1]
        elif action ==2 :
            state[0] = pre_state[0]+1
            state[1] = pre_state[1]
        elif action == 3 :
            state[0] = pre_state[0]
            state[1] = pre_state[1]+1
        move_cmd = Twist()  
        last_rotation = self.yaw
        # 计算与下个目标点的欧式距离
        distance = sqrt(pow(state[0] - self.position.x, 2) + pow(state[1] - self.position.y, 2))
        while distance > 0.05 and not self.done:
            path_angle = atan2(state[1] - self.position.y, state[0]- self.position.x)

            if path_angle < -pi/4 or path_angle > pi/4:
                if state[1] < 0 and  self.position.y < state[1]:
                    path_angle = -2*pi + path_angle
                elif state[1] >= 0 and  self.position.y > state[1]:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and self.yaw <= 0:
                self.yaw = 2*pi + self.yaw
            elif last_rotation < -pi+0.1 and self.yaw > 0:
                self.yaw = -2*pi + self.yaw
            move_cmd.angular.z = self.angular_speed * path_angle-self.yaw

            distance = sqrt(pow((state[0] - self.position.x), 2) + pow((state[1] -  self.position.y), 2))
            move_cmd.linear.x = min(self.linear_speed * distance, 0.2)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 2.0)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -2.0)

            last_rotation = self.yaw
            self.pub_cmd_vel.publish(move_cmd)
            self.r.sleep()  
        self.pub_cmd_vel.publish(Twist())
        goal_distance = self.getGoalDistace()
        reward = self.setReward(goal_distance)
        state =  self.restriction(action, state)
        self.goal_distance = goal_distance
        return state, reward, self.done
 
    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
            # self.resetvisual.reset()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        self.goal_x, self.goal_y = self.respawn_goal.getPosition()
        self.get_goalbox = False
        self.done = False
        self.goal_distance = self.getGoalDistace()
        state = [round(self.position.x, 1),round( self.position.y,1)]
        return state


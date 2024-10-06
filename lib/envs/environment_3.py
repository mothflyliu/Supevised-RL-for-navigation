#!/usr/bin/env python
# encoding: utf-8
#################################################################################
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

import rospy
import numpy as np
import math
from lib.envs.XMLReader import parse
from math import  hypot, sqrt, pow, pi, atan2,hypot
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
from resetvisual import ResetVisual


class Env():
    def __init__(self, ):
        self.goal_x = 5.3
        self.goal_y = 4.3
        self.angular_speed = 1
        self.linear_speed = 1
        self.min_distance = 1
        self.action_yaw = [-0.5*pi, pi,0,0.5*pi]
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.r = rospy.Rate(10)
        # 重置机器人
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.respawn_goal = Respawn()
        self.filename = "SCT-Qlearning.xml"
        self.actions, self.ccontrollable,  self.ncontrollable,  self.states,  self.terminal,  self.initial_state,  self.transitions = parse(self.filename)
        self.actual_state = self.initial_state
        self.action = None
        self.reset_visual = ResetVisual()
        #  x , y = 7,5
        # self.x , self.y = 5,5
        self.x , self.y = 7,5
        self.state_tran = np.ones([self.y,self.x], dtype=type)
        for i in range(self.x):
            for j in range(self.y):
                self.state_tran[j][i] = [0.5+1*i, 0.5+1*j]
        self.state_tran = self.state_tran.reshape(1,self.x*self.y)
        self.yaw =0
        self.last_goal_distance= 0


    def getGoalDistace(self):
        goal_distance = round(self.goal_x - self.position.x+ self.goal_y - self.position.y, 2)
        return goal_distance

    # 实时获取机器人位置
    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw =yaw

    def getState(self):
        # scan_range = []
        # min_range = 0.15   # 0.13 -> 0.15
        # done = False

        # for i in range(len(scan.ranges)):
        #     if scan.ranges[i] == float('Inf'):
        #         scan_range.append(3.5)
        #     elif np.isnan(scan.ranges[i]):
        #         scan_range.append(0)
        #     else:
        #         scan_range.append(scan.ranges[i])
        # # 撞墙，回合结束
        # if min_range > min(scan_range) > 0:
        #     done = True
        # 接近奖励，赢得游戏
        done=0
        current_distance = round(hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.6:
            self.get_goalbox = True

        return done

    def setReward(self,goal_distance, done, actual_state):

        reward = -10*(goal_distance - self.last_goal_distance)
        if done:
            rospy.loginfo("Collision!!")
            reward = -50
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
    
        return reward

    def step(self, action, pre_state):
         # 得到fsm当前状态
        for i in range(self.x*self.y):
            if self.state_tran[0][i] == pre_state:
                self.actual_state = i
                break
        # 在规定时间内到达下一个状态停下
        # 得到fsm执行事件后新状态
        for trans in self.transitions:
            if (trans[0] == self.actual_state and trans[2] == action):
                self.actual_state = trans[1]
                break
        state = self.state_tran[0][self.actual_state]
        cnt = 0
        move_cmd = Twist()
        last_rotation = self.yaw

        # data =  rospy.wait_for_message('scan', LaserScan, timeout=5)

        # 计算与下个目标点的欧式距离
        distance = sqrt(pow(state[0] - self.position.x, 2) + pow(state[1] - self.position.y, 2))
        path_angle = atan2(state[1] - self.position.y, state[0]- self.position.x)
        print(path_angle, self.yaw)
        # while (distance > 0.05  and not self.getState(data) ):
        while (distance > 0.15   ):
            
            # cnt = cnt + 1
            # if cnt == 10:
            #     path_angle = atan2(state[1] - self.position.y, state[0]- self.position.x)
            #     cnt = 0
            # # print(path_angle, self.yaw)
            # if self.yaw - path_angle > 0.3:
            #     move_cmd.linear.x = 0
            #     move_cmd.angular.z =-0.4
            # elif self.yaw - path_angle < -0.3:
            #     move_cmd.linear.x = 0
            #     move_cmd.angular.z =0.4
            # else:
            #     move_cmd.linear.x = 0.2
            #     move_cmd.angular.z =0


            # while self.yaw - path_angle < 0.1 :
            #     move_cmd.linear.x = 0
            #     move_cmd.angular.z =0.1
            #     self.pub_cmd_vel.publish(move_cmd)
            # while path_angle < -0.5:
            #     move_cmd.linear.x = 0
            #     move_cmd.angular.z = -0.2
            #     self.pub_cmd_vel.publish(move_cmd)

            path_angle = atan2(state[1] - self.position.y, state[0]- self.position.x)
            if path_angle < -pi/4 *3 or path_angle > pi/4 *3:
                if state[1] < 0 and  self.position.y < state[1]:
                    path_angle = -2*pi + path_angle
                elif state[1] >= 0 and  self.position.y > state[1]:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and self.yaw <= 0:
                self.yaw = 2*pi + self.yaw
            elif last_rotation < -pi+0.1 and self.yaw > 0:
                self.yaw = -2*pi + self.yaw
            move_cmd.angular.z = self.angular_speed * path_angle - self.yaw

            distance = sqrt(pow((state[0] - self.position.x), 2) + pow((state[1] -  self.position.y), 2))
            # move_cmd.linear.x = min(self.linear_speed * distance, 0.15) #min = 0.1
            move_cmd.linear.x = 0.1

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 2.0)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -2.0)

            last_rotation = self.yaw
            self.pub_cmd_vel.publish(move_cmd)
            # data =  rospy.wait_for_message('scan', LaserScan, timeout=5)
            
            self.r.sleep()

        self.pub_cmd_vel.publish(Twist())

        
        # 没有到达则检查是否撞墙
        # data = None
        # while data is None:
        #     try:
        #         data = rospy.wait_for_message('scan', LaserScan, timeout=5)
        #     except:
        #         pass

        done = self.getState()
        # done = 0
        goal_distance = self.getGoalDistace()
        reward = self.setReward(goal_distance,done,self.actual_state)
        self.last_goal_distance = goal_distance
        return state, reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
            self.reset_visual.reset()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")
        self.goal_x, self.goal_y = self.respawn_goal.getPosition()
        self.last_goal_distance = self.getGoalDistace()
        self.get_goalbox = False
        state = [0.5,0.5]
        # state = [round(self.position.x, 1),round( self.position.y,1)]
        # state = [max(math.floor(self.position.x * 10) / 10,0.5)math.floor(self.position.x * 10) / 10,math.floor(self.position.y * 10) / 10]
        return state


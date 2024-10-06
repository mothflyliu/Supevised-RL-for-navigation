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

import rospy
import json
import numpy as np
import pandas as pd
import os
import sys
path = os.path.abspath('./lib/envs')
sys.path.append(path)
from std_msgs.msg import Float32MultiArray
from environment_3 import Env
#from environment_real import Env
dirpath = os.path.dirname(__file__)
import matplotlib.pyplot as plt
from save_data import SaveData

# 离线策略Q-learning
class QLearningTable(object):
    def __init__(self, state_tran, env_transitions,actions=[0,1,2,3],learning_rate=0.01, reward_decay=0.9, e_greedy=0.7): #e_greedy=0.9
        self.actions =  actions
        self.state_tran = state_tran
        self.env_transitions = env_transitions
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        self.actions_list = []
        self.q_table = pd.DataFrame(columns=self.actions, dtype=np.float64)
        self.load =False
        self.load_ep = 0
        self.model_dir = dirpath+"/model_fsm/"+str(self.load_ep)+"b.csv"
        if self.load:
            self.load_model()

    def check_state_exist_fsm(self, state):
        if state not in self.q_table.index:
            # 如果状态在当前的Q表中不存在,将当前状态加入Q表中
            self.q_table = self.q_table.append(
                pd.Series(
                    [0]*len(self.actions_list),
                    index=self.actions_list,
                    name=state,
                )
            )

    def getAction(self, state):
        temp = json.loads(state)
        # temp = state
        # 得到fsm当前状态
        for i in range(35):
        #for i in range(25):
            if self.state_tran[0][i] == temp:
                # 得到fsm状态可能的动作，添加在actions_list中
                for trans in self.env_transitions:
                    if trans[0] == i:
                        self.actions_list.append(trans[2])
                self.actions_list.sort()
                break
        self.check_state_exist_fsm(state)
        if np.random.rand() < self.epsilon  or self.epsilon>0.99:
            # 选择最优行为
            state_action = self.q_table.loc[state, :]
            # 因为一个状态下最优行为可能会有多个,所以在碰到这种情况时,需要随机选择一个行为进行
            state_action = state_action.reindex(np.random.permutation(state_action.index))
            action = state_action.idxmax()
        else:
            action = np.random.choice(self.actions_list)
        self.actions_list = []
        return int(action)

    def save_model(self):
        self.q_table.to_csv(self.model_dir)

    def load_model(self):
        self.q_table = pd.read_csv(self.model_dir,index_col=0,header=0)

    def learn(self, s, a, r, s_):
        temp = json.loads(s_)
        # 得到fsm当前状态
        # for i in range(35):
        for i in range(35):
            if self.state_tran[0][i] == temp:
                # 得到fsm状态可能的动作，添加在actions_list中
                for trans in self.env_transitions:
                    if trans[0] == i:
                        self.actions_list.append(trans[2])
                self.actions_list.sort()
                break
        self.check_state_exist_fsm(s_)
        self.actions_list = []
        q_predict = self.q_table.loc[s, a]
        if s_ != [5.5, 3.5]:
            # 使用公式：Q_target = r+γ  maxQ(s',a')
            q_target = r + self.gamma * self.q_table.loc[s_, :].max()  # next state is not terminal
        else:
            q_target = r

        # 更新公式: Q(s,a)←Q(s,a)+α(r+γ  maxQ(s',a')-Q(s,a))
        self.q_table.loc[s, a] += self.lr * (q_target - q_predict)

def show_plot(result):
    plt.figure(figsize=(15, 4))
    plt.plot(result.keys(), result.values(),
             )
    # label='%s planning steps' % planning_step

    plt.legend()
    plt.xlabel('Episode')
    plt.ylabel('Score')
    plt.title('Q-learning ')

    plt.show()

if __name__ == '__main__':
    rospy.init_node('turtlebot3_stage_fsm')
    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    result = Float32MultiArray()
    get_action = Float32MultiArray()
    env = Env()
    
    for  n in range(1):
        savedata = SaveData(True)
        agent = QLearningTable(env.state_tran, env.transitions)
        # 收敛标记
        flag = False
        EPISODES = 100
        episode_step=500
        N = 65
        # 相似次数
        count = 0
        # 初始化一个随机策略
        policy = []
        # 记录总步数
        step_num = 0
        result1 = {}

        for e in range( EPISODES):
            done = False
            state = env.reset()     
            tmp_policy = []
            score = 0
            for t in range(episode_step):
                action = agent.getAction(str(state))
                state_item = tuple(state)
                tmp_policy.append({state_item:action})
                rospy.loginfo("action=={}".format(action)) 
                next_state, reward, done = env.step(action, state)
                score += reward
                rospy.loginfo("next_state=={},reward=={},socre=={}".format(next_state,reward,score))
                if agent.load:
                    action = str(action)
                if agent.epsilon < 0.99:
                    agent.learn(str(state), action, reward, str(next_state))
                state = next_state
                get_action.data = [action, score, reward]
                pub_get_action.publish(get_action)
                step_num += 1
                #result1[step_num] = score
                #savedata.InsertData({step_num:score})
                if step_num % 5 ==0:
                    agent.save_model()
                    if agent.epsilon < 0.99: 
                            agent.epsilon += 0.006
                if done:
                    agent.save_model()
                    policy = tmp_policy
                    result1[e] = t 
                    break     
                if env.get_goalbox: 
                    result.data = [score, np.max(agent.q_table.values)]
                    pub_result.publish(result)
                    #result1[e] = score 
                    print(tmp_policy)
                    print("*"*50)    
                    agent.save_model()
                    # 如果N次行走的策略相同,表示已经收敛
                    if policy == tmp_policy :
                        count = count + 1
                        print("第{}次走相同策略".format(count))
                        #if count == N:
                        if e >=  65:

                            flag = True
                    else:
                        count = 0
                        policy = tmp_policy
                    result1[e] = t 
                    savedata.InsertData({e:t})
                    break
                # step_num += 1
            if flag:
                print(step_num)
                # print(tmp_policy)
                print(result1)
                show_plot(result1)
                savedata.SaveToTxt()
                break   
    
        savedata.SaveToTxt()
        

    




    


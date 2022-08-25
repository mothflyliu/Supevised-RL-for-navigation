### 描述改项目

### 需要的配置
turtlebot3

### 运行
1. roslaunch turtlebot3_gazebo world1.launch
2. run_this.py or run_notfsm.py


###NOT_FSM修改
1.stem_num计算位置改变
2.在done处添加 policy = tmp_policy，保证碰撞后的策略描述也保存一下，以便判断连续不碰撞的策略3次相同
3.reward函数   reward = -1*(goal_distance - self.lgoal_distance)
4.e_greedy=0.6

1.roslaunch turtlebot3_gazebo world_smach.launch 
2.2. vscode 运行 smach_machine
3.3.vscode 运行 robot_move.py

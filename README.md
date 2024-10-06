### Description for program
This project is an optimal control algorithm based on supervised control theory & reinforcement learning

### Configuration
turtlebot3 simulation 
gazebo
rviz

### Run
1. roslaunch turtlebot3_gazebo world1.launch
2. run_this.py        ### run this for SCT-RL method Without probability considering
3. run_notfsm.py      ### run this for RL method without SCT
4. run_notfsm_prob.py ### run this for SCT-RL method With probability considering

### Running the robot based on the ROS SMACH module after SCT systhesizing.
1.roslaunch turtlebot3_gazebo world_smach.launch 
2.state_machine.py
3.robot_move.py

### Gazebo world description
Workspace : The workspace of the robot is assumed to be a 5-by-7 grid area with 35 states. Consider that the autonomous delivery robot works in the workspace, where the state of the positions belongs to one of the following six types: Start, Goal, Barrier (or wall), Slop, Corner and Normal. Given the goal position, the robot should learn from the Start state to achieve the Goal state quickly and accurately. Upon the wall or barrier state avoiding, we focus on the uncertain situation with Slop and Corner states. Note that the robot might slip uncontrollably to the wall due to slippery surfaces in Slop area which will make the uncertain action, and high blocking probability in Corner state may take long time for the robot to cross this area.

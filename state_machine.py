#!/usr/bin/env python


### THIS IS THE STATE MACHINE FOR ITU ROVER


import rospy
import smach
import smach_ros
from std_msgs.msg import String
from status_handler import status_handler    
from turtlebot3_msgs.msg import RoverStateMsg,RoverActionMsg
 
status_handler = status_handler()
actionMsg = RoverActionMsg()

 
        
class S0(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['first','run_into_wall','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S0      
    status_handler.publishRoverState(self.stateMsg)
    if(status_handler.action.ACTION  ==  actionMsg.FIRST):      
      return 'first'
    if(status_handler.action.ACTION  ==  actionMsg.RUN_INTO_WALL):      
      return 'run_into_wall'
    return 'repeat'

class S1(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['second','run_into_wall','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S1      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.SECOND):    
      return 'second'
    if (status_handler.action.ACTION ==  actionMsg.RUN_INTO_WALL):    
      return 'run_into_wall'
    return 'repeat'

class S2(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['third','run_into_wall','repeat'])
    self.stateMsg = RoverStateMsg()
  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S2      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.THIRD):    
      return 'third'
    if (status_handler.action.ACTION ==  actionMsg.RUN_INTO_WALL):    
      return 'run_into_wall'
    return 'repeat'

class S3(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['forth','collision2','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S3      
    status_handler.publishRoverState(self.stateMsg)
    if(status_handler.action.ACTION ==  actionMsg.FORTH):      
      return 'forth'
    if(status_handler.action.ACTION ==  actionMsg.COLLISION2):      
      return 'collision2'
    return 'repeat'

class S4(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['fifth','collision2','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S4      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.FIFTH):    
      return 'fifth'
    if (status_handler.action.ACTION ==  actionMsg.COLLISION2):    
      return 'collision2'
    return 'repeat'

class S5(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['sixth','collision3','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S5      
    status_handler.publishRoverState(self.stateMsg)
    if(status_handler.action.ACTION ==  actionMsg.SIXTH):      
      return 'sixth'
    if (status_handler.action.ACTION ==  actionMsg.COLLISION3):    
      return 'collison3'
    return 'repeat'

class S6(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['seventh','collision3','repeat'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S6      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION ==  actionMsg.SEVENTH):    
      return 'seventh'
    if (status_handler.action.ACTION  ==  actionMsg.COLLISION3):    
      return 'collision3'
    return 'repeat'

class S7(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['eighth','collision4','repeat'])
    self.actionMsg = status_handler.action
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.S7      
    status_handler.publishRoverState(self.stateMsg)  
    if(status_handler.action.ACTION  ==  self.actionMsg.EIGHTH):    
      return 'eighth'
    if (status_handler.action.ACTION  ==  self.actionMsg.COLLISION4):    
      return 'collision4'
    return 'repeat'

class FALL1(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['reset'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.FALL1      
    status_handler.publishRoverState(self.stateMsg)
    if (status_handler.action.ACTION  ==  self.actionMsg.RESET):    
      return 'reset'

class FALL2(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['reset2'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.FALL2      
    status_handler.publishRoverState(self.stateMsg)
    if (status_handler.action.ACTION  ==  self.actionMsg.RESET2):    
      return 'reset2'

class FALL3(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['reset3'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.FALL3      
    status_handler.publishRoverState(self.stateMsg)
    if (status_handler.action.ACTION  ==  self.actionMsg.RESET3):    
      return 'reset3'

class FALL4(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['reset4'])
    self.stateMsg = RoverStateMsg()

  def execute(self, userdata):
    self.stateMsg.state = self.stateMsg.FALL4     
    status_handler.publishRoverState(self.stateMsg)
    if (status_handler.action.ACTION  ==  self.actionMsg.RESET4):    
      return 'reset4'

#####################################################################################################################################################
        



def CreateStateMachine():

    #Create the state machine
    sm_rover = smach.StateMachine(outcomes=['DEAD'])
    #Open the container
    with sm_rover:

        smach.StateMachine.add('S0', S0(),
                               transitions={'first': 'S1','run_into_wall':'RUN_INTO_WALL','repeat':'S0', })        
        smach.StateMachine.add('RUN_INTO_WALL', FALL1(), 
                               transitions={'reset':'S0' })

        smach.StateMachine.add('COLLISION2', FALL2(), 
                               transitions={'reset2':'S3' })
        smach.StateMachine.add('COLLISION3', FALL3(), 
                               transitions={'reset3':'S5' })

        smach.StateMachine.add('S1', S1(), 
                               transitions={'second':'S2', 'run_into_wall':'RUN_INTO_WALL','repeat':'S1'})

        smach.StateMachine.add('S2', S2(), 
                               transitions={'third':'S3', 'run_into_wall':'RUN_INTO_WALL','repeat':'S2'})     

        smach.StateMachine.add('S3', S3(), 
                               transitions={'forth':'S4', 'collision2':'COLLISION2','repeat':'S3'})     

        smach.StateMachine.add('S4', S4(), 
                               transitions={'fifth':'S5', 'collision2':'COLLISION2','repeat':'S4'})     

        smach.StateMachine.add('S5', S5(), 
                               transitions={'sixth':'S6', 'collision3':'COLLISION3','repeat':'S5'})     

        smach.StateMachine.add('S6', S6(), 
                               transitions={'seventh':'S7', 'collision3':'COLLISION3','repeat':'S6'})     

        smach.StateMachine.add('S7', S7(), 
                               transitions={'eighth':'S0', 'collision4':'COLLISION4','repeat':'S7'})     

        smach.StateMachine.add('COLLISION4', FALL4(), 
                               transitions={'reset4':'S7'
                               })
        #Codes for smach viewer
        sis = smach_ros.IntrospectionServer('rover_state_machine', sm_rover, '/ROVER_SM_ROOT')
        sis.start()

        outcome = sm_rover.execute()
        sis.stop()

def main():
    #Init,pubs and subs
    rospy.init_node("state_machine")
    global status_handler
    status_handler.start()
    CreateStateMachine()


if __name__ == '__main__':
    main()

#!/usr/bin/env python
"""
Created on Sun Jun 15 14:51:20 2014

@author: ew
"""

import roslib; roslib.load_manifest('ubot_dmp')
import rospy
import actionlib

import ubot_dmp.msg as ubot_dmp_msgs
import ubot_msgs.msg as ubot_msg

import dmp.lib_manager as LibraryManager

import sys, traceback
import numpy as np


class DMPServer(object) :
    def __init__(self, name):
        self.name = name
        self.states = ['WAIT','START', 'COMPUTE', 'EXECUTE', 'DONE']
        self.current_state = 0
        self._last_state = 0
        self.current_positions = None
        
        self.last_time = rospy.Time.now()
        
        self.lib_mgr = LibraryManager.DMPLibraryManager('./dmp/library.yaml', './dmp/pickles/')
        print 'Library Sections : ', self.lib_mgr.listSections()
        #self.lib_mgr.loadEntireLibrary()
        self.lib_mgr.loadSection('Test')
        
        self.active_dmp = None
        
        self.server = actionlib.SimpleActionServer(self.name, ubot_dmp_msgs.DMPAction, None, False)
        self.server.register_goal_callback(self.goal_callback)
        self.server.register_preempt_callback(self.preempt)
        
        rospy.Subscriber("/uBot/joint_positions", ubot_msg.JointPositions, self.joint_position_callback)
        self._joint_goal_pub = rospy.Publisher("/uBot/set_joint_positions", ubot_msg.SetJointPositions)
        self._joint_vel_goal_pub = rospy.Publisher("/uBot/set_joint_velocities", ubot_msg.SetJointVelocities)
        
        self.server.start()
        print 'DMP Server started'
        
    def reset(self):
        self.active_dmp = None
        self.current_state = 0;
        self._perc_complete = 0.0
        
    def joint_position_callback(self, msg):
        self.current_positions = np.array(msg.positions)
        
    def goal_callback(self):
        rospy.loginfo('Received new goal')
        self.goal = self.server.accept_new_goal()
        self.current_state = 1; # start state
    
    """
    Send a failure message and reset class
    """
    def preempt(self):
        rospy.logwarn('Preempting %s server', self.name)
        result_msg = ubot_dmp_msgs.DMPResult()
        result_msg.success = False
        self.server.set_preempted(result_msg)
        self.reset()
    
    """
    Send a success message and reset class
    """
    def setSuccess(self):
        rospy.loginfo('DMP %s server completed successfully.', self.name)
        result_msg = ubot_dmp_msgs.DMPResult()
        result_msg.success = True
        self.server.set_succeeded(result_msg)
        self.reset()
        
    def sendFeedback(self):
        feedback_msg = ubot_dmp_msgs.DMPFeedback()
        feedback_msg.percent_complete = self._perc_complete
        self.server.publish_feedback(feedback_msg)
        
    def send_joint_commands(self):
        msg = ubot_msg.SetJointPositions()
        msg.positions = np.zeros(12)
        msg.positions[self.active_joint_mask] = self.y_track
        msg.bitmask = self.goal.bitmask
        msg.numJoints = 12
        
        self._joint_goal_pub.publish(msg)
    
    """
    Simple wrapper func for the state machine
    """
    def run(self):
        if self.current_state and self.current_positions is not None:
            self.state_machine()
            
    def state_machine(self):
        if self._last_state is not self.current_state:
            self._print_current_state()
            
        if self.current_state == 1:
            self.start_state()
        
        if self.current_state == 2:
            self.compute_state()
        
        if self.current_state == 3:
            self.execute_state()
            
        if self.current_state == 4:
            self.setSuccess()
            
    """
    Retrieve the DMP from the library if available. If not, then log error and preempt
    """
    def start_state(self):
        dmp_lib_id = self.lib_mgr._to_lib_id(self.goal.section, self.goal.shelf, self.goal.dmp_id)
        rospy.loginfo('Retreiving DMP from library : %s', dmp_lib_id) 
        
        self.active_dmp = self.lib_mgr.getDMP(dmp_lib_id)
        
        if self.active_dmp is None:
            rospy.logerr("Could not find dmp by library id %s", dmp_lib_id)
            self.preempt()
            return
        
        # run through the selection mask to get the appropriate joint goals 
        selectMask = 1
        msg_goals = []
        self.active_joint_mask = []
        for i in range(12):
            if self.goal.bitmask & selectMask:
                msg_goals.append(self.goal.goal_pos[i])
                self.active_joint_mask.append(i)
            
            selectMask <<= 1
            
#        if len(self.active_dmp.goal) == len(msg_goals):
#            self.active_dmp.goal = np.array(msg_goals)
#        else:
#            rospy.logerr("Goal mask did not match DMP")
#            self.preempt()
#            return

        self.active_dmp.reset_state() # make sure dmp starts fresh
        self._start_positions = self.current_positions[self.active_joint_mask]

        self.last_time = rospy.Time.now()
        self.current_state = 2; # compute state
        
    def compute_state(self):
        now = rospy.Time.now()
        dt = now.to_sec() - self.last_time.to_sec()
        self.last_time = now
        
        state_fb = self.current_positions[self.active_joint_mask] - self._start_positions
#        err_fb = state_fb - self.active_dmp.goal  
#        
#        for i in range(len(err_fb)):
#            if err_fb[i] > np.pi:
#                err_fb[i] += -np.pi
#            if err_fb[i] < -np.pi:
#                err_fb[i] += np.pi
#        
#        max_err = np.max(np.abs(err_fb))
#        
        #state_fb= None
        self.y_track, self.dy_track, self.ddy_track = self.active_dmp.step(tau=self.goal.tau, state_fb=state_fb, dt=dt)
        
        self.y_track = self.y_track + self._start_positions
        
#        self.dy_track = self.dy_track + self._start_velocities
#        self.ddy_track = self.ddy_track + self._start_accs

        self._perc_complete = -np.log(self.active_dmp.cs.x)
        
        if (self._perc_complete > 0.99):
            self.current_state = 4 # DONE
            return
        
        rospy.logdebug('Canonical System X : %f',self.active_dmp.cs.x)
        rospy.logdebug('y_track, dy_track, ddy_track : %f, %f, %f', self.y_track, self.dy_track, self.ddy_track)
        
        self.current_state = 3; # execute state
        
    def execute_state(self):
        self.sendFeedback()
        self.send_joint_commands()
        self.current_state = 2; # compute state
    
    def _print_current_state(self):
        rospy.logdebug("Current State : %s", self.states[self.current_state])

if __name__ == '__main__' :
    
    rospy.init_node('ubot_dmp_server')
    rospy.loginfo('Starting DMP server as %s', 'ubot_dmp_server')
    dmp_server = DMPServer('DMPServer')    
    
    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        try:
            dmp_server.run()
            r.sleep()
        except Exception:
            rospy.logerr('Exception Caught - Exiting Node')
            dmp_server.preempt()
            ex_type, ex, tb = sys.exc_info()
            print ex_type, ex
            traceback.print_tb(tb)
            rospy.signal_shutdown('Exception termination')
            

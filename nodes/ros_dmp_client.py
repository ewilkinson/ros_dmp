#!/usr/bin/env python
"""
Created on Wed Jun 18 20:43:39 2014

@author: ew
"""

import roslib; roslib.load_manifest('ubot_dmp')
import rospy
import actionlib

import ubot_dmp.msg as ubot_dmp_msgs
import ubot_msgs.msg as ubot_msg

import numpy as np

def feedback_cb(feedback_msg):
    print feedback_msg

if __name__ == '__main__' :
    
    rospy.init_node('ubot_dmp_client')
    rospy.loginfo('Starting DMP client as %s', 'ubot_dmp_client')
    client = actionlib.SimpleActionClient('DMPServer', ubot_dmp_msgs.DMPAction)
    client.wait_for_server()
    rospy.loginfo("Connected to server")
    
    goal = ubot_dmp_msgs.DMPGoal()
    
    goal.tau = 2.0
    goal.dmp_id = 'do_the_twist'
    goal.section = 'Test'
    goal.shelf = 'Test'
    goal.goal_pos = np.ones(12).tolist() # this doesn't matter right now
    
    goal.bitmask = 0b11111111000
    
    client.send_goal(goal, feedback_cb=feedback_cb)
    
    client.wait_for_result()
    
    print client.get_result()
    
    rospy.spin()
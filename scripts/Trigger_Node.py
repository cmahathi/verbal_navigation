#!/usr/bin/env python

import rospy
from verbal_navigation.msg import *
import os
import easygui 

def wait_for_trigger():
    rospy.init_node('Trigger_Node')
    pub = rospy.Publisher('/robot_plan', Robot_Action, queue_size=10)
    easygui.msgbox(msg='Press space to continue', title=' ', ok_button='OK', image=None, root=None)

    msg = Robot_Action()
    msg.robot_id = "bender"
    msg.action_type = "T"
    msg.instructions = "this is a test"
    pub.publish(msg)

if __name__ == "__main__":
    wait_for_trigger()

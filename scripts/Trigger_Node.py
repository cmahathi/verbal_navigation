#!/usr/bin/env python

import rospy
from verbal_navigation.msg import *
import os
import easygui 

messageQueue = []

def wait_for_trigger():
    pub = rospy.Publisher('/robot_plan_pregen', Robot_Action, queue_size=10)
    easygui.msgbox(msg='Press space to continue', title=' ', ok_button='OK', image=None, root=None)

    # msg = Robot_Action()
    # msg.robot_id = "bender"
    # msg.action_type = "T"
    # msg.instructions = "this is a test"
    # pub.publish(msg)
    print len(messageQueue)
    for msg in messageQueue:
        pub.publish(msg)

def handle_msg(msg):
    messageQueue.append(msg)


if __name__ == "__main__":
    rospy.init_node('Trigger_Node')
    sub = rospy.Subscriber('/robot_plan', Robot_Action, handle_msg)
    wait_for_trigger()

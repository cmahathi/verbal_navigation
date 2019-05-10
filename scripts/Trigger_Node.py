#!/usr/bin/env python

import rospy
from verbal_navigation.msg import *
import easygui 
import rosbag

messageQueue = []

def wait_for_trigger():
    pub = rospy.Publisher('/robot_plan', Robot_Action, queue_size=10)
    easygui.msgbox(msg='Press space to continue', title=' ', ok_button='OK', image=None, root=None)

    for msg in messageQueue:
        pub.publish(msg)
    

if __name__ == "__main__":
    rospy.init_node('Trigger_Node')


    bag = rosbag.Bag('robot_plan_final.bag')
    for topic, msg, t in bag.read_messages(topics=['/robot_plan']):
        messageQueue.append(msg)
    bag.close()

    wait_for_trigger()

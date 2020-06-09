#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from CONSTANTS import *


def callback(msg):
    print(msg.data)

rospy.init_node('topic_sub')

sub = rospy.Subscriber('counter', Int32, callback)

rospy.spin()
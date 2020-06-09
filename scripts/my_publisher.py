#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from CONSTANTS import *

# from scripts.CONSTANTS import *

rospy.init_node('topic_publisher')

pub = rospy.Publisher('counter', Int32, latch=True, queue_size=10)
rate = rospy.Rate(1)  # 2 Hz

count = 0

while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()

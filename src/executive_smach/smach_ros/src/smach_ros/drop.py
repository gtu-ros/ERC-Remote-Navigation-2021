#!/usr/bin/env python

import roslib
import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations
import smach
import smach_ros
from enum import Enum
import time
from std_msgs.msg import Empty

rospy.init_node('dropper')
pub = rospy.Publisher('/probe_deployment_unit/drop', Empty, queue_size=1)
#rate = rospy.Rate(10)
while pub.get_num_connections() < 1:
    pass

msg = Empty()
pub.publish(msg)

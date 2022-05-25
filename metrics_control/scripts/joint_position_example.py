#!/usr/bin/env python3

import time
import rospy
from cobot_msgs.srv import *


if __name__ == '__main__':
	rospy.init_node('joint_position_example')

	time.sleep(10)
	move_to_joint_target = rospy.ServiceProxy('/take_action', TakeAction)
	backplate_approach_pose = [0.6813842747504251, 0.3863561676158779, -0.15252376738556644, -1.7943590762326294, 0.06897386904405396, 2.1883793025301634, -0.24329813243779871]
	
	req = TakeActionRequest()
	req.q = backplate_approach_pose
	move_to_joint_target(req)

	rospy.spin()
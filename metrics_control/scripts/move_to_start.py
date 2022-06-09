#!/usr/bin/env python3


import time
import rospy
import math
from cobot_msgs.msg import *
from cobot_msgs.srv import *

if __name__ == '__main__':
	try:
		rospy.init_node('metrics_assembly')

		time.sleep(5)

		move_to_joint_target = rospy.ServiceProxy('/take_action', TakeAction)
		cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)

		ready_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, 0.25 * math.pi]
		req = TakeActionRequest()
		req.q = ready_pose
		move_to_joint_target(req)
		time.sleep(2)
		cartesian_action_service_2D(pose=[0.48, 0.00], slow=False)		
	
	except rospy.ROSInterruptException:
		pass
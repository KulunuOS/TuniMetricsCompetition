#!/usr/bin/env python3


import time
import rospy
import math
from cobot_msgs.msg import *
from cobot_msgs.srv import *

"""

def callback(data):

	if data.id == 38 and data.score > 0.30:

		print("Request received, going down.")
		cartesian_action_service_1D(z_pose=0.3)
	elif data.id == 39 and data.score > 0.30:

		move_to_joint_target(req)
		time.sleep(1)
		cartesian_action_service_2D(pose=[0.48, 0.00], slow=False)	
"""



if __name__ == '__main__':
	try:
		rospy.init_node('opendr_test')
		print("Opendr test node has started.")

		rospy.wait_for_service('/opendr/image_pose_annotated')
		print("Skeleton action detected waiting for a command.")

		#rospy.Subscriber("/opendr/skeleton_based_action_recognition", ObjectHypothesis, callback)


		time.sleep(1)


		detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
		cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
		cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
		move_gripper_service = rospy.ServiceProxy("move_gripper", MoveGripper)
		move_to_joint_target = rospy.ServiceProxy('/take_action', TakeAction)


	

		ready_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, 0.25 * math.pi]
		req = TakeActionRequest()
		req.q = ready_pose


		"""
		cartesian_action_service_2D(pose=[0.3, 0.3])

		cartesian_action_service_1D(z_pose=0.40)
		"""
		#rospy.spin()



	except rospy.ROSInterruptException:
		pass 
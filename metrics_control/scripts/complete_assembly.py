#!/usr/bin/env python3

import time
import rospy
from cobot_msgs.msg import *
from cobot_msgs.srv import *



def main():


	detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
	
	rotate_ee_service = rospy.ServiceProxy("/rotate_ee", RotateEE)
	cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
	cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
	move_gripper_service = rospy.ServiceProxy("move_gripper", MoveGripper)

	grasping_service = rospy.ServiceProxy("grasp", Grasp)

	def grasp(x, y):
		cartesian_action_service_2D(pose=[x, y])
		time.sleep(3)
		#rotate_ee_service(angle=detection.quaternion)
		time.sleep(3)
		cartesian_action_service_1D(z_pose=0.205)

		# grasping_service(width=0.048, force=20.0)
		grasping_service(width=0.05, force=20.0)
		time.sleep(3)
		cartesian_action_service_1D(z_pose=0.40)

	# Make sure the gripper is working properly
	try:
		move_gripper_service(20.0, 0.02) 
		move_gripper_service(20.0, 0.08) 
	except rospy.ServiceException as exc:
		print("Gripper not operational")
	else:

 		#####################################################################
		#						Beginning of the task 						#
		#####################################################################

		gear_detection = False
		bottom_casing_detection = False

		while not (gear_detection and bottom_casing_detection):
			objects = detection_client()

			for obj in objects.detection.detections:
				if obj.obj_class == 8:
					gear_detection = obj
				if obj.obj_class != 8:
					bottom_casing_detection = obj

			print("No detection")
			time.sleep(1)

		grasp(gear_detection.x, gear_detection.y)

		# Bring them above the back plate

		cartesian_action_service_2D(pose=[bottom_casing_detection.x, bottom_casing_detection.y])
		time.sleep(3)

		# insert the gears 
		cartesian_action_service_1D(z_pose=0.215)

	

if __name__ == '__main__':
	rospy.init_node('metrics_assembly')
	time.sleep(3)
	main()
	rospy.spin()
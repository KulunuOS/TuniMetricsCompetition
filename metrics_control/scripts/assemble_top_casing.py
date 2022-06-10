#!/usr/bin/env python3

import time
import rospy
import math
from cobot_msgs.msg import *
from cobot_msgs.srv import *

"""
This script assembles the top_casing on already assembled left_gear + right_gear + bottom_casing

There are two ways to assemble the top_casing

(A) top_casing_in_hand() : Pick the top_casing first and push down on semi assembled gears + bottom casing
(b) assembled_gears_in_hand() : Pick the semi_assembled gears + bottom casing and push it down on top_casing

"""


if __name__ == '__main__':
	try:
		rospy.init_node('metrics_assembly2')

		time.sleep(5)

		detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
		
		rotate_ee_service = rospy.ServiceProxy("/rotate_ee", RotateEE)
		cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
		cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
		move_gripper_service = rospy.ServiceProxy("move_gripper", MoveGripper)
		
		move_to_joint_target = rospy.ServiceProxy('/take_action', TakeAction)
	
		
		grasping_service = rospy.ServiceProxy("grasp", Grasp)

		try:
			move_gripper_service(20.0, 0.02) 
			time.sleep(1)
			move_gripper_service(20.0, 0.08) 
		except rospy.ServiceException as exc:
			print("Gripper not operational")
		else:

			input("If the robot is in the start position type enter to grasp. Otherwise - rosrun metrics_control move_to_start.py")

			

			# Looking for 2 detections
			# 1. Detecting the inverted top casing inv
			# 2. Detecting the assembled gears
				
			casing_detection = False 
			bottom_detection = False 

			while not (casing_detection and  bottom_detection):
				objects = detection_client()
				for obj in objects.detection.detections:
					if obj.obj_class != 9:
						print("Top casing inv found")
						casing_detection = obj
					if obj.obj_class == 9:
						bottom_detection = obj 					
				print("Cannot locate the objects. Trying again.")
				time.sleep(1)


			#print(detection)
			
			#print("detections : " , detections)
			print ('Picking the top casing')

			cartesian_action_service_2D(pose=[casing_detection.x, casing_detection.y])
			time.sleep(3)
			rotate_ee_service(angle= casing_detection.angle)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.3)
			cartesian_action_service_1D(z_pose=0.20133,slow=True)
			grasping_service(width=0.0019, force=20.0)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.45)

			# MOVE ON TOP Bottom_casing + gears assembly


			print ('Moving on to top casing')
			cartesian_action_service_2D(pose=[bottom_detection.x+0.004, bottom_detection.y+0.00385])
			time.sleep(3)
			rotate_ee_service(angle= bottom_detection.angle)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.21375, slow=True)
			move_gripper_service(20.0, 0.8)				
	

			cartesian_action_service_1D(z_pose=0.30)
			move_gripper_service(20.0, 0.0)	
			cartesian_action_service_1D(z_pose=0.2095, slow=True)
			cartesian_action_service_1D(z_pose=0.45)
			move_gripper_service(20.0, 0.8)	


			

	except rospy.ROSInterruptException:
		pass
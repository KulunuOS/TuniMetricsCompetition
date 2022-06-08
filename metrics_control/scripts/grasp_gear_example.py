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

			objects = detection_client()

			detection = False 

			while not detection:
				for obj in objects.detection.detections:
					if obj.obj_class == 8 or obj.obj_class == 1:
						detection = obj
				print("Cannot see a gear. Trying again.")
				time.sleep(1)

			print(detection)	
			
			cartesian_action_service_2D(pose=[detection.x, detection.y])
			time.sleep(3)
			rotate_ee_service(angle=detection.angle)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.201)

			grasping_service(width=0.003, force=20.0)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.40)
		

	except rospy.ROSInterruptException:
		pass
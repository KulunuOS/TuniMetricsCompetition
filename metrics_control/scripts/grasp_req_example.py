#!/usr/bin/env python3


import time
import rospy
from cobot_msgs.msg import *
from cobot_msgs.srv import *

if __name__ == '__main__':
	rospy.init_node('metrics_assembly')

	time.sleep(10)

	detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
	
	cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
	cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
	move_gripper_service = rospy.ServiceProxy("move_gripper", MoveGripper)
	rotate_ee_service = rospy.ServiceProxy("/rotate_ee", RotateEE)

	grasping_service = rospy.ServiceProxy("grasp", Grasp)

	move_gripper_service(20.0, 0.08) 

	time.sleep(2)

	objects = detection_client()
	print("object_detected")
	print(objects)
	detection = objects.detection.detections[0]
	print(detection)	
	
	cartesian_action_service_2D(pose=[detection.x, detection.x])
	time.sleep(3)
	rotate_ee_service(angle=detection.quaternion)
	time.sleep(3)
	cartesian_action_service_1D(z_pose=0.201)

	grasping_service(width=0.003, force=20.0)
	time.sleep(3)
	cartesian_action_service_1D(z_pose=0.40)

	rospy.spin()
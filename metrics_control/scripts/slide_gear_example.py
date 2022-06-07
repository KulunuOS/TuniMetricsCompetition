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

			gears_detection = []

			while (len(gears_detection) < 2 or len(gears_detection) > 2):
				objects = detection_client()
				print(objects)
				gears_detection = []
				for obj in objects.detection.detections:
					if obj.obj_class == 1:
						gears_detection.append(obj)
				if (len(gears_detection) < 2):
					print("Cannot see two gears.")
				elif (len(gears_detection) > 2):
					print("Too many gears on the table.")
				time.sleep(2)

			obj1 = gears_detection[0]
			obj2 = gears_detection[1]
			dist = math.sqrt((obj1.x - obj2.x)**2 + (obj1.y - obj2.y)**2)
			print("dist(gear1 - gear2) = {}".format(dist))
			
			move_gripper_service(20.0, 0.0)	
				
			if (obj1.y < obj2.y): # second object is on the right
				cartesian_action_service_2D(pose=[obj1.x, obj1.y+0.04])
				time.sleep(3)
				cartesian_action_service_1D(z_pose=0.201)
				time.sleep(3)
				cartesian_action_service_2D(pose=[obj2.x, obj2.y+0.01])
		
		
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
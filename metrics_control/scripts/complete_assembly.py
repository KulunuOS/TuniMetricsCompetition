#!/usr/bin/env python3

import time
import rospy
import math
from cobot_msgs.msg import *
from cobot_msgs.srv import *



def main():


	detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
	
	rotate_ee_service = rospy.ServiceProxy("/rotate_ee", RotateEE)
	cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
	cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
	move_gripper_service = rospy.ServiceProxy("move_gripper", MoveGripper)
	move_to_joint_target = rospy.ServiceProxy('/take_action', TakeAction)
	grasping_service = rospy.ServiceProxy("grasp", Grasp)

	def grasp(x, y, angle, high=False):
		cartesian_action_service_2D(pose=[x, y])
		time.sleep(3)
		rotate_ee_service(angle=angle)
		time.sleep(3)
		cartesian_action_service_1D(z_pose=0.202)

		# grasping_service(width=0.048, force=20.0)
		grasping_service(width=0.0245, force=20.0)
		time.sleep(3)
		if not high:
			cartesian_action_service_1D(z_pose=0.40)
		else:
			cartesian_action_service_1D(z_pose=0.65)

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
		input("If the robot is in the start position type enter to grasp. Otherwise - rosrun metrics_control move_to_start.py")
		#### SLIDE GEARS TOGETHER 
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
			
		if (obj1.y < obj2.y): # first object is on the right
			cartesian_action_service_2D(pose=[obj1.x, obj1.y-0.05], slow=False)
			time.sleep(3)
			rotate_ee_service(angle=0.0)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.201)
			time.sleep(3)
			cartesian_action_service_2D(pose=[obj2.x, obj2.y-0.01], slow=True)
		elif (obj1.y > obj2.y):  # first object is on the left
			cartesian_action_service_2D(pose=[obj1.x, obj1.y+0.05], slow=False)
			time.sleep(3)
			rotate_ee_service(angle=0.0)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.201)
			time.sleep(3)
			cartesian_action_service_2D(pose=[obj2.x, obj2.y-0.01], slow=True)

		##### MOVE TO START 
		ready_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, 0.25 * math.pi]
		req = TakeActionRequest()
		req.q = ready_pose
		move_to_joint_target(req)
		time.sleep(2)
		cartesian_action_service_2D(pose=[0.48, 0.00], slow=False)
		move_gripper_service(20.0, 0.08) 
		time.sleep(5)

		##### BRING THEM OVER 
		gear_detection = False
		bottom_casing_detection = False

		while not (gear_detection and bottom_casing_detection):
			objects = detection_client()

			for obj in objects.detection.detections:
				if obj.obj_class == 8:
					gear_detection = obj
				if obj.obj_class == 3:
					bottom_casing_detection = obj

			print("No detection")
			time.sleep(1)

		grasp(gear_detection.x, gear_detection.y, gear_detection.angle, high=True)

		# Bring them above the back plate

		cartesian_action_service_2D(pose=[bottom_casing_detection.x+0.005, bottom_casing_detection.y+0.005])
		time.sleep(3)
		rotate_ee_service(angle=bottom_casing_detection.angle)
		time.sleep(3)
		# insert the gears 
		cartesian_action_service_1D(z_pose=0.35)
		cartesian_action_service_1D(z_pose=0.21, slow=True)
		move_gripper_service(20.0, 0.08)

		##### MOVE TO START 
		ready_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, 0.25 * math.pi]
		req = TakeActionRequest()
		req.q = ready_pose
		move_to_joint_target(req)
		time.sleep(2)
		cartesian_action_service_2D(pose=[0.48, 0.00], slow=False)

	

if __name__ == '__main__':
	rospy.init_node('metrics_assembly')
	time.sleep(3)
	main()
	rospy.spin()
#!/usr/bin/env python3

import time
import rospy
import math
from cobot_msgs.msg import *
from cobot_msgs.srv import *
from franka_msgs.msg import FrankaState
from franka_msgs.msg import ErrorRecoveryActionGoal


safety_on = True
error = False 

def checking_collisions(msg, error_pub):
	global safety_on
	global error
	mode = msg.robot_mode
	if mode == 4 and not safety_on:
		error_pub.publish(ErrorRecoveryActionGoal())
		error = True

def main():

	detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
	
	rotate_ee_service = rospy.ServiceProxy("/rotate_ee", RotateEE)
	cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
	cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
	move_gripper_service = rospy.ServiceProxy("move_gripper", MoveGripper)
	move_to_joint_target = rospy.ServiceProxy('/take_action', TakeAction)
	grasping_service = rospy.ServiceProxy("grasp", Grasp)


	# Make sure the gripper is working properly
	try:
		move_gripper_service(20.0, 0.02) 
		move_gripper_service(20.0, 0.08) 
	except rospy.ServiceException as exc:
		print("Gripper not operational")
	else:
		global safety_on

		global error 
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
			safety_on = False
			cartesian_action_service_1D(z_pose=0.3)
			cartesian_action_service_1D(z_pose=0.201, slow=True)
			safety_on = True
			time.sleep(3)
			# Push left
			cartesian_action_service_2D(pose=[obj2.x, obj2.y-0.03], slow=True)
		elif (obj1.y > obj2.y):  # first object is on the left
			cartesian_action_service_2D(pose=[obj1.x, obj1.y+0.06], slow=False)
			time.sleep(3)
			rotate_ee_service(angle=0.0)
			time.sleep(3)
			safety_on = False
			cartesian_action_service_1D(z_pose=0.3)
			cartesian_action_service_1D(z_pose=0.201, slow=True)
			safety_on = True
			time.sleep(3)
			# Push right
			cartesian_action_service_2D(pose=[obj2.x, obj2.y+0.03], slow=True)


		##### MOVE TO START 

		cartesian_action_service_1D(z_pose=0.40)

		ready_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, 0.25 * math.pi]
		req = TakeActionRequest()
		req.q = ready_pose
		move_to_joint_target(req)
		time.sleep(2)
		cartesian_action_service_2D(pose=[0.48, 0.00], slow=False)
		move_gripper_service(20.0, 0.08) 
		time.sleep(10)

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

		cartesian_action_service_2D(pose=[gear_detection.x, gear_detection.y])
		time.sleep(3)
		rotate_ee_service(angle=gear_detection.angle)
		time.sleep(3)
		safety_on = False
		cartesian_action_service_1D(z_pose=0.3)
		cartesian_action_service_1D(z_pose=0.2015, slow=True)
		safety_on = True
		# grasping_service(width=0.048, force=20.0)
		grasping_service(width=0.0245, force=20.0)
		time.sleep(3)
		cartesian_action_service_1D(z_pose=0.55)

		# Bring them above the back plate

		cartesian_action_service_2D(pose=[bottom_casing_detection.x, bottom_casing_detection.y])
		time.sleep(3)
		rotate_ee_service(angle=bottom_casing_detection.angle)
		time.sleep(3)
		# insert the gears 
		cartesian_action_service_1D(z_pose=0.35)
		cartesian_action_service_1D(z_pose=0.21, slow=True)
		move_gripper_service(20.0, 0.08)

		# Gently push the gears down to make sure they are correctly assembled
		safety_on = False
		cartesian_action_service_1D(z_pose=0.30)
		move_gripper_service(20.0, 0.0)	
		cartesian_action_service_1D(z_pose=0.210, slow=True)
		cartesian_action_service_1D(z_pose=0.45)
		move_gripper_service(20.0, 0.8)	
		safety_on = True

		#print(detection)
		
		#print("detections : " , detections)
		print ('Picking the top casing')
		

		dontmoveon = True 
		while(dontmoveon):
			error = False 
			##### MOVE TO START 
			ready_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, 0.25 * math.pi]
			req = TakeActionRequest()
			req.q = ready_pose
			move_to_joint_target(req)
			time.sleep(2)
			cartesian_action_service_2D(pose=[0.48, 0.00], slow=False)

		
			##### BRING THE TOP CASING ON TOP 
			time.sleep(5)
				
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

			safety_on = False
			cartesian_action_service_2D(pose=[casing_detection.x, casing_detection.y])
			time.sleep(3)
			rotate_ee_service(angle= casing_detection.angle)
			time.sleep(3)
			cartesian_action_service_1D(z_pose=0.3)
			cartesian_action_service_1D(z_pose=0.20133,slow=True)
			if not error:
				dontmoveon = False
			else:
				time.sleep(2) 

		grasping_service(width=0.0019, force=20.0)
		time.sleep(3)
		cartesian_action_service_1D(z_pose=0.45)

		# MOVE ON TOP Bottom_casing + gears assembly


		print ('Moving on to top casing')
		cartesian_action_service_2D(pose=[bottom_detection.x, bottom_detection.y -0.002])
		time.sleep(3)
		rotate_ee_service(angle= bottom_detection.angle)
		time.sleep(3)
		cartesian_action_service_1D(z_pose=0.214, slow=True)
		move_gripper_service(20.0, 0.8)				


		cartesian_action_service_1D(z_pose=0.30)
		move_gripper_service(20.0, 0.0)	
		for i in range(3):
			cartesian_action_service_1D(z_pose=0.2095, slow=True)
		cartesian_action_service_1D(z_pose=0.45)
		time.sleep(1)
		move_gripper_service(20.0, 0.8)	
			
		##### MOVE TO START 
		ready_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, 0.25 * math.pi]
		req = TakeActionRequest()
		req.q = ready_pose
		move_to_joint_target(req)
		time.sleep(2)
		cartesian_action_service_2D(pose=[0.48, 0.00], slow=False)


if __name__ == '__main__':
	rospy.init_node('metrics_assembly')
	error_pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=10)
	rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, checking_collisions, (error_pub))
	time.sleep(3)
	main()
	rospy.spin()
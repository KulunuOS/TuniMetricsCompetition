#!/usr/bin/env python3
import tf
import time
import rospy
from cobot_msgs.srv import *
from sensor_msgs.msg import JointState, CameraInfo
from cobot_vision.camera_parameters import CameraParameters
from cobot_controllers.demo_grasp import SingleDemoGraspAction

if __name__ == '__main__':
	rospy.init_node('metrics_assembly')
	
	listener_tf = tf.TransformListener()

	camera_params = CameraParameters()
	rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_params.camera_info_callback)

	grasp_client = SingleDemoGraspAction(listener_tf, camera_params)

	detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
	req = GraspPoseDetectionRequest()
	objects = detection_client(req)

	#grasp_client.open_hand()

	time.sleep(5)
	# Grasp a gear
	#for obj in objects:
	#	if (obj.obj_class == 1):

	detection = objects.detection.detections[0]
	print(detection)	
	grasp_client.reach_hover(detection.x, detection.y)
	# grasp_client.reach_grasp_hover_kps(detection.kps_x, detection.kps_y)
	# grasp_client.fix_angle(detection.angle)
	
	#cartesian_plan, fraction = grasp_client.plan_linear_z(-0.39)
	#grasp_client.execute_plan(cartesian_plan)
	cartesian_plan, fraction =grasp_client.plan_linear_z(0.20)
	grasp_client.execute_plan(cartesian_plan)
	# grasp_client.approach_grasp()
	#grasp_client.close_hand()
	
	#cartesian_plan, fraction =grasp_client.plan_linear_z(0.39)
	#grasp_client.execute_plan(cartesian_plan)

	rospy.spin()
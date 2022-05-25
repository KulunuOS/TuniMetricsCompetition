#!/usr/bin/env python3
import tf
import time
import rospy
from cobot_msgs.srv import *
from sensor_msgs.msg import JointState
from cobot_controllers.demo_grasp import SingleDemoGraspAction

if __name__ == '__main__':
	rospy.init_node('metrics_assembly')
	
	time.sleep(10)
	listener_tf = tf.TransformListener()

	grasp_client = SingleDemoGraspAction(listener_tf)

	detection_client = rospy.ServiceProxy('/detect_grasp_pose', GraspPoseDetection)
	req = GraspPoseDetectionRequest()
	objects = detection_client(req)

	# Grasp a gear
	#for obj in objects:
	#	if (obj.obj_class == 1):
	detection = objects.detection.detections[0]
	
	grasp_client.reach_hover(detection.x, detection.y)
	grasp_client.fix_angle(detection.angle)
	grasp_client.reach_grasp_hover_kps(detection.kps_x, detection.kps_y)
	grasp_client.close_hand()
	cartesian_plan, fraction = grasp_client.plan_linear_z(0.45)
	grasp_client.execute_plan(cartesian_plan)

	rospy.spin()
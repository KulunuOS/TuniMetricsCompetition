import tf
import numpy as np
import cv2
import math
import rospy
import threading
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from cobot_msgs.msg import *
from cobot_msgs.srv import *

from cobot_vision.stream_monitor import ImageStreamMonitor

class ResnetStreamMonitor(ImageStreamMonitor):
	
	def __init__(self, predictor, camera_params, camera_2_robot_pose_converter):
		super().__init__( predictor, camera_params, camera_2_robot_pose_converter)
		self.detection_type = "contours"
		self.bridge = CvBridge()
		self.classes = {
			'garbage':0,
			'gear':1,
			'gear_side':2,
			'bottom_casing':3,
			'bottom_casing_side':4,
			'bottom_casing_inv':5,
			'top_casing':6,
			'top_casing_inv':7,
			'two_gears':8,
			'two_gears_on_bottom_casing':9
		}

		self.detections = {
			0:[],
			1:[],
			2:[],
			3:[],
			4:[],
			5:[],
			6:[],
			7:[],
			8:[],
			9:[]
		}

	def reset_detections(self):		
		self.detections = {
			0:[],
			1:[],
			2:[],
			3:[],
			4:[],
			5:[],
			6:[],
			7:[],
			8:[],
			9:[]
		}

	def preprocess_img(self, rgb_data):
		try:
			rgb_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		except CvBridgeError as e:
			print(e)
		finally:
			return rgb_img

	def draw_outputs(self, rgb_image, detections):
		if (len(detections) > 0):
			#print("Predictions[0] - {}".format(predictions[0]))
			#print("Predictions[1] - {}".format(predictions[1]))
			for prediction in detections:
				#print("Prediction - {}".format(prediction))
				x,y,w,h = cv2.boundingRect(prediction[1])
				cv2.putText(rgb_image, prediction[0], (x,y+h+10), 
		                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
				cv2.putText(rgb_image, str(prediction[3]), (x,y+h+60), 
		                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255))
				cv2.drawContours(rgb_image, [prediction[1]], -1, (0,255,0), 3)
				cv2.circle(rgb_image, (prediction[2][0], prediction[2][1]), 7, (255, 255, 255), -1)
		cv2.imshow("image", rgb_image)
		keyboard = cv2.waitKey(5)

	def process_detections(self, detections):
		result = []
		ref_x = self.camera_params.ctrX
		ref_y = self.camera_params.ctrY
		for prediction in detections:	 
			M = cv2.moments(prediction[1])
			cX = int(M["m10"] / M["m00"])
			#cX = int(cX - ref_x)
			cY = int(M["m01"] / M["m00"])
			#cY = int(ref_y - cY)
			angle = self.camera_2_robot_pose_converter.get_orientation(prediction[1])
			# angle = np.degrees(angle)
			result.append((prediction[0], prediction[1], (cX, cY), angle))
		return result

	def publish_results(self, detections):
		msg = Detections()
		for prediction in detections:
		    detection = Detection()
		    detection.obj_class =  self.classes[prediction[0]]
		    detection.obj_class_name =  prediction[0]
		    detection.x =  prediction[2][0]
		    detection.y =  prediction[2][1]
		    msg.detections.append(detection)
		
		self.detection_pub.publish(msg)

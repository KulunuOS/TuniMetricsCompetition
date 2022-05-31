#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np

import time
import cv2.aruco as aruco

class testBGSub():

	def __init__(self):

		self.trained = False
		self.depth_raw = None
		self.backSub = cv2.createBackgroundSubtractorMOG2()
		self.lr = -1
		self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_cb)
		self.bridge = CvBridge()
	def img_cb(self, rgb_data):
		self.rgb_img = None
		try:
		    self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		except CvBridgeError as e:
		    print(e)
		fgMask = self.backSub.apply(self.rgb_img,learningRate = self.lr)
		keyboard = cv2.waitKey(10)
		if keyboard == ord('s'):
			print('change lr')
			self.lr = 0
			self.trained = True

		cv2.imshow("mask", fgMask)
		cv2.waitKey(10)
		if keyboard == ord('q'):			
			cv2.destroyAllWindows()
			sys.exit(0)


def main():

	rospy.init_node('azureTest')
	rospy.loginfo("Trying to start . . ")
	rospy.loginfo("test 0 ")
	rospy.loginfo("test 1 ")
	nodeMain = testBGSub()
	rospy.spin()
	rospy.loginfo("test 2 ")
	rospy.loginfo("End of main()")

if __name__ == "__main__":
    main()

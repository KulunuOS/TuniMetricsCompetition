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
		blur = cv2.blur(self.rgb_img,(9,9))
		fgMask = self.backSub.apply(blur,learningRate = self.lr)
		keyboard = cv2.waitKey(10)
		if keyboard == ord('s'):
			print('change lr')
			self.lr = 0
			self.trained = True
		fgMask[fgMask != 255] = 0

		if self.lr == -1:
			self.bg = blur
		if self.lr == 0:
			t1 = np.float32(np.mean(self.bg,-1))
			t2 = np.float32(np.mean(blur,-1))
			tmp = np.abs(t1-t2)
			tmp = tmp>50
			fgMask = np.uint8(tmp)*255
		contours, hierarchy = cv2.findContours(fgMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		area = np.array([cv2.contourArea(cnt) for cnt in contours])
		contours = [contours[i] for i in range(len(contours)) if area[i] > 50]
		print(len(contours))
		cv2.drawContours(self.rgb_img, contours, -1, (0,255,0), 3)
		cv2.imshow("mask", self.rgb_img)
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

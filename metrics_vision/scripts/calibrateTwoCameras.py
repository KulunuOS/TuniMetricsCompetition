#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
import os
import time


class bgSubtractor_simple():
	def __init__(self,thresh =50):
		self.bg = None
		self.thresh =50
		self.mask = None
	def init_bg(self,im):
		self.bg = cv2.blur(im,(9,9))
		
	def getMask(self,im):
		blur = cv2.blur(im,(9,9))
		t1 = np.float32(np.mean(self.bg,-1))
		t2 = np.float32(np.mean(blur,-1))
		tmp = np.abs(t1-t2)
		tmp = tmp>self.thresh
		self.mask = np.uint8(tmp)*255		
		return self.mask
		
		
class testBGSub():

	def __init__(self):
		self.bgsubt_top = bgSubtractor_simple()
		self.bgsubt_side = bgSubtractor_simple()
		self.points_top = []
		self.points_side = []
		self.H = None
		self.trained = False
		self.depth_raw = None
		self.lr = -1
		self.rgb_sub1 = message_filters.Subscriber("/cam_top/color/image_raw", Image)
		self.rgb_sub2 = message_filters.Subscriber("/cam_side/color/image_raw", Image)
		ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub1, self.rgb_sub2], 1, 1)  
		self.counter = 0
		ts.registerCallback(self.img_cb)
		self.bridge = CvBridge()
		self.initFlag = False

	def get_center_mass(self,cnt):
		M = cv2.moments(cnt)	
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		return (cX,cY)	
	def img_cb(self, rgb_top,rgb_side):
		self.rgb_img = None
		self.counter+=1
		c_top = None
		c_side = None
		if self.counter < 100:
	    	    return		
		try:
		    self.rgb_top = self.bridge.imgmsg_to_cv2(rgb_top, "bgr8")
		    self.rgb_side = self.bridge.imgmsg_to_cv2(rgb_side, "bgr8")
		except CvBridgeError as e:
		    print(e)
		if not self.initFlag:
			self.bgsubt_top.init_bg(self.rgb_top)
			self.bgsubt_side.init_bg(self.rgb_side)
			self.initFlag = True
		msk_top = self.bgsubt_top.getMask(self.rgb_top)
		msk_side =self.bgsubt_side.getMask(self.rgb_side)
		contours_top, hierarchy = cv2.findContours(msk_top, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours_side, hierarchy = cv2.findContours(msk_side, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)		
		if len(contours_top) == 1 and len(contours_side) == 1:
			c_top = self.get_center_mass(contours_top[0])
			c_side = self.get_center_mass(contours_side[0])
		#area = np.array([cv2.contourArea(cnt) for cnt in contours])
		#contours = [contours[i] for i in range(len(contours)) if area[i] > 100]
		#cv2.drawContours(self.rgb_img, contours, -1, (0,255,0), 3)
		
		keyboard = cv2.waitKey(1)
		if keyboard == ord('q'):			
			cv2.destroyAllWindows()
			sys.exit(0)
		if keyboard == ord('s'):
			self.points_top.append([c_top[0],c_top[1]])
			self.points_side.append([c_side[0],c_side[1]])
			print('point added')
		if keyboard == ord('h'):
			if len(self.points_top) < 4:
				print('add more points')
			else:
				self.points_top = np.array(self.points_top)
				self.points_side = np.array(self.points_side)
				self.H, mask = cv2.findHomography(self.points_top, self.points_side)
				print(self.H)
		if self.H is not None and c_top is not None:
			cv2.circle(msk_top, c_top, 2, (125, 125, 125), -1)
			dst = cv2.perspectiveTransform(np.array([c_top],dtype=np.float64).reshape(-1,1,2), self.H)
			dst = dst.reshape((-1,2))
			cv2.circle(msk_side, (int(dst[0,0]), int(dst[0,1])), 2, (125, 125, 125), -1)
		cv2.imshow("mask top", cv2.resize(msk_top,(640,480)))
		cv2.imshow("mask side", cv2.resize(msk_side,(640,480)))	
		keyboard = cv2.waitKey(5)	
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

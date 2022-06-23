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

class testBGSub():

	def __init__(self):

		self.trained = False
		self.depth_raw = None
		self.lr = -1
		self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_cb)
		self.bridge = CvBridge()
		self.initFlag = False
		self.curclass = '01'
		#os.mkdir('classification_dataset')
		#os.mkdir('classification_dataset/rgb')
		#os.mkdir('classification_dataset/mask')
		#for i in range(10):
			#classname = str(i).zfill(2)
			#os.mkdir(os.path.join('classification_dataset/rgb',classname))
			#os.mkdir(os.path.join('classification_dataset/mask',classname))
			
	def img_cb(self, rgb_data):
		self.rgb_img = None
		try:
		    self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		except CvBridgeError as e:
		    print(e)
		fgMask = np.zeros((self.rgb_img.shape[0],self.rgb_img.shape[1]),dtype=np.uint8)
		blur = cv2.blur(self.rgb_img,(9,9))

		if not self.initFlag:
			self.bg = blur
			self.initFlag = True
		if self.initFlag:
			t1 = np.float32(np.mean(self.bg,-1))
			t2 = np.float32(np.mean(blur,-1))
			tmp = np.abs(t1-t2)
			tmp = tmp>30
			fgMask = np.uint8(tmp)*255
		contours, hierarchy = cv2.findContours(fgMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		area = np.array([cv2.contourArea(cnt) for cnt in contours])
		contours = [contours[i] for i in range(len(contours)) if area[i] > 100]
		#cv2.drawContours(self.rgb_img, contours, -1, (0,255,0), 3)
		
		cv2.imshow("mask", cv2.resize(fgMask,(640,480)))
		keyboard = cv2.waitKey(5)
		if keyboard == ord('q'):			
			cv2.destroyAllWindows()
			sys.exit(0)
		if keyboard >= ord('0') and keyboard <= ord('9'):
			self.curclass = chr(keyboard).zfill(2)
			print('class '+self.curclass+' selected')
		if keyboard == ord('s'):
				rgbfolder = os.path.join('classification_dataset/rgb',self.curclass)
				mskfolder = os.path.join('classification_dataset/mask',self.curclass)
				for contour in contours:
					imname = str(len(os.listdir(rgbfolder))).zfill(3)
					x,y,w,h = cv2.boundingRect(contour); 
					im = self.rgb_img[y:y+h,x:x+w,:]
					msk = fgMask[y:y+h,x:x+w]
					cv2.imwrite(os.path.join(rgbfolder,imname)+'.png',im)
					cv2.imwrite(os.path.join(mskfolder,imname)+'.png',msk)
					print('image saved '+imname)
			
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

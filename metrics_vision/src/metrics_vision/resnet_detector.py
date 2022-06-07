import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
import os
import time
import sys
import torch
from torchvision import datasets, models, transforms
import torch.nn as nn
import albumentations as A
from albumentations.pytorch import ToTensorV2

class ResnetMetrics():

    def __init__(self, model_filepath):
        self.model_filepath = model_filepath
        self.detection_type = "contours"
        self.trained = False
        self.depth_raw = None
        self.lr = -1
        self.bridge = CvBridge()
        self.initFlag = False
        self.transforms = A.Compose(
            [
                A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
                A.Resize(224, 224),
                ToTensorV2(),
            ])
        self.model = models.resnet50(pretrained=True)
        num_ftrs = self.model.fc.in_features
        self.model.fc = nn.Linear(num_ftrs, 10)
        self.model = self.model.to('cpu')
        self.model.eval()
        self.model.load_state_dict(torch.load(self.model_filepath, map_location=torch.device('cpu')))
        self.classnames = ['garbage','gear','gear_side','bottom_casing',
                           'bottom_casing_side','bottom_casing_inv','top_casing',
                           'top_casing_inv','two_gears','two_gears_on_bottom_casing']


    def predict(self, image):
        result = []
        fgMask = np.zeros((image.shape[0],image.shape[1]),dtype=np.uint8)
        blur = cv2.blur(image,(9,9))

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
        #cv2.drawContours(image, contours, -1, (0,255,0), 3)

        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            im = image[y:y+h,x:x+w,:]
            im_torch = self.transforms(image=im)
            im_torch = im_torch['image'][None,...]
            with torch.no_grad():
                out = self.model(im_torch)
                _, preds = torch.max(out, 1)
                probs = torch.softmax(out,1)
            objectclass = self.classnames[preds]
            if preds != 0:
                result.append((objectclass, contour))
        return result


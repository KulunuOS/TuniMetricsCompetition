# -*- coding: utf-8 -*-
"""
Created on Wed Jun  8 10:46:43 2022

@author: monakhov
"""
import numpy as np
import cv2

'''
finds orientation between first axis of object (axis along which the object is stretched the most) and the x-axis of the robot
cnt - points of object's contour
homMatrix - homography matrix between camera coordinates and the robot base frame
angle is returned in radians, in range [-pi/2, pi/2]
'''
def getOrientation(cnt,homMatrix):
    xVector = np.array([1.0,0])
    sz = len(cnt)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = cnt[i,0,0]
        data_pts[i,1] = cnt[i,0,1]
    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
    #print(eigenvectors)
    #print(eigenvalues)
    mainAxis = eigenvectors[0]
    pts_im = np.array([[0.,0],[mainAxis[0],mainAxis[1]]]).reshape(-1,1,2)
    pts_base = cv2.perspectiveTransform(pts_im, homMatrix)
    pts_base.reshape(-1,2)
    mainAxis = pts_base[0]-pts_base[1]
    mainAxis = mainAxis/np.linalg.norm(mainAxis)
    angle = np.arccos(np.clip(np.dot(mainAxis, xVector), -1.0, 1.0))
    if angle > np.pi/2:
        angle = angle-np.pi
    return angle

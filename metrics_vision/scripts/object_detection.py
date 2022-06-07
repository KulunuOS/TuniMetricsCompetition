#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cobot_msgs.msg import *
from cobot_msgs.srv import GraspPoseDetection
from cobot_vision.detectron_model import DetectronModel
from metrics_vision.resnet_detector import ResnetMetrics
from metrics_vision.resnet_monitor import ResnetStreamMonitor
from cobot_vision.camera_parameters import CameraParameters
from cobot_vision.poses_converter import PosesConverter


if __name__ == '__main__':
    rospy.init_node('metrics_vision')

    #object_locator = DetectronModel("/home/panda2/catkin_ws/src/tuni-panda/TuniMetricsCompetition/metrics_vision/models/metrics_model.pth")
    object_locator = ResnetMetrics("/home/panda2/catkin_ws/src/tuni-panda/TuniMetricsCompetition/metrics_vision/models/model_50epochs.pth")
    
    camera_params = CameraParameters()
    camera_2_robot_pose_converter = PosesConverter(camera_params, "/panda_link0", "/panda_link8","/camera_color_frame")

    stream_monitor = ResnetStreamMonitor(object_locator, camera_params, camera_2_robot_pose_converter)
    
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_params.camera_info_callback)

    detection_server = rospy.Service('/detect_grasp_pose', GraspPoseDetection, stream_monitor.image_analyze)
    
    sub = rospy.Subscriber("/camera/color/image_raw", Image, stream_monitor.image_analyze_stream)
    
    rospy.spin()

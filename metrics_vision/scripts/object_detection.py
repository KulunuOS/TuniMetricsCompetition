#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cobot_msgs.msg import *
from cobot_msgs.srv import GraspPoseDetection
from cobot_vision.detectron_model import DetectronModel
from cobot_vision.stream_monitor import ImageStreamMonitor



def callback(image_data, stream_monitor):
    # Preprocess image
    rgb_image = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width,3)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
    # Process image
    stream_monitor.image_analyze_stream(rgb_image)


if __name__ == '__main__':
    rospy.init_node('metrics_vision')

    object_locator = DetectronModel("/home/panda2/catkin_ws/src/tuni-panda/TuniMetricsCompetition/metrics_vision/models/metrics_model.pth")
    stream_monitor = ImageStreamMonitor(object_locator)
    
    detection_server = rospy.Service('/detect_grasp_pose', GraspPoseDetection, stream_monitor.image_analyze)
    
    sub = rospy.Subscriber("/camera/color/image_raw", Image, callback, (stream_monitor))
    
    rospy.spin()

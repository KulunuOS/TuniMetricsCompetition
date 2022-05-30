#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image, CameraInfo

from cobot_msgs.srv import GraspPoseDetection
from cobot_vision.detectron_model import DetectronModel
from cobot_vision.stream_monitor import ImageStreamMonitor
from cobot_vision.camera_parameters import CameraParameters


def callback(image_data, stream_monitor):
    # Preprocess image
    rgb_image = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width,3)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
    # Process image
    stream_monitor.image_analyze_stream(rgb_image)


if __name__ == '__main__':
    rospy.init_node('metrics_vision')

    camera_params = CameraParameters()
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_params.camera_info_callback)

    object_locator = DetectronModel("/home/panda2/catkin_ws/src/tuni-panda/TuniMetricsCompetition/metrics_vision/models/metrics_model.pth")
    stream_monitor = ImageStreamMonitor(object_locator, camera_params)
    
    detection_server = rospy.Service('/detect_grasp_pose', GraspPoseDetection, stream_monitor.image_analyze)
    
    sub = rospy.Subscriber("/camera/color/image_raw", Image, callback, (stream_monitor))
    
    rospy.spin()

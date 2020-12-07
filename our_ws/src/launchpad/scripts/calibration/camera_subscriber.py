#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def listener():
    # node setup
    rospy.init_node('camera_subscriber')
    sub = rospy.Subscriber('image_raw', Image, process_image)
    rospy.spin()

    # shutdown windows
    cv2.destroyAllWindows()

def process_image(req):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(req, "bgr8")
        
        # camera parameters
        DIM = (320,240)
        K = np.array([[160.023219, 0, 162.971810], [0, 160.263543, 123.423868], [0, 0, 1]])
        D = np.array([[-0.266205], [0.045222], [-0.001402], [-0.000906]])
        
        # undistort
        K_new, roi = cv2.getOptimalNewCameraMatrix(K, D, DIM, 1, DIM)
        und = cv2.undistort(image, K, D, None, K_new)

        cv2.imshow('image', image)
        cv2.imshow('und', und)
        cv2.waitKey(100)
    except CvBridgeError:
        pass

if __name__ == '__main__':
    listener()

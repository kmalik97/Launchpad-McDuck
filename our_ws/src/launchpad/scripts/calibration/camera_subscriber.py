#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def listener():
    # node setup
    rospy.init_node('camera_subscriber')
    sub = rospy.Subscriber('raw_image', Image, process_image)
    rospy.spin()

    # shutdown windows
    cv2.destroyAllWindows()

def process_image(req):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(req, "bgr8")
        cv2.imshow('image', image)
        cv2.waitKey(100)
    except CvBridgeError:
        pass

if __name__ == '__main__':
    listener()

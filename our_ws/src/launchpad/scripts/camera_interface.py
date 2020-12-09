#!/usr/bin/env python
import rospy
import cv2
import time
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from cv_bridge import CvBridge, CvBridgeError
from launchpad.srv import snapshot, snapshotResponse

class Camera_Interface:
    def __init__(self):
        self.service = rospy.Service("snapshot", snapshot, self.handle_snapshot)
        self.camera = PiCamera()
        self.camera.resolution = (320, 240)
        self.frame = PiRGBArray(self.camera, size=(320, 240))
        self.bridge = CvBridge()

    # take a snapshot with the camera
    def handle_snapshot(self, req):
        # get snapshot
        start_timer = time.time()
        #self.camera.capture(self.frame, format="bgr")

        self.camera.capture(self.frame, format="bgr", use_video_port=True)
        print("total time: %f"%(time.time()-start_timer))
        image = self.frame.array
        self.frame.truncate(0)


        # convert cv2 image to ROS image
        try:
            image_ros = self.bridge.cv2_to_imgmsg(image, "bgr8")
            return snapshotResponse(image_ros)
        except CvBridgeError as e:
            print("cv2 to ROS conversion failed: %s"%e)

            
    # shutdown
    def on_shutdown(self):
        self.camera.close()
        print("camera_interface node shutdown")

# setup snapshot server
def server_snapshot():
    print("initializing camera_interface node")
    rospy.init_node("camera_interface")
    camera_interface = Camera_Interface()
    rospy.on_shutdown(camera_interface.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    server_snapshot()

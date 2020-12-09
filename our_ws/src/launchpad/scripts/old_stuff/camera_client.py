#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from launchpad.srv import snapshot

class Camera_Client:
    def __init__(self):
        self.bridge = CvBridge()
        self.rate = 0.01
        self.timer = rospy.Timer(rospy.Duration(self.rate), self.display_image)

    def display_image(self, event):
        # need to wait until serive is available
        rospy.wait_for_service("snapshot")

        try:
            # service callback function handle
            handle_snapshot = rospy.ServiceProxy("snapshot", snapshot)

            # service response
            image_response = handle_snapshot()
            image_ros = image_response.image

            # convert ROS image to cv2 image and display
            try:
                image = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
                cv2.imshow("image", image)
                cv2.waitKey(int(self.rate*1000))
            except CvBridgeError as e:
                print("ROS to cv2 conversion failed: %s"%e)
        except rospy.ServiceException as e:
            print("snapshot service call failed: %s"%e)
    
    # shutdown
    def on_shutdown(self):
        cv2.destroyAllWindows()
        print("camera_client node shutdown")

# setup snapshot client
def client_snapshot():
    print("initializing camera_client node")
    rospy.init_node("camera_client")
    camera_client = Camera_Client()
    rospy.on_shutdown(camera_client.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    client_snapshot()

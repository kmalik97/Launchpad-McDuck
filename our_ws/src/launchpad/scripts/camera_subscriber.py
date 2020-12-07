import rospy
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv_bridge import CVBridge, CVBridgeError
from sensor_msgs.msg import Image

def listener():
    # node setup
    rospy.init_node('camera_subscriber')
    sub = rospy.Subscriber('raw_image', Image, process_image)
    bridge = CvBridge()
    rospy.spin()

def callback(req):
    try:
        image = bridge.imgmsg_to_cv2(req, "bgr8")
        cv2.imshow('image', image)
        cv2.waitKey(100)
    except CvBridgeError:
        pass

if __name__ == '__main__':
    listener()

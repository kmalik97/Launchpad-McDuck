import rospy
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv_bridge import CVBridge, CVBridgeError
from sensor_msgs.msg import Image

def talker():
    # node setup
    rospy.init_node('camera_publisher')
    pub = rospy.Publisher('raw_image', Image, queue_size=10)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    # camera setup
    camera = PiCamera()
    camera.resolution = (320,240)
    frame = PiRGBArray(camera, size=(320,240))
    
    # publish snapshot
    while not rospy.is_shutdown():
        camera.capture(frame, format='bgr')
        image = frame.array
        frame.truncate(0)
        image_ros = bridge.cv2_to_imgmsg(image, 'bgr8')
        pub.publish(image_ros)
        rate.sleep()

    # release camera on shutdown
    camera.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

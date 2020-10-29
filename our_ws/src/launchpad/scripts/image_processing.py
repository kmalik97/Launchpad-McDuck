#!/usr/bin/env python
import rospy 
import sys
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv_bridge import CvBridge, CvBridgeError
from launchpad.srv import snapshot,command,commandResponse


# TO DO: send motion_logic measurements instead of a command, will need to update service, break up into functions (?)

# image_processing.py: get the image from camera_interface.py, spit out string command to motion_logic

class Image_Processing:
    def __init__(self):

        # converting ROS image messages to OpenCV image
        self.bridge = CvBridge()
        self.rate = 0.01

        # rospy service get_command will return a letter that determines which way the duckiebot should go
        self.service = rospy.Service("get_command", command, self.handle_get_command)
    
    # handle the command service
    def handle_get_command(self,cmd):
        rospy.wait_for_service("snapshot")

        # get the ROS image
        try:
            handle_snapshot = rospy.ServiceProxy("snapshot", snapshot)
            image_response = handle_snapshot()
            image_ros = image_response.image

            # convert ROS image to cv2
            try:
                image = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
            except CvBridgeError as e:
                print("ROS to cv2 conversion failed: %s"%e)
        except rospy.ServiceException as e:
            print("snapshot service call failed: %s"%e)
        
        # hls: hue lightness saturation
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        
        # set lower and upper bounds (try for different shades/hues of yellow)
        lower_yellow = np.array([20, 100, 150])
        upper_yellow = np.array([30, 150, 255])


        # set lower and upper bounds (try for different shades/hues of white)
        lower_white = np.array([0, 190, 0])
        upper_white = np.array([255, 255, 255])

        # create mask for yellow and white colors
        mask_yellow = cv2.inRange(hls, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hls, lower_white, upper_white)
 
        # mask allowing both yellow and white
        mask = cv2.bitwise_or(mask_yellow, mask_white)
        
        # black out top portion of image so that we ignore the background
        for i in range(240/3):
            for j in range(320):
                mask[i][j] = 0

        # from the original image, get the yellow middle line and white boundaries (now in color)
        result = cv2.bitwise_and(image, image, mask=mask)
        
        # smooth the mask to allow for better edge detection
        mask = cv2.GaussianBlur(mask, (5,5), 0)

        # edge detection
        edges = cv2.Canny(mask, 200, 400)
        
        # reject lines shorter than this
        min_line_length = 8

        # maximum gap allowed between line segments for them to be considered a single line
        max_line_gap = 4

        # 1 degree angle precision
        angle_precision = np.pi/180
        
        # rho is the perpendicular distance from the origin to the line
        rho_precision = 1
       
        # minimum vote it should get for it to be considered a line (depends on number of points on line)
        min_vote = 10
        
        # create lines from edges
        lines = cv2.HoughLinesP(edges, rho_precision, angle_precision, min_vote, min_line_length, max_line_gap)
        
        # overlay red lines on the original image
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(result, (x1,y1), (x2,y2), (0,0,255), 2)

        cv2.imshow("result", result)
        cv2.waitKey(0)
        
        # arbitrary command
        return commandResponse("a")

    # shutdown
    def on_shutdown(self):
        cv2.destroyAllWindows()
        print("image_processing node shutdown")

def process_snapshot():
    print("initializing image_processing node")
    rospy.init_node("image_processing")
    image_processing = Image_Processing()
    rospy.on_shutdown(image_processing.on_shutdown)
    rospy.spin()


if __name__ == "__main__":
    process_snapshot()

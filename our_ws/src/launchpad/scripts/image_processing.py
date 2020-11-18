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
        upper_yellow = np.array([30, 175, 255])

        # set lower and upper bounds (try for different shades/hues of white)
        lower_white = np.array([0, 210, 0])
        upper_white = np.array([255, 255, 255])

        # create mask for yellow and white colors
        mask_yellow = cv2.inRange(hls, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hls, lower_white, upper_white)
 
        # mask allowing both yellow and white
        # ignore dashed yellow line for now
        # mask = cv2.bitwise_or(mask_yellow, mask_white)
        # mask = mask_white

        # black out top portion of image so that we ignore the background
        for i in range(240/3):
            for j in range(320):
                mask_white[i][j] = 0
                mask_yellow[i][j] = 0

        # smooth the mask to allow for better edge detection
        mask_white = cv2.GaussianBlur(mask_white, (11,11), 0)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (3,3), 0)

        # diliations and erosions to remove any small blobs left in image
        mask_white = cv2.erode(mask_white, None, iterations=2)
        #mask_yellow = cv2.erode(mask_yellow, None, iterations=0)
        mask_white = cv2.dilate(mask_white, None, iterations=2)
        #mask_yellow = cv2.dilate(mask_yellow, None, iterations=0)
               
        # from the original image, get the yellow middle line and white boundaries (now in color)
        mask = cv2.bitwise_or(mask_white, mask_yellow)
        result = cv2.bitwise_and(image, image, mask=mask)

        # edge detection
        edges_white = cv2.Canny(mask_white, 200, 400)
        edges_yellow = cv2.Canny(mask_yellow, 200, 400)
        edges = cv2.bitwise_or(edges_white, edges_yellow)
        cv2.imshow("white edges", edges_white)
        cv2.imshow("yellow edges", edges_yellow)
        cv2.imshow("edges", edges)

        # reject lines shorter than this
        min_line_length = 5

        # maximum gap allowed between line segments for them to be considered a single line
        max_line_gap = 10

        # 1 degree angle precision
        angle_precision = np.pi/180
        
        # rho is the perpendicular distance from the origin to the line
        rho_precision = 1
       
        # minimum vote it should get for it to be considered a line (depends on number of points on line)
        min_vote = 10
        
        # create lines from edges
        lines = cv2.HoughLinesP(edges, rho_precision, angle_precision, min_vote, None, min_line_length, max_line_gap)
        # print(lines)
        
        # overlay red lines on the original image
        if lines is not None:
            print("%d lines"%len(lines))
            for i in range(0, len(lines)):
                line = lines[i][0]
                cv2.line(result, (line[0],line[1]), (line[2],line[3]), (0,0,255), 1, cv2.LINE_AA)

        # polyfit the yellow line
        
        data_points = np.argwhere(edges_yellow>0)
        x_points = data_points[:,[1]]
        y_points = data_points[:,[0]]

        cv2.line(result, (x_points[0,:], y_points[0,:]), (x_points[-1,:], y_points[-1,:]), (255,0,255), 1, cv2.LINE_AA)

        curve_coefficients = np.polyfit(y_points[:,0], x_points[:,0], 2)
        new_y = np.linspace(0, 239, num=240)
        new_x = np.polyval(curve_coefficients, new_y)
        new_points = np.asarray([new_x,new_y]).astype(np.int32).T
        cv2.polylines(result, [new_points], False, (0,255,0), 1)   
        
        """
        x_points = np.zeros(2*len(lines))
        y_points = np.zeros(2*len(lines))
        if lines is not None:
            for i in range(0, len(lines)):
                line = lines[i][0]
                x_points[2*i] = line[0]
                x_points[2*i+1] = line[2]
                y_points[2*i] = line[1]
                y_points[2*i+1] = line[3]

        curve_coefficients = np.polyfit(y_points, x_points, 1)
        new_y = np.linspace(0, 239, num=240)
        new_x = np.polyval(curve_coefficients, new_y)
        new_points = np.asarray([new_x,new_y]).astype(np.int32).T
        # print(new_points.shape)
        cv2.polylines(result, [new_points], False, (0,255,0), 1)   
        """


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

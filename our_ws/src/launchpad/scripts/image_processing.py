#!/usr/bin/env python
import rospy 
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from launchpad.srv import snapshot,measurement,measurementResponse

# image_processing.py: get the image from camera_interface.py, spit out string command to motion_logic

class Image_Processing:
    def __init__(self):

        # converting ROS image messages to OpenCV image
        self.bridge = CvBridge()
        self.rate = 0.01

        # rospy service get_measurement will return an error from the desired heading
        self.service = rospy.Service("get_measurement", measurement, self.handle_get_measurement)
    
    # handle the image_measurement service
    def handle_get_measurement(self,req):
        # store operating point
        y_operating = req.y_operating       

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
        
        # undistorting image here 
        DIM=(320, 240)
        K=np.array([[157.48126107325643, 0.0, 155.54100584796126], [0.0, 157.4460985081828, 124.1673880384651], [0.0, 0.0, 1.0]])
        D=np.array([[-0.016100807266528787], [-0.0660261841490366], [0.12071322102537432], [-0.07516938046185537]])
        h,w = image.shape[:2]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        
        # Image is now the undistorted image
        
        # hls: hue lightness saturation
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        
        # set lower and upper bounds (try for different shades/hues of white)
        lower_white = np.array([0, 180, 0])
        upper_white = np.array([255, 255, 255])
        
        # set lower and upper bounds (try for different shades/hues of yellow)

        # KITCHEN VALUES
        #lower_yellow = np.array([25, 80, 150])
        #upper_yellow = np.array([30, 140, 255])

        # LUCAS ROOM VALUES
        #lower_yellow = np.array([20, 100, 100])
        #upper_yellow = np.array([30, 200, 255])

        # LIVING ROOM VALUES
        lower_yellow = np.array([20, 80, 175])
        upper_yellow = np.array([30, 175,255])
        
        # Red Object Values
        lower_red = np.array([0, 180, 52])
        upper_red = np.array([17, 210, 200])
        
        # Red Mask 
        red_image = image.astype(np.uint8)
        hsv = cv2.cvtColor(red_image,cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_red = cv2.bitwise_and(red_image,red_image,mask=mask_red)
        mask_red = cv2.dilate(mask_red, None, iterations=1)

        # grayscale
        hsv = cv2.cvtColor(mask_red, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(hsv, 127, 255, 0)
        cv2.imshow('thresh', thresh)
        im2, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Finding Red Object
        #cnts = cv2.findContours(mask_red.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = [] #if len(cnts) == 2 else cnts[1]
        
        red_obj_detect = False
        THRESHOLD = 20
        
        # If cnts is empty
        if not cnts:
            red_obj_detect = False
        else:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < threshold:
                print("Area under threshold")
            else:
                print('Red Object Detected Above Threshold')
                red_obj_detect = True
                
                # Detecting object corners
                x,y,w,h = cv2.boundingRect(c)
                
                # Drawing Rectangle Around object 
                cv2.rectangle(red_image,(x,y),(x+w,y+h),(0,255,0),2)
                
                # Center of mass 
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centerText = "Center Coordinates : ({x_coord} , {y_coord})".format(x_coord=center[0], y_coord=center[1])
                cv2.putText(red_image, centerText, (4, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # create mask for yellow and white colors
        mask_white = cv2.inRange(hls, lower_white, upper_white)
        mask_yellow = cv2.inRange(hls, lower_yellow, upper_yellow)

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
        mask_white = cv2.dilate(mask_white, None, iterations=2)
        #mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
        #mask_yellow = cv2.dilate(mask_yellow, None, iterations=1)
               
        # from the original image, get the yellow middle line and white boundaries (now in color)
        mask = cv2.bitwise_or(mask_white, mask_yellow)
        result = cv2.bitwise_and(image, image, mask=mask)

        # edge detection
        edges_white = cv2.Canny(mask_white, 200, 400)
        edges_yellow = cv2.Canny(mask_yellow, 200, 400)     # do we even need this?
        edges = cv2.bitwise_or(edges_white, edges_yellow)
        
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
        
        # overlay red lines on the original image
        if lines is not None:
            #print("%d lines"%len(lines))
            for i in range(0, len(lines)):
                line = lines[i][0]
                # cv2.line(result, (line[0],line[1]), (line[2],line[3]), (0,0,255), 1, cv2.LINE_AA)

        # get the pixel coordinates that are yellow
        data_points = np.argwhere(edges_yellow>0)

        x_error = 0.0
        
        if data_points.size > 0:
            lane_offset = 75
            x_points = data_points[:,[1]] + lane_offset
            y_points = data_points[:,[0]]

            # complete camera calibration so that x_points, y_points are transformed to a new set of points





            desired_coefficients = np.polyfit(y_points[:,0], x_points[:,0], 1)
            new_y = np.linspace(0, 239, num=240)
            new_x = np.polyval(desired_coefficients, new_y)
            new_points = np.asarray([new_x,new_y]).astype(np.int32).T
            cv2.polylines(result, [new_points], False, (0,255,0), 1)   
            # x_operating is found from desired line, y_operating passed as input from motion_logic
            x_operating = np.polyval(desired_coefficients, y_operating)
            # get x_error, the difference (eventually in meters) between the center line and the desired center line
            x_error = x_operating - 160

        
        # draw a straight line down the middle
        cv2.line(result, (320/2, 0), (320/2, 240), (0, 0, 255), 1, cv2.LINE_AA)
        # display images
        cv2.imshow("red_mas",mask_red)
        cv2.imshow("mask_yellow", mask_yellow)
        cv2.imshow("original", image)
        cv2.imshow("result", result)
        cv2.waitKey(0)

        
        print("x_error: %f"%x_error)

        return measurementResponse(x_error)

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

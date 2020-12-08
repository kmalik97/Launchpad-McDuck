#!/usr/bin/env python
import rospy 
import sys
import cv2
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from launchpad.srv import snapshot,measurement,measurementResponse
# image_processing.py: get the image from camera_interface.py, spit out string command to motion_logic

class Image_Processing:
    def __init__(self):

        # converting ROS image messages to OpenCV image
        self.bridge = CvBridge()

        # rospy service get_measurement will return an error from the desired heading
        self.service = rospy.Service("get_measurement", measurement, self.handle_get_measurement)
    
    # handle the image_measurement service
    def handle_get_measurement(self,req):
        start_time = time.time()

        # store operating point
        y_operating_mm = req.y_operating       

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
        DIM = (320,240)
        K = np.array([[160.023219, 0, 162.971810], [0, 160.263543, 123.423868], [0, 0, 1]])
        D = np.array([[-0.266205], [0.045222], [-0.001402], [-0.000906]])

        # undistort
        K_new, roi = cv2.getOptimalNewCameraMatrix(K, D, DIM, 1, DIM)
        und = cv2.undistort(image, K, D, None, K_new)

        #cv2.imshow('dst',dst)   
        #cv2.imshow('dis', image)

        #map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        

        #image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        
        #cv2.waitKey(0)
        
        # Image is now the undistorted image
        
        # hls: hue lightness saturation
        #hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        
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
        upper_yellow = np.array([30, 175,235])
        
        # Red Object Values
        lower_red = np.array([0, 180, 35])
        upper_red = np.array([17, 210, 200])
        
        # Red Mask 
        red_image = image.astype(np.uint8)
        hsv = cv2.cvtColor(red_image,cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        #mask_red = cv2.bitwise_and(red_image,red_image,mask=mask_red)
        mask_red = cv2.GaussianBlur(mask_red, (11,11), 0)
        #mask_red = cv2.erode(mask_red, None, iterations=2)
        #mask_red = cv2.dilate(mask_red, None, iterations=1)
        #edges_red = cv2.Canny(mask_red, 200, 400)
        edges_red = mask_red
        

        # grayscale
        #hsv = cv2.cvtColor(edges_red, cv2.COLOR_BGR2GRAY)
        #ret, thresh = cv2.threshold(hsv, 127, 255, 0)
        thresh = edges_red
        #cv2.imshow('thresh', thresh)
        
        cnts = cv2.findContours(edges_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

        # print(cnts)
        
        # Finding Red Object
        #cnts = cv2.findContours(mask_red.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
        
        red_obj_det = False
        THRESHOLD = 20

        # Known Parameters 
        REAL_WIDTH = 150 # mm  
        PIXEL_WIDTH = 124 # pixels
        KNOWN_DISTANCE = 270 # mm

        FOCAL_LENGTH = (PIXEL_WIDTH * KNOWN_DISTANCE) / REAL_WIDTH 
        


        # If cnts is empty
        if len(cnts) == 0:
            red_obj_detect = False
        else:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < THRESHOLD:
                red_obj_det = False
                print("Area under threshold")
            else:
                #print('Red Object Detected Above Threshold')

                
                # Detecting object corners of the red object in pixels
                x,y,w,h = cv2.boundingRect(c)
                
                distance = (REAL_WIDTH * FOCAL_LENGTH) / w 
                print("Distance to object %d" % distance)

                if distance <= 200:
                    red_obj_det = True

                # Drawing Rectangle Around object 
                cv2.rectangle(red_image,(x,y),(x+w,y+h),(0,255,0),2)
                
                # Center of mass 
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centerText = "Center Coordinates : ({x_coord} , {y_coord})".format(x_coord=center[0], y_coord=center[1])
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(red_image, centerText, (4, 200), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)


        # LIVING ROOM HSV VALUES
        lower_yellow = np.array([0, 215, 115])
        upper_yellow = np.array([56, 255, 255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # black out top portion of image so that we ignore the background
        for i in range(240/3):
            for j in range(320):
                mask_yellow[i][j] = 0

        # smooth the mask to allow for better edge detection
        mask_yellow = cv2.GaussianBlur(mask_yellow, (3,3), 0)

        # diliations and erosions to remove any small blobs left in image
        #mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
        #mask_yellow = cv2.dilate(mask_yellow, None, iterations=1)
               
        # from the original image, get the yellow middle line and white boundaries (now in color)
        mask = mask_yellow
        result = cv2.bitwise_and(image, image, mask=mask)

        # edge detection
        edges_yellow = cv2.Canny(mask_yellow, 200, 400)     # do we even need this?
        edges = edges_yellow
        
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

        x_error_pix = 0.0
        
        if data_points.size > 0:
            # distance between yellow line and desired heading (mm)
            lane_offset = np.array([[120],[0],[0]])

            # location of yellow line points (pixels)
            x_points_pix = data_points[:,[1]]
            y_points_pix = data_points[:,[0]]
            
            # yellow line in image plane
            yellow_coefficients_pix = np.polyfit(y_points_pix[:,0], x_points_pix[:,0], 1)
            y_yellow_pix = np.linspace(0, 239, num=240)
            x_yellow_pix = np.polyval(yellow_coefficients_pix, y_yellow_pix)
            
            # display regressed yellow line in image plane
            reg_yellow_pix = np.asarray([x_yellow_pix, y_yellow_pix]).astype(np.int32).T
            cv2.polylines(result, [reg_yellow_pix], False, (0,255,255), 1)   
            
            # convert image plane points to world points
            # homography matrix
            #H = np.array( [[120.0, 151.0, 50736 ], [0, 72.0, 37055.0], [0, 1.0, 316.0] ])
            R = np.array([[1, 0, 0],[0, -.342, -0.9397], [0, .9397, -.342]])
            t = np.array([[0], [25.5652], [222.1405]])
            T = np.concatenate((R[:,0:2], t), axis=1)
            H = np.matmul(K_new,T)
            reg_yellow_pix = np.concatenate((reg_yellow_pix, np.ones((240,1))), axis=1).T
            Hinv = np.linalg.inv(H)
            reg_yellow_mm = np.matmul(Hinv, reg_yellow_pix)

            #fail = np.argwhere(reg_yellow_mm[2,:]==0)

            reg_yellow_mm = reg_yellow_mm / reg_yellow_mm[2,:]

            # get desired heading from yellow line
            desired_mm = np.add(reg_yellow_mm, lane_offset)
            
            # desired line in world plane
            desired_coefficients_mm = np.polyfit(desired_mm[1,:], desired_mm[0,:], 1)

            # x-component of operating point in world plane
            x_operating_mm = np.polyval(desired_coefficients_mm, y_operating_mm)

            # get x_error, the difference between the true heading and desired heading in world plane
            x_error_mm = x_operating_mm

            # convert x difference in mm to width difference in pixels
            x_error_pix = np.array([[x_operating_mm], [y_operating_mm], [1]])
            x_error_pix = np.matmul(H,x_error_pix)

            # x_error_pix[0]: width in pixels, subtract from center to get error from center 
            x_error_pix = (x_error_pix[0]/x_error_pix[2])-160
        
        # draw a straight line down the middle
        # cv2.line(result, (320/2, 0), (320/2, 240), (0, 0, 255), 1, cv2.LINE_AA)
        #cv2.imshow("red_mask",mask_red)
        #cv2.imshow("mask_yellow", mask_yellow)
        #cv2.imshow("original", image)
        #cv2.imshow("result", result)
        #cv2.imshow("red image", red_image)
        #cv2.waitKey(10)
        print("total time: %f"%(time.time()-start_time))

        print("x_error_pix: %f"%x_error_pix)

        return measurementResponse(x_error_pix, red_obj_det)

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

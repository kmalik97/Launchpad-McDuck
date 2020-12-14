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
        self.bridge = CvBridge()
        self.service = rospy.Service("get_measurement", measurement, self.handle_get_measurement)
        self.prev_error = 0.0

    # handle the image_measurement service
    def handle_get_measurement(self,req):

        # how far in front of us (mm) we want to calculate heading error
        y_operating_mm = 100       

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
                rospy.loginfo("image_processing: ROS to cv2 conversion failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.loginfo("image_processing: snapshot service call failed: %s"%e)
        
        # image resolution
        DIM = (320,240)

        # intrinsic camera matrix
        K = np.array([[160.023219, 0, 162.971810], [0, 160.263543, 123.423868], [0, 0, 1]])
        
        # distortion coefficients
        D = np.array([[-0.266205], [0.045222], [-0.001402], [-0.000906]])
        

        red_image = image.copy()
        # black out top portion of image so that we ignore the background
        for i in range(240/3):
            for j in range(320):
                image[i][j] = np.asarray([0,0,0])
        # new camera matrix for undistorted image
        K_new, roi = cv2.getOptimalNewCameraMatrix(K, D, DIM, 1, DIM)
        
        # undistort
        und = cv2.undistort(image, K, D, None, K_new)
        
        red_image = cv2.undistort(red_image, K, D, None, K_new)
        
        # Red Object Values
        lower_red = np.array([0, 144, 179])
        upper_red = np.array([16, 255, 255])
        
        # Red Mask 
        red_image = red_image.astype(np.uint8)
        hsv = cv2.cvtColor(red_image,cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        #mask_red = cv2.bitwise_and(red_image,red_image,mask=mask_red)
        mask_red = cv2.GaussianBlur(mask_red, (11,11), 0)
        mask_red = cv2.erode(mask_red, None, iterations=2)
        mask_red = cv2.dilate(mask_red, None, iterations=1)
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
        THRESHOLD = 5

        # Known Parameters 
        REAL_WIDTH = 85 # mm  
        PIXEL_WIDTH = 78
        KNOWN_DISTANCE = 290 # mm

        # F_x from the intrinsic parameters
        FOCAL_LENGTH = K_new[0,0]
        FOCAL_LENGTH_CALC = (PIXEL_WIDTH * KNOWN_DISTANCE) / REAL_WIDTH 


        # If cnts is empty
        if len(cnts) == 0:
            red_obj_detect = False
        else:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < THRESHOLD:
                red_obj_det = False
                print("Area under threshold")
            else:
                
                # Detecting object corners of the red object in pixels
                x,y,w,h = cv2.boundingRect(c)
                
                distance = (REAL_WIDTH * FOCAL_LENGTH) / w 
                distance2 = (REAL_WIDTH * FOCAL_LENGTH_CALC ) / w 
                print("Distance to object %d" % distance)

                if distance <= 155:
                    red_obj_det = True

                # Drawing Rectangle Around object 

                cv2.rectangle(red_image,(x,y),(x+w,y+h),(0,255,0),2)
                print(w)
                
                # Center of mass 
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centerText = "Distance to Object {distance}".format(distance=distance)
                centerText2 = "Distance2 to Object {distance}".format(distance=distance2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(red_image, centerText, (4, 200), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(red_image, centerText2, (4, 220), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

        # CLOSET VALUES
        #lower_yellow = np.array([21, 144, 201])
        #upper_yellow = np.array([100, 255, 255])
        lower_yellow = np.array([3, 156, 213])
        upper_yellow = np.array([177, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # smooth the mask to allow for better edge detection
        mask_yellow = cv2.GaussianBlur(mask_yellow, (3,3), 0)

        # diliations and erosions to remove any small blobs left in image
        mask_yellow = cv2.erode(mask_yellow, np.ones((5,5), np.uint8), iterations=1)
        mask_yellow = cv2.dilate(mask_yellow, np.ones((5,5), np.uint8), iterations=1)
               
        # from the original image, get the yellow middle line and white boundaries (now in color)
        mask = mask_yellow
        result = cv2.bitwise_and(image, image, mask=mask)

        # edge detection
        edges_yellow = cv2.Canny(mask_yellow, 200, 400)     # do we even need this?
        edges = edges_yellow
        
        # get the pixel coordinates that are yellow
        data_points = np.argwhere(edges_yellow>0)

        x_error_pix = self.prev_error 
        
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
            
            # extrinsic rotation
            R = np.array([[1, 0, 0],[0, -.342, -0.9397], [0, .9397, -.342]])
            
            # extrinsic translation
            t = np.array([[0], [93.9693], [34.2020]])
            
            # extrinsic matrix
            T = np.concatenate((R[:,0:2], t), axis=1)
            
            # homography matrix
            H = np.matmul(K_new,T)

            # convert image coordinates to world coordinates
            reg_yellow_pix = np.concatenate((reg_yellow_pix, np.ones((240,1))), axis=1).T
            Hinv = np.linalg.inv(H)
            reg_yellow_mm = np.matmul(Hinv, reg_yellow_pix)
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
            
            self.prev_error = x_error_pix

        cv2.imshow("red_mask",mask_red)
        #cv2.imshow("mask_yellow", mask_yellow)
        #cv2.imshow("original", image)
        cv2.imshow("result", result)
        cv2.imshow("red image", red_image)
        cv2.waitKey(10)

        rospy.loginfo("image_processing: x_error_pix: %f"%x_error_pix)
        print(red_obj_det)
        return measurementResponse(x_error_pix, red_obj_det)

    # shutdown
    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("image_processing: image_processing node shutdown")

def process_snapshot():
    rospy.loginfo("image_processing: initializing image_processing node")
    rospy.init_node("image_processing")
    image_processing = Image_Processing()
    rospy.on_shutdown(image_processing.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    process_snapshot()

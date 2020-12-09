#!/usr/bin/env python
import rospy
import sys
import tty
import termios
import numpy as np
from launchpad.srv import motionLogic, motionLogicResponse, measurement

class Motion_Logic:
    def __init__(self):
        self.service = rospy.Service("motionLogic", motionLogic, self.handle_motion_logic)
        self.running_error = 0
        self.prev_error = 0
        self.linear_vel = 0.55
        self.angular_vel = 0.0

    # determine linear and angular velocity
    def handle_motion_logic(self, req):

        linear_vel = self.linear_vel
        angular_vel = self.angular_vel

        # get the image data from image_processing.py
        rospy.wait_for_service("get_measurement")
        try:
            handle_measurement = rospy.ServiceProxy("get_measurement", measurement)
            # calculate the operating point
            pwm_left = req.pwm_left
            pwm_right = req.pwm_right
            
            # the point at which we determine the error, how far in front of us we want to take the error
            y_operating_mm = 100
            meas = handle_measurement(y_operating_mm)

            # get the error from the desired heading
            # use x_error to determine velocities
            x_error = meas.x_error
            # get the status of whether or not there is a object 
            red_obj_det = meas.red_obj_det

            # PID Parameters
            Kp = 0.001
            Ki = 0.001
            Kd = 0.001

            # PlaceHolder Value
            #running_error = 0
            #prev_error = 0

            # Get running_error: 
            # if there is a sign change or its close to zero, otherwise keep adding the error
            if not(np.sign(x_error) + np.sign(self.prev_error)) or (abs(x_error) < 0.01):
                self.running_error = 0 
            else: 
                self.running_error += x_error
            delta_error = x_error - self.prev_error
            print("x_error: %f, running error: %f, prev_error: %f"%(x_error,self.running_error, self.prev_error))

            if abs(self.running_error) > 2000:
                self.running_error = np.sign(self.running_error)*2000
            # Calculate Motor Offset
            motor_offset = x_error * Kp + Ki * self.running_error + Kd * delta_error
            
            # Motor_offset, will range from [-1, 1] -> [Left, Right]
            #   Turning left means negative angular velocity, with counterclockwise rotation

            # Need to save previous error and running_error

            # TODO : Convert motor offsets to angular_vel

            if motor_offset < -1:
                motor_offset = -1
            if motor_offset > 1:
                motor_offset = 1

            #angular_vel = -.75*motor_offset
            angular_vel = -1*motor_offset
                
            """
            # if error is positive, we want to rotate clockwise
            if x_error<0:
                angular_vel = 0.5
            else:
                angular_vel = -0.5
            """        

        except rospy.ServiceException as e:
            print("get_measurement service call failed: %s"%e)

        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.prev_error = x_error
        
        if False:
            linear_vel = 0.0
            angular_vel = 0.0
        #linear_vel = 0.0
        #angular_vel = 0.0
        
        return motionLogicResponse(linear_vel, angular_vel)

    # shutdown
    def on_shutdown(self):
        print("motion_logic node shutdown")

# setup motion logic server
def server_motion_logic():
    print("initializing motion_logic node")
    rospy.init_node("motion_logic")
    motion_logic = Motion_Logic()    
    rospy.on_shutdown(motion_logic.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    server_motion_logic()

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
        # default remain on current trajectory
        linear_vel = self.linear_vel
        angular_vel = self.angular_vel

        rospy.wait_for_service("get_measurement")
        
        # get the image data from image_processing.py
        try:
            handle_measurement = rospy.ServiceProxy("get_measurement", measurement)
            meas = handle_measurement()

            # error from desired heading, used to calculate velocities
            x_error = meas.x_error

            # get the status of whether or not there is a object 
            red_obj_det = meas.red_obj_det

            # PID Parameters
            Kp = 0.006
            Ki = 0.003
            Kd = 0.1

            # running error is the error accumulation over time 
            # if there is a sign change or its close to zero, otherwise keep adding the error
            if not(np.sign(x_error) + np.sign(self.prev_error)) or (abs(x_error) < 0.01):
                self.running_error = 0 
            else: 
                self.running_error += x_error
            
            # error difference between this function call and the previous function call
            delta_error = x_error - self.prev_error
            
            rospy.loginfo("motion_logic: x_error: %f, running error: %f, prev_error: %f"%(x_error,self.running_error, self.prev_error))

            # saturate running error
            error_saturation = 1000 
            if abs(self.running_error) > error_saturation:
                self.running_error = np.sign(self.running_error) * error_saturation
            
            # calculate motor offset
            motor_offset = x_error * Kp + Ki * self.running_error + Kd * delta_error
            
            # saturate motor offset
            if motor_offset < -1:
                motor_offset = -1
            if motor_offset > 1:
                motor_offset = 1

            # convert motor offset to angular velocity
            # positive motor offset = turn left = negative angular velocity
            angular_vel = -1*motor_offset
                
        except rospy.ServiceException as e:
            rospy.loginfo("motion_logic: get_measurement service call failed: %s"%e)

        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.prev_error = x_error
        
        #linear_vel = 0.0
        #angular_vel = 0.0
        
        return motionLogicResponse(linear_vel, angular_vel)

    # shutdown
    def on_shutdown(self):
        rospy.loginfo("motion_logic: motion_logic node shutdown")

# setup motion logic server
def server_motion_logic():
    rospy.loginfo("motion_logic: initializing motion_logic node")
    rospy.init_node("motion_logic")
    motion_logic = Motion_Logic()    
    rospy.on_shutdown(motion_logic.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    server_motion_logic()

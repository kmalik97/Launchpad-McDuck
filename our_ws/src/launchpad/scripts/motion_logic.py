#!/usr/bin/env python
import rospy
import time
import numpy as np
import csv
from launchpad.srv import motionLogic, motionLogicResponse, measurement

# constant linear velocity
v = 0.45
class Motion_Logic:
    def __init__(self):
        self.service = rospy.Service("motionLogic", motionLogic, self.handle_motion_logic)
        self.running_error = 0
        self.prev_error = 0
        self.linear_vel = v
        self.angular_vel = 0.0
        self.just_stopped = False
        self.stop_time = time.time()
        self.csv_file = open('motion_logic_output.csv','w') # overwrite mode
        self.writer = csv.writer(self.csv_file, lineterminator = '\n')

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
            Ki = 0.001
            Kd = 0.01

            # running error is the error accumulation over time 
            # if there is a sign change or its close to zero, otherwise keep adding the error
            if not(np.sign(x_error) + np.sign(self.prev_error)) or (abs(x_error) < 0.01):
                self.running_error = 0 
            else: 
                self.running_error += x_error
            
            # error difference between this function call and the previous function call
            delta_error = x_error - self.prev_error
            
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
            # positive motor offset = turn right = negative angular velocity
            angular_vel = -motor_offset
            
            current_time = time.time()
        
            # stop at stop sign
            if red_obj_det and not self.just_stopped:
                linear_vel = 0.0
                angular_vel = 0.0
                self.just_stopped = True
                self.stop_time = time.time()
            elif self.just_stopped and current_time - self.stop_time <= 5:
                linear_vel = 0.0
                angular_vel = 0.0
            elif self.just_stopped and current_time - self.stop_time > 5 and current_time - self.stop_time < 8:
                linear_vel = v
            else:
                self.just_stopped = False
        
            # Save data to csv file
            try:
                self.writer.writerow([current_time,x_error])
            except Exception as E:
                rospy.loginfo(E)
                rospy.loginfo("--- Error occured during file save ---")
        
        except rospy.ServiceException as e:
            rospy.loginfo("motion_logic: get_measurement service call failed: %s"%e)
      
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.prev_error = x_error

        return motionLogicResponse(linear_vel, angular_vel)

    # shutdown
    def on_shutdown(self):
        self.csv_file.close()
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

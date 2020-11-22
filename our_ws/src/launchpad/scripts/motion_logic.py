#!/usr/bin/env python
import rospy
import sys
import tty
import termios
import numpy as np
from launchpad.srv import motionLogic, motionLogicResponse, measurement

# read char from terminal
# from https://github.com/magmax/python-readchar

running_error = None
prev_error = None

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class Motion_Logic:
    def __init__(self):
        self.service = rospy.Service("motionLogic", motionLogic, self.handle_motion_logic)

    # determine linear and angular velocity
    def handle_motion_logic(self, req):

        global running_error
        global prev_error

        if running_error == None:
            running_error = 0
            prev_error = 0



        # default is no motion
        linear_vel = 0.0
        angular_vel = 0.0

        # get the image data from image_processing.py
        rospy.wait_for_service("get_measurement")
        try:
            handle_measurement = rospy.ServiceProxy("get_measurement", measurement)
            # calculate the operating point
            pwm_left = req.pwm_left
            pwm_right = req.pwm_right
            y_operating = 120
            meas = handle_measurement(y_operating)
            print("meas: %f"%meas.x_error)
            # get the error from the desired heading
            # use x_error to determine velocities
            x_error = meas.x_error
            linear_vel = 0.5

            # PID Parameters
            Kp = 1
            Ki = 0
            Kd = 0

            # PlaceHolder Value
            #running_error = 0
            #prev_error = 0

            # Get running_error
            running_error = 0 if not((np.sign(x_error) or np.sign(prev_error)) or (
                        0.01 >= x_error >= -0.01)) else running_error + x_error
            delta_error = x_error - prev_error

            # Calculate Motor Offset
            motor_offset = x_error * Kp + Ki * running_error + Kd * delta_error

            # Motor_offset, will range from [-1, 1] -> [Left, Right]

            right_offset = 0
            left_offset = 0
            if motor_offset > 0:
                right_offset = motor_offset
                left_offset = -1 * motor_offset
            elif motor_offset < 0:
                right_offset = -1*motor_offset
                left_offset = motor_offset

            # Need to save previous error and running_error

            # TODO : Convert motor offsets to angular_vel

            # if error is positive, we want to rotate clockwise
            if x_error<0:
                angular_vel = 0.5
            else:
                angular_vel = -0.5

        except rospy.ServiceException as e:
            print("get_measurement service call failed: %s"%e)

        return motionLogicResponse(linear_vel, angular_vel)

    # shutdown
    def on_shutdown(self):
        print("motion_logic node shutdown")

# setup motion logic server
def server_motion_logic():
    print("initializing motion_logic node")
    print("gets here")
    rospy.init_node("motion_logic")
    motion_logic = Motion_Logic()    
    rospy.on_shutdown(motion_logic.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    server_motion_logic()

#!/usr/bin/env python
import rospy
import sys
import tty
import termios
from launchpad.srv import motionLogic, motionLogicResponse

# read char from terminal
# from https://github.com/magmax/python-readchar
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
        # default is no motion
        linear_vel = 0.0
        angular_vel = 0.0
        
        # get user input
        inp = readchar()

        # drive forward
        if inp == 'w':
            linear_vel = 1.0
        
        # drive backward
        if inp == 's':
            linear_vel = -1.0
        
        # rotate counter-clockwise
        if inp == 'a':
            angular_vel = 1.0

        # rotate clockwise
        if inp == 'd':
            angular_vel = -1.0

        # forward and right
        if inp == 'e':
            linear_vel = 1.0
            angular_vel = -1.0

        # forward and left
        if inp == 'q':
            linear_vel = 1.0
            angular_vel = 1.0

        # backward and right
        if inp == 'v':
            linear_vel = -1.0
            angular_vel = -1.0

        # backward and left
        if inp == 'z':
            linear_vel = -1.0
            angular_vel = 1.0
        
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

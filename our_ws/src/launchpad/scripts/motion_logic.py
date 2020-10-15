#!/usr/bin/env python
from __future__ import print_function
import rospy
from launchpad.srv import motionLogic, motionLogicResponse

# determine linear and angular velocity
def handle_motion_logic(req):
    # default is no motion
    linear_vel = 0.0
    angular_vel = 0.0
    
    # get user input
    inp = raw_input('enter command>> ')

    # drive forward
    if inp == 'w':
        linear_vel = 0.5

    # drive backward
    if inp == 's':
        linear_vel = -0.5

    # rotate counter-clockwise
    if inp == 'a':
        angular_vel = 1.0

    # rotate clockwise
    if inp == 'd':
        angular_vel = -1.0

    return motionLogicResponse(linear_vel, angular_vel)

# setup motion logic server
def server_motion_logic():
    print('initializing motion_logic node')
    rospy.init_node('motion_logic')
    my_service = rospy.Service('motionLogic', motionLogic, handle_motion_logic)
    rospy.spin()

if __name__ == '__main__':
    server_motion_logic()

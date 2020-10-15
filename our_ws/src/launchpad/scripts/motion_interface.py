#!/usr/bin/env python
from __future__ import print_function
import rospy
from launchpad.srv import motionLogic

# get motion logic and print it
def print_logic(event):
    # need to wait until service is available
    rospy.wait_for_service('motionLogic')

    try:
        # service callback function handle
        handle_motion_logic = rospy.ServiceProxy('motionLogic', motionLogic)

        # service response
        my_logic = handle_motion_logic()
        print("motion = [%.2f, %.2f]"%(my_logic.linear_vel, my_logic.angular_vel))

    except rospy.ServiceException as e:
        print("motionLogic service call failed: %s"%e)

# setup motion logic client
def client_motion_logic():
    print("initializing motion_interface node")
    rospy.init_node('motion_interface')
    # print logic every tenth of a second
    myTimer = rospy.Timer(rospy.Duration(0.1), print_logic)
    rospy.spin()

if __name__ == "__main__":
    client_motion_logic()

#!/usr/bin/env python
import rospy
import numpy as np
from math import floor
from Adafruit_MotorHAT import Adafruit_MotorHAT
from launchpad.srv import motionLogic

# define constants
LEFT_MIN = 80
LEFT_MAX = 255
LEFT_RES = LEFT_MAX - LEFT_MIN
RIGHT_MIN = 80
RIGHT_MAX = 255
RIGHT_RES = RIGHT_MAX - RIGHT_MIN

class Motion_Interface:
    def __init__(self):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.motor_left = self.motorhat.getMotor(1)
        self.motor_right = self.motorhat.getMotor(2)
        self.pwm_left = 0
        self.pwm_right = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.timer = rospy.Timer(rospy.Duration(0.01), self.set_motors)

    # set motors
    def set_motors(self, event):
        # get linear and angular velocity, both of which are in range [-1 1] (from motion_logic.py)
        self.get_velocity()

        # get motor PWM
        self.get_pwm()
        
        # set motor speed
        self.motor_left.setSpeed(abs(self.pwm_left))
        self.motor_right.setSpeed(abs(self.pwm_right))

        # set motor direction
        if self.pwm_left < 0:
            self.motor_left.run(Adafruit_MotorHAT.BACKWARD)
        elif self.pwm_left > 0:
            self.motor_left.run(Adafruit_MotorHAT.FORWARD)
        else:
            self.motor_left.run(Adafruit_MotorHAT.RELEASE)

        if self.pwm_right < 0:
            self.motor_right.run(Adafruit_MotorHAT.BACKWARD)
        elif self.pwm_right > 0:
            self.motor_right.run(Adafruit_MotorHAT.FORWARD)
        else:
            self.motor_right.run(Adafruit_MotorHAT.RELEASE)

    # determine motor PWM
    def get_pwm(self):    
        angular_gain = 0.2
        
        # decrease linear portion of PWM when turning
        linear_gain = 1 - angular_gain - 0.25 * (abs(self.angular_vel))
        
        # PWM due to linear velocity
        # left motor has reversed polarity
        pwm_linear_left = -np.sign(self.linear_vel) * floor(abs(self.linear_vel) * LEFT_RES + LEFT_MIN)
        pwm_linear_right = np.sign(self.linear_vel) * floor(abs(self.linear_vel) * RIGHT_RES + RIGHT_MIN)

        # PWM due to angular velocity
        pwm_angular_left = np.sign(self.angular_vel) * floor(abs(self.angular_vel) * LEFT_MAX)
        pwm_angular_right = np.sign(self.angular_vel) * floor(abs(self.angular_vel) * RIGHT_MAX)

        # need to negate angular PWM if driving backwards
        if self.linear_vel < 0:
            pwm_angular_left *= -1
            pwm_angular_right *= -1

        # total PWM 
        pwm_left = int(linear_gain * pwm_linear_left + angular_gain * pwm_angular_left)
        pwm_right = int(linear_gain * pwm_linear_right + angular_gain * pwm_angular_right)

        # do not let it drop below the minimum PWM values
        if abs(pwm_left) < LEFT_MIN:
            pwm_left = np.sign(pwm_left) * LEFT_MIN
        if abs(pwm_right) < RIGHT_MIN:
            pwm_right = np.sign(pwm_right) * RIGHT_MIN

        self.pwm_left = pwm_left
        self.pwm_right = pwm_right

    # get linear and angular velocity
    def get_velocity(self):
        # need to wait until service is available
        rospy.wait_for_service("motionLogic")

        try:
            # service callback function handle
            handle_motion_logic = rospy.ServiceProxy("motionLogic", motionLogic)

            # service response
            velocities = handle_motion_logic()
            self.linear_vel = velocities.linear_vel
            self.angular_vel = velocities.angular_vel
        except rospy.ServiceException as e:
            rospy.loginfo("motion_interface: motionLogic service call failed: %s"%e)

    # shutdown
    def on_shutdown(self):
        # set motor speed
        self.motor_left.setSpeed(0)
        self.motor_right.setSpeed(0)
        self.motor_left.run(Adafruit_MotorHAT.FORWARD)
        self.motor_right.run(Adafruit_MotorHAT.FORWARD)
        self.motor_left.run(Adafruit_MotorHAT.RELEASE)
        self.motor_right.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat
        rospy.loginfo("motion_interface: motion_interface node shut down")

# setup motion logic client
def client_motion_logic():
    rospy.loginfo("motion_interface: initializing motion_interface node")
    rospy.init_node("motion_interface")
    motion_interface = Motion_Interface()
    rospy.on_shutdown(motion_interface.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    client_motion_logic()

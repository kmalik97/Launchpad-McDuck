#!/usr/bin/env python
import rospy
from math import floor
from Adafruit_MotorHAT import Adafruit_MotorHAT
from launchpad.srv import motionLogic

# define constants
LEFT_MIN = 0
LEFT_MAX = 255
LEFT_RES = LEFT_MAX - LEFT_MIN
RIGHT_MIN = 0
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
        self.timer = rospy.Timer(rospy.Duration(0.1), self.set_motors)

    # set motors
    def set_motors(self, event):
        # get linear and angular velocity, both of which are in range [-1 1]
        self.get_velocity()

        # get motor PWM
        self.get_pwm()
        
        print("vel = [%.2f, %.2f], pwm = [%d, %d]"%(self.linear_vel, self.angular_vel, self.pwm_left, self.pwm_right))

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
        # fractional weighting of linear velocity to angular velocity
        gain = 0.75

        # PWM due to linear velocity
        pwm_linear_left = floor(self.linear_vel * LEFT_RES + LEFT_MIN)
        pwm_linear_right = floor(self.linear_vel * RIGHT_RES + RIGHT_MIN)

        # PWM due to angular velocity
        pwm_angular_left = floor(self.angular_vel * LEFT_RES + LEFT_MIN)
        pwm_angular_right = floor(self.angular_vel * RIGHT_RES + RIGHT_MIN)

        # need to negate angular PWM if driving backwards
        if self.linear_vel < 0:
            pwm_angular_left *= -1
            pwm_angular_right *= -1

        # total PWM
        self.pwm_left = -int(gain * pwm_linear_left - (1 - gain) * pwm_angular_left)
        self.pwm_right = int(gain * pwm_linear_right + (1 - gain) * pwm_angular_right)

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
            print("motionLogic service call failed: %s"%e)

    # shutdown
    def on_shutdown(self):
        self.motor_left.run(Adafruit_MotorHAT.RELEASE)
        self.motor_right.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat
        print("motion_interface node shut down")

# setup motion logic client
def client_motion_logic():
    print("initializing motion_interface node")
    rospy.init_node("motion_interface")
    motion_interface = Motion_Interface()
    rospy.on_shutdown(motion_interface.on_shutdown)
    rospy.spin()

if __name__ == "__main__":
    client_motion_logic()

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from fibre.utils import Event, Logger
from math import pi
import Jetson.GPIO as GPIO
import time

SPEED_LIMIT = 2000
MSG_PER_SECOND = 60
WD_FEED_PER_SECOND = 2

GPIO.setmode(GPIO.BOARD)
SHOULDER_LS_PIN = 12
ELBOW_LS_PIN = 15

class Driver():

    def __init__(self, timeout):
        # setup GPIO pins
        GPIO.setup(SHOULDER_LS_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ELBOW_LS_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # specify left, middle, and right ODrives
        rospy.loginfo("Looking for ODrives...")



        self.SERIAL_NUM = 35550393020494,  #TODO find serial number for arm odrive
        self.odrv = None
        self.zeropts = [None,None]

        # Get ODrives
        done_signal = Event(None)

        self.odrv = odrive.find_any()
        # Wait for ODrives

        rospy.loginfo("Found ODrives")

        # # Set axis state
        # rospy.logdebug("Setting velocity control")
        # self.odrv.watchdog_feed()

        # Clear errors
        dump_errors(self.odrv, True)

        self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

        self.zeros = self.find_zero()

        # Sub to topic
        rospy.Subscriber('joint_states', JointState, self.pos_callback)

        rospy.loginfo("Ready for topic")
        rospy.spin()

    def pos_callback(self, data):
        # Convert the angle from radians to encoder ticks
        shoulderPos = data.position[1]*42*300/2/pi + self.zeros[0] #TODO + const from lim switch to default
        elbowPos = data.position[2]*42*300/2/pi + self.zeros[1] #TODO + const from lim switch to default

        # Set the postion to the goal state
        self.odrv.axis0.controller.pos_setpoint = shoulderPos
        self.odrv.axis1.controller.pos_setpoint = elbowPos

        # Log positions
        rospy.loginfo("Set position of Shoulder axis to " + str(shoulderPos))
        rospy.loginfo("Set position of Elbow axis to " + str(elbowPos))

    def find_zero(self):
        pos0 = self.odrv.axis0.encoder.pos_estimate
        pos1 = self.odrv.axis1.encoder.pos_estimate
        axis0z = None
        axis1z = None
        print("Finding zeros")

        #Moves each axis down until they hit the limit switch, then records the result
        while (axis0z == None) or (axis1z == None):

            self.odrv.axis0.controller.pos_setpoint = pos0
            self.odrv.axis1.controller.pos_setpoint = pos1
            #TODO 
            if (not GPIO.input(SHOULDER_LS_PIN)) and (axis0z == None):
                axis0z = self.odrv.axis0.encoder.pos_estimate
                self.odrv.axis0.controller.pos_setpoint = axis0z

                print("Axis0: Zero position set")
            else:
                pos0 -= 1
            if (not GPIO.input(SHOULDER_LS_PIN)) and (axis1z == None):
                axis1z = self.odrv.axis1.encoder.pos_estimate
                self.odrv.axis1.controller.pos_setpoint = axis1z

                print("Axis1: Zero position set")
            else:
                pos1 -= 1
            time.sleep(0.01666666666/2)


        return [axis0z, axis1z]


    def clear_errors(self, odrv):
        dump_errors(odrv, True)


if __name__ == '__main__':
    rospy.init_node('driver', log_level=rospy.DEBUG)
    timeout = 2
    driver = Driver(timeout)

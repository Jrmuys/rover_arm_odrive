#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from fibre.utils import Event, Logger
from math import pi


SPEED_LIMIT = 2000
MSG_PER_SECOND = 60
WD_FEED_PER_SECOND = 2

class Driver():

    def __init__(self, timeout):
        
        # specify left, middle, and right ODrives
        rospy.loginfo("Looking for ODrives...")

        self.SERIAL_NUMS = 35593293288011,  #TODO find serial number for arm odrive

        self.odrvs = None

        self.zeropts = [None,None]

        # Get ODrives
        done_signal = Event(None)

        def discovered_odrv(obj):
            print("Found odrive with sn: {}".format(obj.serial_number))
            if obj.serial_number in self.SERIAL_NUMS:
                self.odrvs[self.SERIAL_NUMS.index(obj.serial_number)] = obj
                print("ODrive is # {}".format(self.SERIAL_NUMS.index(obj.serial_number)))
            else:
                print("ODrive sn not found in list. New ODrive?")
            if not None in self.odrvs:
                done_signal.set()

        odrive.find_all("usb", None, discovered_odrv, done_signal, None, Logger(verbose=False))
        # Wait for ODrives
        try:
            done_signal.wait(timeout=120)
        finally:
            done_signal.set()

        # self.odrv0 = odrive.find_any()
        # # odrv1 = odrive.find_any()
        # # odrv2 = odrive.find_any()
        rospy.loginfo("Found ODrives")

        # # Set axis state
        # rospy.logdebug("Setting velocity control")
        # for ax in (self.leftAxes + self.rightAxes):
        #     ax.watchdog_feed()

        # Clear errors
        dump_errors(self.odrvs, True)

        self.odrvs.axis0.controller.vel_ramp_enable = True
        self.odrvs.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrvs.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.odrvs.axis1.controller.vel_ramp_enable = True
        self.odrvs.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrvs.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

        self.zeros = find_zero(self)

        # Sub to topic
        rospy.Subscriber('joint_states', JointState, self.pos_callback)

        # # Set first watchdog
        # self.timeout = timeout  # log error if this many seconds occur between received messages
        # self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)
        # self.watchdog_fired = False

        # # Init other variables
        # self.last_msg_time = 0
        # self.last_recv_time = 0
        # self.next_wd_feed_time = 0

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
        rospy.loginfo("Set position of Shoulder axis to " + shoulderPos)
        rospy.loginfo("Set position of Elbow axis to " + elbowPos)
        """
                # # Notify of reconnection
                # if (self.watchdog_fired == True):
                #     self.watchdog_fired = False
                #     self.conn_lost_dur = rospy.Time.now() - self.conn_lost_time
                #     rospy.logwarn("Connection to controller reestablished! Lost connection for {} seconds.".format(self.conn_lost_dur.to_sec()))

                # --- Time BEGIN here
                # odrv_com_time_start = rospy.Time.now().to_sec()
                # # Read errors and feed watchdog at slower rate
                # if (recv_time > self.next_wd_feed_time):
                #     self.next_wd_feed_time = recv_time + 1.0/WD_FEED_PER_SECOND
                #     # Do stuff for all axes
                #     for ax in self.axes:
                #         ax.watchdog_feed()

                #         # TODO
                #         # # ODrive watchdog error clear
                #         # if(ax.error == errors.axis.ERROR_WATCHDOG_TIMER_EXPIRED):
                #         #     ax.error = errors.axis.ERROR_NONE
                #         #     rospy.logwarn("Cleared ODrive watchdog error")
                #         # For other errors
                #         if (ax.error != errors.axis.ERROR_NONE):
                #             rospy.logfatal("Received axis error: {} {}".format(self.axes.index(ax), ax.error))
                
                # -- Time STOP: Calculate time taken to reset ODrive
                # rospy.logdebug("Reseting each ODrive watchdog took {} seconds".format(rospy.Time.now().to_sec() - odrv_com_time_start))

                # # Emergency brake - 4 & 5 are bumpers
                # if (data.buttons[4] and data.buttons[5]):
                #     # Stop motors
                #     rospy.logdebug("Applying E-brake")
                #     for ax in (self.leftAxes + self.rightAxes):
                #         ax.controller.vel_ramp_target = 0
                #         ax.controller.vel_setpoint = 0
                # else:
                #     # Control motors as tank drive
                #     for ax in self.leftAxes:
                #         ax.controller.vel_ramp_target = data.axes[1] * SPEED_LIMIT
                #     for ax in self.rightAxes:
                #         ax.controller.vel_ramp_target = data.axes[4] * SPEED_LIMIT
                #     # -- Time STOP: Calculate time taken to reset ODrive
                #     rospy.logdebug("Communication with odrives took {} seconds".format(rospy.Time.now().to_sec() - odrv_com_time_start))

                # rospy.loginfo(rospy.get_caller_id() + "Left: %s", data.axes[1] * SPEED_LIMIT)
                # rospy.loginfo(rospy.get_caller_id() + "Right: %s", data.axes[4] * SPEED_LIMIT)

                # Received mesg so reset watchdog
                # self.timer.shutdown()
                # self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)

                # tot_time = rospy.Time.now().to_sec() - recv_time

                # --- Time STOP: Calculate time taken to reset ODrive
            #     rospy.logdebug("Callback execution took {} seconds".format(tot_time))

            # def watchdog_callback(self, event):
            #     # Have not received mesg for self.timeout seconds
            #     self.conn_lost_time = rospy.Time.now()
            #     rospy.logwarn("Control timeout! {} seconds since last control!".format(self.timeout))
            #     self.watchdog_fired = True

            #     # Stop motors
            #     for ax in self.leftAxes:
            #         ax.controller.vel_ramp_target = 0
            #         ax.controller.vel_setpoint = 0
            #     for ax in self.rightAxes:
            #         ax.controller.vel_ramp_target = 0
            #         ax.controller.vel_setpoint = 0
        """

    def find_zero(self):
        pos0, pos1 = 0
        axis0z, axis1z = None

        #Moves each axis down until they hit the limit switch, then records the result
        while axis0z == None | axis1z == None:
            self.odrv.axis0.controller.pos_setpoint = pos0
            self.odrv.axis1.controller.pos_setpoint = pos1
            #TODO 
            # if axis0 limit switch pressed:
            #     axis0z = self.odrv.axis0.encoder.pos_estimate
            # else:
            #     pos0 -= pos0
            # if axis1 limit switch pressed:
            #     axis1z = self.odrv.axis1.encoder.pos_estimate
            # else:
            #     pos1 -= pos1
        
        return [axis0z, axis1z]


    def clear_errors(self, odrv):
        dump_errors(odrv, True)


if __name__ == '__main__':
    rospy.init_node('driver', log_level=rospy.DEBUG)
    timeout = 2
    driver = Driver(timeout)

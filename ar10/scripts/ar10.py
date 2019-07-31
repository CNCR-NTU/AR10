#!/usr/bin/env python

from __future__ import print_function
# -*- coding: utf-8 -*-
"""
:ABSTRACT:
This script is part of the Enhaced grasp project

:REQUIRES:

:
:AUTHOR:  Pedro Machado
:ORGANIZATION: Nottingham Trent University
:CONTACT: pedro.baptistamachado@ntu.ac.uk
:SINCE: '13/03/2019'
:VERSION: 0.1

This file is part of Enhanced grasp project.
the Robot 2 Robot interaction project can not be copied and/or distributed without the express
permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk>

Copyright (C) 2019 All rights reserved, Nottingham Trent University
Computational Neuroscience and Cognitive Robotics Laboratory
email:  pedro.baptistamachado@ntu.ac.uk
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory


"""
# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Pedro Machado'
__contact__ = 'pedro.baptistamachado@ntu.ac.uk'
__copyright__ = 'Enhanced grasp project can not be copied and/or distributed \
without the express permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk'
__license__ = '2019 (C) CNCR@NTU, All rights reserved'
__date__ = '08/03/2019'
__version__ = '0.1'
__file_name__ = 'armHand.py'
__description__ = 'armHand class'
__compatibility__= "Python 2 and Python 3"
__platforms__= "AR10 hand"


# ===============================================================================
# IMPORT STATEMENTS
# ===============================================================================

import time
import serial
import csv
import os
import grasping_project.fingertips_config as fc

# ===============================================================================
# GLOBAL VARIABLES DECLARATIONS
# ===============================================================================

# ===============================================================================
# METHODS
# ===============================================================================
## \class ar10
#
# \brief AR10 hand class
class ar10:
    def __init__(self,device):
        self.__speed__ = 20
        self.__acceleration__ = 10

        self.__intercept__ = []
        self.__slope__ = []

        self.__max__=7950
        self.__min__=4200
        self.__finger_names__= ["thumb", "pinky", "ring", "middle", "index"]
        self.__used_joints__= [True, True, False, False, True, True, False, False, True, True]
        # When connected via USB, the Maestro creates two virtual serial ports
        # /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
        # Be sure the Maestro is configured for "USB Dual Port" serial mode.
        # "USB Chained Mode" may work as well, but hasn't been tested.

        # Pololu protocol allows for multiple Maestros to be connected. A device
        # number is used to index each connected unit.  This code currently is statically
        # configured to work with the default device 0x0C (or 12 in decimal).

        self.__usb__ = serial.Serial(device, baudrate=9600)
        # Command lead-in and device 12 are sent for each Pololu serial commands.
        self.__pololu_command__ = chr(0xaa) + chr(0xc)

        #  Read the calibration file
        if not os.path.isfile(os.path.dirname(os.path.realpath(__file__))+"/parameters/"+fc.FILE_NAME):
            print("Calibrating hand")
            self.__calibrate_hand__()

        cal_file = csv.reader(open(os.path.dirname(os.path.realpath(__file__))+"/parameters/"+fc.FILE_NAME), delimiter='\t')
        for row in cal_file:
            self.__intercept__.append(float(row[1]))
            self.__slope__.append(float(row[2]))

    ##Clean close function
    #
    # \details Cleanup by closing USB serial port
    def close(self):
        self.__usb__.close()

    ## \brief Change speed setting
    #
    # \param speed int between 1 and 60 (number of seconds to do the movement)
    def change_speed(self, speed):
        self.__speed__ = speed

    ## Set speed of channel
    #
    # \param channel the channel to update
    def set_speed(self, channel):
        lsb = self.__speed__ & 0x7f  # 7 bits for least significant byte
        msb = (self.__speed__ >> 7) & 0x7f  # shift 7 and take next 7 bits for msb
        # Send Pololu intro, device number, command, channel, speed lsb, speed msb
        command = self.__pololu_command__ + chr(0x07) + chr(channel) + chr(lsb) + chr(msb)
        self.__usb__.write(command)

    ## Change acceleration setting
    #
    # \param acceleration
    def change_acceleration(self, acceleration):
        self.__acceleration__ = acceleration

    ## \brief Set acceleration of channel
    #
    # \details This provide soft starts and finishes when servo moves to target position.
    def set_acceleration(self, channel, acceleration):
        lsb = acceleration & 0x7f  # 7 bits for least significant byte
        msb = (acceleration >> 7) & 0x7f  # shift 7 and take next 7 bits for msb
        # Send Pololu intro, device number, command, channel, acceleration lsb, acceleration msb
        command = self.__pololu_command__ + chr(0x09) + chr(channel) + chr(lsb) + chr(msb)
        self.__usb__.write(command)

    ## Set channel to a specified position target
    #
    # \details This function is a private class method, use move() or <finger_name>()
    # to actually move the hand
    def __set_target__(self, channel, target):
        lsb = target & 0x7f  # 7 bits for least significant byte
        msb = (target >> 7) & 0x7f  # shift 7 and take next 7 bits for msb
        # Send Pololu intro, device number, command, channel, and target lsb/msb
        command = self.__pololu_command__ + chr(0x04) + chr(channel) + chr(lsb) + chr(msb)
        self.__usb__.write(command)

    ## Convert joint number to channel number
    #
    # \details This function a private class method, is used by move()
    def __joint_to_channel__(self, joint):
        channel = joint + 10
        return channel

    ## Get the current position of the device on the specified channel
    #
    # \details This is not reading the true servo position, but the last target position sent
    # to the servo.  If the Speed is set to below the top speed of the servo, then
    # the position result will align well with the acutal servo position, assuming
    # it is not stalled or slowed.
    # \return The result is returned in a measure of quarter-microseconds, which mirrors
    # the Target parameter of set_target.
    def get_set_position(self, joint):
        # convert joint to channel
        channel = self.__joint_to_channel__(joint)

        command = self.__pololu_command__ + chr(0x10) + chr(channel)
        self.__usb__.write(command)
        lsb = ord(self.__usb__.read())
        msb = ord(self.__usb__.read())

        return (msb << 8) + lsb

    ## Check if servo outputs Have reached their targets
    #
    # \details This function is the equivalent of a private class method, is used by get_position()
    def get_read_position(self, channel):
        command = self.__pololu_command__ + chr(0x90) + chr(channel)
        self.__usb__.write(command)
        lsb = ord(self.__usb__.read())
        msb = ord(self.__usb__.read())
        read_position = (256 * msb) + lsb

        return read_position

    ## Check if servo outputs Have reached their targets
    #
    # \details This is useful only if Speed and/or Acceleration have been set on one or more of the channels.
    # \param channel the channel to check, as given by joint_to_channel()
    # \return True or False, wether if targets have been reached or not
    def get_position(self, channel):
        read_position = self.get_read_position(channel)
        position = self.__intercept__[channel] + (self.__slope__[channel] * read_position)

        return position

    def get_fingers_position(self):
        angles=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        for i in range(0,len(angles)):
            read_position = self.get_read_position(i)
            angles[i] = self.__intercept__[i] + (self.__slope__[i] * read_position)
        return angles
    ## Check if servo outputs Have reached their targets
    #
    # \details This is useful only if Speed and/or Acceleration have been set on one or more of the channels.
    # \return True or False, wether if targets have been reached or not
    def get_moving_state(self):
        command = self.__pololu_command__ + chr(0x13) + chr(0x01)
        self.__usb__.write(command)
        if self.__usb__.read() == chr(0):
            return False
        else:
            return True

    ## Run a Maestro Script subroutine in the currently active script.
    #
    # \details Scripts can have multiple subroutines, which get numbered sequentially from 0 on up.  Code your
    # Maestro subroutine to either infinitely loop, or just end (return is not valid).
    def run_script(self, subNumber):
        command = self.__pololu_command__ + chr(0x27) + chr(subNumber)
        # can pass a param with comman 0x28
        #  command = self.pololu_command + chr(0x28) + chr(subNumber) + chr(lsb) + chr(msb)
        self.__usb__.write(command)

    ## Stop the current Maestro Script
    def stop_script(self):
        command = self.__pololu_command__ + chr(0x24)
        self.__usb__.write(command)

    ## Move joint to target position
    #
    # \details Main function, simplified with the <finger_name>() functions
    # \param joint number of the joint to move
    # \param target value of the position to reach
    def move_joint(self, joint, target):
        # convert joint to channel
        channel = self.__joint_to_channel__(joint)

        # check target position is in range
        if target > self.__max__:
            target = self.__max__
        elif target < self.__min__:
            target = self.__min__
        target=int(target)

        # a speed of 1 will take 1 minute
        # a speed of 60 would take 1 second.
        # Speed of 0 is unlimited
        self.set_speed(channel)
        # Valid values are from 0 to 255. 0=unlimited, 1 is slowest start.
        # A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
        self.set_acceleration(channel, self.__acceleration__)
        # Valid servo range is 256 to 3904
        self.__set_target__(channel, target)

    ## Move a finger to target position
    #
    # \param base target for the first articulation
    # \param second target for the second articulation
    def move_finger(self, name, lower, upper):
        for i in range(0,len(self.__finger_names__)):
            if self.__finger_names__[i]==name:
                if self.__used_joints__[i*2]:
                    self.move_joint(i * 2, lower)
                if self.__used_joints__[i * 2]:
                    self.move_joint(i * 2 + 1, upper)

    ## Move a fingers to target positions
    #
    # \param base target for the first articulation
    # \param second target for the second articulation
    def move_fingers(self, vector):
        if len(vector)==len(self.__used_joints__):
            # print("Moving the hand....")
            for i in range(len(vector)):
                if self.__used_joints__[i]:
                    self.move_joint(i, vector[i])

    ## Wait for joints to stop moving
    def wait_for_hand(self):
        while self.get_moving_state():
            time.sleep(0.25)

    ## open hand
    def __open_hand__(self):
        #Open thumb first
        for i in range(0,2):
            self.move_joint(i, 8000)
        time.sleep(1.0)
        #open the the remaining joints
        for joint in range(2, 10):
            self.move_joint(joint, 8000)
        self.wait_for_hand()

    def __calibrate_joint__(self, joint):
        n_points = 0

        sum_x = 0.0
        sum_y = 0.0
        sum_xy = 0.0
        sum_xx = 0.0

        if joint == 9:
            self.move_joint(8, 5500)

        for target in range(4500, 8000, 500):
            self.move_joint(joint, target)
            self.wait_for_hand()
            time.sleep(2.0)

            position = self.get_read_position(joint)

            n_points = n_points + 1

            sum_x = sum_x + position
            sum_y = sum_y + target
            sum_xy = sum_xy + (position * target)
            sum_xx = sum_xx + (position * position)

        slope = ((sum_x * sum_y) - (n_points * sum_xy)) / ((sum_x * sum_x) - (n_points * sum_xx))
        y_intercept = (sum_y - (slope * sum_x)) / n_points

        self.move_joint(joint, 7950)
        self.wait_for_hand()

        if joint == 9:
            self.move_joint(8, 7950)
            self.wait_for_hand()

        return y_intercept, slope

    def __calibrate_hand__(self):

        # open calibration file
        cal_file = open(os.path.dirname(os.path.realpath(__file__))+"/parameters/"+fc.FILE_NAME, "w+")

        self.__open_hand__()

        for joint in range(0, 10):
            y_intercept, slope = self.__calibrate_joint__(joint)
            print("joint = " + str(joint) + " and intercept = " + str(y_intercept) + " slope = " + str(slope))
            cal_file.write(str(joint))
            cal_file.write("\t")
            cal_file.write(str(y_intercept))
            cal_file.write("\t")
            cal_file.write(str(slope))
            cal_file.write("\n")

        # close calibration file
        cal_file.close()

    def __del__(self):
        pass
        # print("AR10 Hand closed with success!")

#===============================================================================
#  TESTING AREA
#===============================================================================
# def test():
#     ar10_hand=ar10()
#     ar10_hand.close()
#     del ar10_hand




#===============================================================================
# MAIN METHOD
#===============================================================================
# if __name__ == '__main__':
#     test()
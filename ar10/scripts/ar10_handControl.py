#!/usr/bin/env python
from __future__ import print_function

# -*- coding: utf-8 -*-
"""
:ABSTRACT:
This script is part of the of the enhanced grasp project

:REQUIRES:

:
:AUTHOR:  Pedro Machado
:ORGANIZATION: Nottingham Trent University
:CONTACT: pedro.baptistamachado@ntu.ac.uk
:SINCE: 31/07/2019
:VERSION: 0.1

2019 (c) GPLv3, Nottingham Trent University
Computational Neuroscience and Cognitive Robotics Laboratory
email:  pedro.baptistamachado@ntu.ac.uk
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory


"""
# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Pedro Machado'
__contact__ = 'pedro.baptistamachado@ntu.ac.uk'
__copyright__ = '2019 (C) GPLv3, CNCR@NTU, Prof. Martin McGinnity martin.mcginnity@ntu.ac.uk'
__license__ = 'GPLv3'
__date__ = '31/07/2019'
__version__ = '1.0'
__file_name__ = 'ar10_handControl.py'
__description__ = 'Subscribe the ar10 topic and controls the hand'
__compatibility__ = "Python 2 and Python 3"
__platforms__ = "i386, x86_64, arm32 and arm64"
__diff__= "GPLv3"

#===============================================================================
# IMPORT STATEMENTS
#===============================================================================
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import os
import ar10 as ar
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

#===============================================================================
# GLOBAL VARIABLES DECLARATIONS
#===============================================================================
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

PATH=os.path.dirname(os.path.realpath(__file__))+"/parameters/"
#===============================================================================
# METHODS
#===============================================================================
def callback_ar10_control(data):
   pass

def publish_pose(pub0):
    global ar10
    handRealPos = np.asarray(ar10.get_fingers_position(), dtype=np.float32)
    pub0.publish(handRealPos.flatten('F'))

def callback_ar10_requests(data,pub0):
    print(data.data)
    if data.data=="pose":
        publish_pose(pub0)



def listener():
    global ar10
    while not rospy.is_shutdown():
        try:
            pub0 = rospy.Publisher('actuators/ar10/pose', numpy_msg(Floats), queue_size=10)
            print("Subscribing: /actuators/ar10/control")
            print("Subscribing: /actuators/ar10/requests")
            print("AR10 pose published in topic: /actuators/ar10/pose.")
            rospy.Subscriber("actuators/ar10/control", numpy_msg(Floats), callback_ar10_control)
            rospy.Subscriber("actuators/ar10/requests", String, callback_ar10_requests,(pub0))
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down the Biotac subscriber!")
        except IOError:
            print(IOError)
            print("Shuting down the Biotac subscriber!")
    del ar10
#===============================================================================
#  TESTING AREA
#===============================================================================

#===============================================================================
# MAIN METHOD
#===============================================================================
if __name__ == '__main__':
    global ar10
    print("[Initialising AR10 controller...]\n")
    rospy.init_node('ar10_controller', anonymous=True)
    device = ""
    items = os.listdir("/dev")
    ct = 0
    fg = True
    while fg and ct < 10:
        for i in items:
            if i == "ttyACM" + str(ct):
                fg = False
                device = "ttyACM" + str(ct)
                break
        ct += 1
    print("/dev/" + device)
    if len(device) == 0 and ct == 10:
        print("ERROR: Device not found. Please switch on the AR10.")
    ar10 = ar.ar10("/dev/" + device)
    listener()

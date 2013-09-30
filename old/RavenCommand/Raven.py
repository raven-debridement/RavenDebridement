#!/usr/bin/env python

import roslib; roslib.load_manifest('RavenDebridement')
import os, sys
import rospy

import tfx
from raven_2_msgs.msg import *

# rename so no conflict with raven_2_msgs.msg.Constants
from RavenDebridement.Utils import Constants as MyConstants
from RavenDebridement.RavenCommand import RavenArm

class Raven():
    def __init__(self,createLeftArm=False,createRightArm=False):
        self.leftArm = None
        self.rightArm = None
        
        if createLeftArm:
            self.leftArm = RavenArm(MyConstants.Arm.Left)
        if createRightArm:
            self.rightArm = RavenArm(MyConstants.Arm.Right)
        
    def arm(self, armName):
        if armName == MyConstants.Arm.Left:
            return self.leftArm
        elif armName == MyConstants.Arm.Right:
            return self.rightArm
        
        return None
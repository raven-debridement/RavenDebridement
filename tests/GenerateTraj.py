#!/usr/bin/env python

import roslib; roslib.load_manifest('RavenDebridement')
import numpy as np
import os, sys
import random
import math

import rospy

import tf
import tf.transformations as tft

from geometry_msgs.msg import *
from std_msgs.msg import Header

import code
import IPython

import tfx
from raven_2_msgs.msg import *

import openravepy as rave

# rename so no conflict with raven_2_msgs.msg.Constants
from RavenDebridement.Utils import Constants as MyConstants
from RavenDebridement.Utils import Util
from RavenDebridement.RavenCommand.RavenPlanner import RavenPlanner, Request
from RavenDebridement.RavenCommand.RavenArm import RavenArm

class GenerateTraj():
    def __init__(self, arm=MyConstants.Arm.Right):
        self.arm = arm
        
        self.ravenArm = RavenArm(self.arm)
        self.ravenPlanner = RavenPlanner(self.arm)
        
    def generateTraj(self, deltaPose):
        startPose = tfx.pose(self.ravenArm.getGripperPose())
        deltaPose = tfx.pose(deltaPose)
        
        #code.interact(local=locals())
        
        endPose = Util.endPose(startPose, deltaPose)
        #endPose = startPose
        
        #IPython.embed()
        
        jointTraj = self.ravenPlanner.getTrajectoryFromPose(endPose,n_steps=15)
        
        return jointTraj
    
def testGenerateTraj():
    rospy.init_node('generate_traj_node',anonymous=True)
    gt = GenerateTraj()
    rospy.sleep(2)
    deltaPose = tfx.pose([.04,0,0],frame=MyConstants.Frames.Link0)
    #IPython.embed()
    jointTraj = gt.generateTraj(deltaPose)
    for joints in jointTraj:
        positions = []
        for rosJointType in gt.ravenPlanner.rosJointTypes:
            positions.append(joints[rosJointType])
        print(positions)
    IPython.embed()
    
if __name__ == '__main__':
    testGenerateTraj()
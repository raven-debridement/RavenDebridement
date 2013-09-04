#!/usr/bin/env python


import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from math import *
import copy
import sys, os

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *


import openravepy as rave

import tf
import tfx
import numpy as np

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants
from RavenDebridement.RavenCommand.RavenPlanner2 import RavenPlanner
from RavenDebridement.RavenCommand.kinematics import *

import IPython

def testFK():
    rospy.init_node('test_IK',anonymous=True)
    rp = RavenPlanner(MyConstants.Arm.Both)
    rp.env.SetViewer('qtcoin')
    rospy.sleep(2)

    leftPose = rp.getCurrentPose(MyConstants.Arm.Left)
    rightPose = rp.getCurrentPose(MyConstants.Arm.Right)
    
    leftGrasp = rp.getCurrentGrasp(MyConstants.Arm.Left)
    rightGrasp = rp.getCurrentGrasp(MyConstants.Arm.Right)

    leftJoints = invArmKin(MyConstants.Arm.Left, leftPose, leftGrasp)
    rightJoints = invArmKin(MyConstants.Arm.Right, rightPose, rightGrasp)


    rp.updateOpenraveJoints(MyConstants.Arm.Left, joints1=leftJoints, grasp=leftGrasp)
    rp.updateOpenraveJoints(MyConstants.Arm.Right, joints1=rightJoints, grasp=rightGrasp)
    
    raveLeftMatrix = Util.openraveTransformFromTo(rp.robot, np.eye(4), MyConstants.Frames.LeftTool, MyConstants.Frames.Link0)
    raveRightMatrix = Util.openraveTransformFromTo(rp.robot, np.eye(4), MyConstants.Frames.RightTool, MyConstants.Frames.Link0)
    
    raveLeftPose = tfx.pose(raveLeftMatrix,frame=leftPose.frame)
    raveRightPose = tfx.pose(raveRightMatrix,frame=rightPose.frame)
    
    deltaLeft = Util.deltaPose(leftPose, raveLeftPose)
    deltaRight = Util.deltaPose(rightPose, raveRightPose)
    
    g = []
    #g += Util.plot_transform(rp.env, Util.openraveTransformFromTo(rp.robot, np.eye(4), MyConstants.Frames.LeftTool, 'world'))
    #g += Util.plot_transform(rp.env, Util.openraveTransformFromTo(rp.robot, np.eye(4), MyConstants.Frames.RightTool, 'world'))
    g += Util.plot_transform(rp.env, Util.openraveTransformFromTo(rp.robot, leftPose.matrix, leftPose.frame[1:], 'world'))
    g += Util.plot_transform(rp.env, Util.openraveTransformFromTo(rp.robot, rightPose.matrix, rightPose.frame[1:], 'world'))
    
    
    IPython.embed()
    


if __name__ == '__main__':
    testFK()
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
from RavenDebridement.RavenCommand.RavenPlanner import RavenPlanner

import IPython

def testFK():
    rospy.init_node('test_IK',anonymous=True)
    rp = RavenPlanner(MyConstants.Arm.Right)
    rp.env.SetViewer('qtcoin')
    rospy.sleep(2)

    currPose = tfx.pose([0,0,0], frame=rp.toolFrame)
    tf_tool_to_link0 = tfx.lookupTransform(MyConstants.Frames.Link0, currPose.frame, wait=5)
    currPose = tf_tool_to_link0 * currPose

    joints = rp.currentJoints
    for jointType, jointPos in joints.items():
        print("jointType = {0}, jointPos = {1}".format(jointType,((180.0/pi)*jointPos)))

    rp.updateOpenraveJoints()
    
    
    refLinkName = currPose.frame
    targLinkName = 'world'
    
    # ref -> world
    refFromWorld = rp.robot.GetLink(refLinkName).GetTransform()
    
    # target -> world
    targFromWorld = rp.robot.GetLink(targLinkName).GetTransform()
    
    # target -> ref
    targFromRef = np.dot(np.linalg.inv(targFromWorld), refFromWorld)
    
    newCurrPoseMat = np.dot(targFromRef, np.array(currPose.matrix))
    currPose = tfx.pose(newCurrPoseMat, frame=targLinkName)
    """
    refLinkName = rp.toolFrame
    targLinkName = MyConstants.Frames.Link0
    # world <- ref
    refLink = rp.robot.GetLink(refLinkName)
    worldFromRefLink = refLink.GetTransform()

    # world <- targ
    targLink = rp.robot.GetLink(targLinkName)
    worldFromTargLink = targLink.GetTransform()

    # targ <- EE
    worldFromEE = rp.manip.GetEndEffectorTransform()
    targLinkFromEE = np.dot(np.linalg.inv(worldFromTargLink), worldFromEE)
    
    ravePose = tfx.pose(targLinkFromEE)
    ravePose.position.z-=.01
    """
    ravePose = tfx.pose(rp.robot.GetLink(rp.toolFrame).GetTransform())
    

    print('currPose')
    print(currPose.position)
    print(currPose.tb_angles)
    
    print('openrave pose')
    print(ravePose.position)
    print(ravePose.tb_angles)

    deltaPose = tfx.pose(Util.deltaPose(currPose,ravePose))
    print('delta pose')
    print(deltaPose.position)
    print(deltaPose.tb_angles)
    
    
    g = []
    g += Util.plot_transform(rp.env, np.array(ravePose.matrix))
    
    
    IPython.embed()

if __name__ == '__main__':
    testFK()
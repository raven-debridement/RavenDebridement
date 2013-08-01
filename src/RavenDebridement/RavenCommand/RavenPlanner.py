#!/usr/bin/env python

"""
Adapted from Ben Kehoe's trajectory_player
"""

import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from math import *
import copy
import sys, os

import openravepy as rave

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *

import openravepy as rave
import numpy as np


import tf
import tfx

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants

import code

class RavenPlanner():

    """
    Joint order:
    shoulder
    elbow
    insertion
    tool_roll
    wrist_joint
    wrist_bend
    grasper_joint_1
    grasper_joint_2

    add '_L' or '_R' depending on arm
    
    rosNameToRaveName = {Constants.JOINT_TYPE_SHOULDER : "shoulder",
                         Constants.JOINT_TYPE_ELBOW    : "elbow",
                         Constants.JOINT_TYPE_INSERTION
    """
    raveJointNames = ["shoulder",
                      "elbow",
                      "insertion",
                      "tool_roll",
                      "wrist_joint",
                      "wrist_bend"]

    rosJointIndices = [Constants.JOINT_TYPE_SHOULDER,
                       Constants.JOINT_TYPE_ELBOW,
                       Constants.JOINT_TYPE_INSERTION,
                       Constants.JOINT_TYPE_ROTATION,
                       Constants.JOINT_TYPE_YAW,
                       Constants.JOINT_TYPE_PITCH]

    def __init__(self, armName=MyConstants.Arm.Left):
        """
        openrave planner for the raven
        """
        rospy.loginfo('Initializing openrave planner for ' + armName + ' raven arm')

        self.armName = armName

        self.currentState = None
        self.jointStates = None

        rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)


        self.env = rave.Environment()
        ravenFile = os.path.dirname(__file__) + '/../../../models/raven2.zae'
        self.env.Load(ravenFile)
        self.robot = self.env.GetRobots()[0]
        self.manip = self.robot.GetActiveManipulator()

        ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=rave.IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        self.myRaveJointNames = [raveJointName + '_' + self.armName[0].upper() for raveJointName in self.raveJointNames]
        self.raveJointIndices = [self.robot.GetJointIndex(name) for name in self.myRaveJointNames]
        self.raveJointIndicesToRos = dict((rave,ros) for rave,ros in zip(self.raveJointIndices, self.rosJointIndices))
        self.rosJointIndicesToRave = dict((ros,rave) for ros,rave in zip(self.rosJointIndices, self.raveJointIndices))
        


    def _ravenStateCallback(self, msg):
        self.currentState = msg
        for arm in msg.arms:
            if arm.name == self.armName:
                self.jointStates = arm.joints
                
    def updateRave(self):
        if self.jointStates == None:
            return False

        jointStates = self.jointStates

        self.positions = []
        indices = []
        for jointState in jointStates:
            if self.rosJointIndicesToRave.has_key(jointState.type):
                self.positions.append(jointState.position)
                indices.append(self.rosJointIndicesToRave[jointState.type])
            
        
        self.robot.SetJointValues(self.positions, indices)
            
        return True

    def getRosJointTypes(self):
        return list(self.rosJointIndices)

    def getCurrentJointPositions(self):
        return list(self.positions)

    def getJointPositionsFromPose(self, endPose):
        if not self.updateRave():
            return

        endPose = Util.convertToFrame(tfx.pose(endPose), MyConstants.Frames.Link0)
        endMatrix = np.array(endPose.matrix)

        solution = self.manip.FindIKSolution(endMatrix, rave.IkFilterOptions.CheckEnvCollisions)

        code.interact(local=locals())

        return solution
        
        


def testRavenPlanner():
    rospy.init_node('test_raven_planner',anonymous=True)
    rp = RavenPlanner()

    while not rospy.is_shutdown() and rp.jointStates == None:
        rospy.sleep(.1)


    rp.getJointPositionsFromPose(tfx.pose([-.015, -.014, -.069],tfx.tb_angles(-159.1,74.9,-84.8)))

    rospy.spin()

if __name__ == '__main__':
    testRavenPlanner()

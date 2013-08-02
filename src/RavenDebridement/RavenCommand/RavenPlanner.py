#!/usr/bin/env python


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
import trajoptpy
import json
import trajoptpy.kin_utils as ku
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
                       Constants.JOINT_TYPE_PITCH,
                       Constants.JOINT_TYPE_YAW]

    def __init__(self, armName=MyConstants.Arm.Left):
        """
        openrave planner for the raven
        """
        rospy.loginfo('Initializing openrave planner for ' + armName + ' raven arm')

        self.armName = armName

        self.currentState = None
        self.jointStates = None

        self.jointPositions = [0 for _ in range(len(self.rosJointIndices))]
        self.jointTypes = list(self.rosJointIndices)

        rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)


        self.env = rave.Environment()

        # TEMP
        #self.env.SetViewer('qtcoin')

        ravenFile = os.path.dirname(__file__) + '/../../../models/ravens.xml' #'/../../../models/raven2.zae'
        self.env.Load(ravenFile)
        self.robot = self.env.GetRobots()[0]
        self.robot.SetActiveManipulator(armName[0].lower() + '_arm')
        self.manip = self.robot.GetActiveManipulator()

        ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=rave.IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        self.myRaveJointNames = [armName[0].lower() + '_' + raveJointName + '_L' for raveJointName in self.raveJointNames]
        #self.myRaveJointNames = [raveJointName + '_L' for raveJointName in self.raveJointNames]
        self.raveJointIndices = [self.robot.GetJointIndex(name) for name in self.myRaveJointNames]
        self.raveJointIndicesToRos = dict((rave,ros) for rave,ros in zip(self.raveJointIndices, self.rosJointIndices))
        self.rosJointIndicesToRave = dict((ros,rave) for ros,rave in zip(self.rosJointIndices, self.raveJointIndices))
        
        for link in self.robot.GetLinks():
            print(link.GetName())

        while self.jointStates == None:
            rospy.loginfo('Waiting for first joint state message...')
            rospy.sleep(.5)
        #code.interact(local=locals())


    def _ravenStateCallback(self, msg):
        self.currentState = msg
        for arm in msg.arms:
            if arm.name == self.armName:
                self.jointStates = arm.joints
                
    def updateRave(self):
        if self.jointStates == None:
            return False

        jointStates = self.jointStates

        jointPositions = []
        jointTypes = []
        raveIndicies = []
        for jointState in jointStates:
            if self.rosJointIndicesToRave.has_key(jointState.type):
                jointPositions.append(jointState.position) # not sure why negative, but looks right
                raveIndices.append(self.rosJointIndicesToRave[jointState.type])
            
        
        #code.interact(local=locals())
        self.robot.SetJointValues(jointPositions, raveIndices)
            
        return True

    def getRosJointTypes(self):
        return list(self.rosJointIndices)

    def getCurrentJointPositions(self):
        return list(self.positions)

    def getJointPositionsFromPose(self, endPose):
        if not self.updateRave():
            rospy.loginfo('Failed to update')
            return

        endPose = Util.convertToFrame(tfx.pose(endPose), MyConstants.Frames.RightBase)
        endMatrix = np.array(endPose.matrix)

        #solution = self.manip.FindIKSolution(endMatrix, rave.IkFilterOptions.CheckEnvCollisions)
        init_joint_target = ku.ik_for_link(endMatrix, self.manip, "r_link1_L", filter_options=rave.IkFilterOptions.CheckEnvCollisions)

        code.interact(local=locals())

        
        


def testRavenPlanner():
    rospy.init_node('test_raven_planner',anonymous=True)
    rp = RavenPlanner(MyConstants.Arm.Right)

    while not rospy.is_shutdown() and rp.jointStates == None:
        rospy.sleep(.1)

    rospy.loginfo('Getting joint positions')
    rp.getJointPositionsFromPose(tfx.pose([0,0,0],frame=MyConstants.Frames.RightTool))

    rospy.spin()

if __name__ == '__main__':
    testRavenPlanner()

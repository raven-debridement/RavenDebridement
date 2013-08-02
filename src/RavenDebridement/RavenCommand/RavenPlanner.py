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
#import trajoptpy
import json
#import trajoptpy.kin_utils as ku
import numpy as np


import tf
import tfx

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants

import code

class Request():
    """
    Class for the trajopt json requests
    """
    def __init__(self, n_steps=20):
        self.n_steps = n_steps
        

    def stayHigh(self, armName, toolFrame, pose, initialJoints):
        pose = tfx.pose(pose)
        
        desPos = pose.position.list
        desQuat = pose.orientation.list

        # request isn't correct
        # it doesn't force arm to stay high
        request = {
            "basic_info" : {
                "n_steps" : self.n_steps,
                "manip" : armName,
                "start_fixed" : True
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]}
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20],
                        "dist_pen" : [0.025]
                        }
                    },
                {
                        "type" : "pose",
                        "name" : "target_pose",
                        "params" : {"xyz" : desPos,
                                    "xyzw" : desQuat,
                                    "link": toolFrame,
                                    "rot_coeffs" : [1,1,0],
                                    "pos_coeffs" : [0,0,0],
                                    "coeffs" : [20]
                                    }
                        },
                ],
            "constraints" : [
                # BEGIN pose_target
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : desPos,
                                "xyzw" : desQuat,
                                "link": toolFrame,
                                "rot_coeffs" : [0,0,0],
                                "pos_coeffs" : [1,1,1]
                                }
                    
                    },
                #END pose_target
                ],
            "init_info" : {
                "type" : "straight_line",
                "endpoint" : initialJoints
                }
            }

class RavenPlanner():

    """
    Joint order:
    shoulder
    elbow
    insertion
    tool_roll
    wrist_joint
    grasper_joint_1
    grasper_joint_2
    """
    rosJointIndices = [Constants.JOINT_TYPE_SHOULDER,
                       Constants.JOINT_TYPE_ELBOW,
                       Constants.JOINT_TYPE_INSERTION,
                       Constants.JOINT_TYPE_ROTATION,
                       Constants.JOINT_TYPE_PITCH,
                       Constants.JOINT_TYPE_GRASP_FINGER1]

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

        self.refFrame = MyConstants.Frames.Link0

        #rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)


        self.env = rave.Environment()

        # TEMP
        #self.env.SetViewer('qtcoin')

        ravenFile = os.path.dirname(__file__) + '/../../../models/ravenII_2arm.xml'
        self.env.Load(ravenFile)

        self.robot = self.env.GetRobots()[0]
        if self.armName == MyConstants.Arm.Left:
            self.toolFrame = MyConstants.OpenraveLinks.LeftGrasper1
            self.robot.SetActiveManipulator('left_arm')
        else:
            self.toolFrame = MyConstants.OpenraveLinks.RightGrasper1
            self.robot.SetActiveManipulator('right_arm')
        self.manip = self.robot.GetActiveManipulator()
        self.manipJoints = self.robot.GetJoints(self.manip.GetArmJoints())

        rospy.loginfo('Loading model')
        ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=rave.IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        rospy.loginfo('Model loaded')

        self.raveJointNames = [str(joint.GetName()) for joint in self.manipJoints]

        self.raveJointIndices = list(self.manip.GetArmIndices())
        self.raveJointIndicesToRos = dict((rave,ros) for rave,ros in zip(self.raveJointIndices, self.rosJointIndices))
        self.rosJointIndicesToRave = dict((ros,rave) for ros,rave in zip(self.rosJointIndices, self.raveJointIndices))
        
        rospy.loginfo('Links')
        for link in self.robot.GetLinks():
            print(link.GetName())

        rospy.loginfo('Joints')
        for joint in self.robot.GetJoints():
            print(joint.GetName())

        rospy.loginfo('Independent links')
        indLinks = self.manip.GetIndependentLinks()
        for link in indLinks:
            print(link.GetName())

        rospy.loginfo('Dependent links')
        for link in self.robot.GetLinks():
            if link not in indLinks:
                print(link.GetName())

        self.robot.SetJointValues(self.getCurrentJointPositions(), self.raveJointIndices)
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
        # stub
        return list((pi/180.0)*np.array([21.9,95.4,-7.5,20.4,-29.2,14.6]))
        #return list(self.positions)

    def transformRelativePoseForIk(poseMatrix, refLinkName, targLinkName):
        """
        Adapted from PR2.py

        Returns transformed poseMatrix that can be used in openrave IK
        """
        # world <- ref
        refLink = self.robot.GetLink(refLinkName)
        worldFromRefLink = refLink.GetTransform()

        # world <- targ
        targLink = self.robot.GetLink(targLinkName)
        worldFromTargLink = targLink.GetTransform()

        # targ <- EE
        worldFromEE = self.manip.GetEndEffectorTransform()
        targLinkFromEE = np.dot(np.linalg.inv(worldFromTargLink), worldFromEE)

        # world <- EE
        refLinkFromTargLink = poseMatrix
        newWorldFromEE = np.dot(np.dot(worldFromRefLink, refLinkFromTargLink), targLinkFromEE)

        return newWorldFromEE

    def getJointPositionsFromPose(self, endPose):
        if not self.updateRave():
            rospy.loginfo('Failed to update')
            return

        endPose = Util.convertToFrame(tfx.pose(endPose), self.toolFrame)
        endMatrix = np.array(endPose.matrix)

        endMatrix = self.transformRelativePoseForIk(endMatrix, endPose.frame, self.toolFrame)
        endJointPositions = self.manip.FindIKSolution(endMatrix, rave.IkFilterOptions.CheckEnvCollisions)
        #init_joint_target = ku.ik_for_link(endMatrix, self.manip, "r_link1_L", filter_options=rave.IkFilterOptions.CheckEnvCollisions)

        code.interact(local=locals())

        
        


def testRavenPlanner():
    rospy.init_node('test_raven_planner',anonymous=True)
    rp = RavenPlanner(MyConstants.Arm.Right)

    """
    while not rospy.is_shutdown() and rp.jointStates == None:
        rospy.sleep(.1)

    rospy.loginfo('Getting joint positions')
    rp.getJointPositionsFromPose(tfx.pose([0,0,0],frame=MyConstants.Frames.RightTool))
    """

    rospy.spin()

def testRequest():
    rospy.init_node('test_request',anonymous=True)
    code.interact(local=locals())

if __name__ == '__main__':
    #testRavenPlanner()
    testRequest()

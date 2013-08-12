#!/usr/bin/env python


import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from math import *
import copy
import sys, os
import numpy as np

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *

from RavenDebridement.srv import InvKinSrv


import openravepy as rave
import trajoptpy

import cravenbsppy as ravenbsp

import tf
import tfx

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants
from RavenDebridement.RavenCommand.RavenPlanner import RavenPlanner, Request

import IPython


"""
C  planner->start = start;
C  planner->start_sigma = start_sigma;
C  planner->goal_trans = goal_trans;
C  planner->T = T;


C   planner->controls = initial_controls;
   planner->robot = robot;
   planner->rad = RADFromName(manip_name, robot);
   planner->link = planner->rad->GetRobot()->GetLink(link_name);
   planner->method = BSP::DiscontinuousBeliefSpace;
   planner->initialize();
"""

class InitializationType:
    Zero = 0
    IK = 1

class RavenBSP(RavenPlanner):

    def __init__(self, armName=MyConstants.Arm.Left):
        """
        openrave planner for the raven
        """
        rospy.loginfo('Initialize BSP planner')
        RavenPlanner.__init__(self, armName)

        self.viewer = ravenbsp.GetViewer(self.env)

        self.wrapper = ravenbsp.RavenBSPWrapper(self.env)
        
        self.wrapper.set_manip_name(self.manipName)
        self.wrapper.set_start_sigma(np.eye(len(self.rosJointTypes))*(.0001**2))
        self.wrapper.set_link_name(self.toolFrame)
        
            
        
    #################################
    # Control initializations       #
    #################################
    

        
    def zeroControlInitialization(self, T):
        controlList = []
        for _ in range(T-1):
            controlList.append(np.zeros(len(self.rosJointTypes)))
            
        return controlList
    
    def IKControlInitialization(self, T, startJointPositions, endJointPositions):
        startJointPositions = np.array(startJointPositions)
        endJointPositions = np.array(endJointPositions)
        
        avgDeltaJointPositions = (endJointPositions - startJointPositions) / float(T)
        
        controlList = [np.array(avgDeltaJointPositions) for _ in range(T-1)]
        
        return controlList
                
    
    #################################
    # IK and Trajectory planning    #
    #################################


    def getTrajectoryFromPose(self, endPose, startJoints=None, initType=InitializationType.Zero, n_steps=15):
        """
        Use trajopt to compute trajectory

        Returns a list of joint dictionaries
        """
        
        endPose = tfx.pose(endPose)
        
        endJoints = self.getJointsFromPose(endPose)
        
        if startJoints == None:
            startJoints = self.currentJoints
            if startJoints == None:
                rospy.loginfo('Have not received start joints. Aborting')
                return
        self.updateOpenraveJoints(startJoints)
        
        startJointPositions = []
        endJointPositions = []
        for raveJointType in self.manip.GetArmIndices():
            rosJointType = self.raveJointTypesToRos[raveJointType]
            startJointPositions.append(startJoints[rosJointType])
            endJointPositions.append(endJoints[rosJointType])
            
        goalTrans = self.transformRelativePoseForIk(endPose.matrix, endPose.frame, self.toolFrame)
    
        self.wrapper.set_T(n_steps)
        self.wrapper.set_start(np.array(startJointPositions))
        self.wrapper.set_goal_trans(goalTrans)
        
        if initType == InitializationType.IK:
            initialControls = self.IKControlInitialization(n_steps, startJointPositions, endJointPositions)
        else:
            initialControls = self.zeroControlInitialization(n_steps)
            
        self.wrapper.set_controls(initialControls)
        
        # setup, solve, execute
        self.wrapper.initialize()
        self.wrapper.solve()
        self.wrapper.simulate_execution()
        
        # get the controls
        finalControls = self.wrapper.get_controls()
        
        IPython.embed()
    
    
def testBSP(armName = MyConstants.Arm.Right):
    rospy.init_node('testBSP',anonymous=True)
    rospy.sleep(2)
    bsp = RavenBSP(armName)
    rospy.sleep(1)
    
    desPose = tfx.pose([0,0,0], frame=bsp.toolFrame)
    tf_tool_to_link0 = tfx.lookupTransform(MyConstants.Frames.Link0, desPose.frame, wait=5)
    desPose = tf_tool_to_link0 * desPose
    
    desPose.position.y -= .03
    
    bsp.getTrajectoryFromPose(desPose)
    
    
if __name__ == '__main__':
    testBSP()

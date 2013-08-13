#!/usr/bin/env python

import roslib; roslib.load_manifest('RavenDebridement')
import numpy as np
import os, sys
import random
import math

import trajoptpy
import json

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
        startJoints = {0:0.51091998815536499,
                       1:1.6072717905044558,
                       2:-0.049991328269243247,
                       4:0.14740140736103061,
                       5:0.10896652936935426,
                       8:-0.31163200736045837}
        

        endJoints = {0:0.45221099211619786,
                     1:2.2917657932075581,
                     2:-0.068851854076958902,
                     4:0.44096283117933965,
                     5:0.32085205361054148,
                     8:-0.82079765727524379}
        
        endPose = Util.endPose(startPose, deltaPose)
        #endPose = startPose
        
        #IPython.embed()
        
        jointTraj = self.ravenPlanner.getTrajectoryFromPose(endPose, startJoints=startJoints, n_steps=15)
        """
        n_steps = 15
        
        self.ravenPlanner.updateOpenraveJoints(startJoints)
        
        endJointPositions = endJoints.values()
        request = Request.joints(n_steps, self.ravenPlanner.manip.GetName(), endJointPositions)
        
        # convert dictionary into json-formatted string
        s = json.dumps(request)
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.ravenPlanner.env)
        # do optimization
        result = trajoptpy.OptimizeProblem(prob)

        # check trajectory safety
        from trajoptpy.check_traj import traj_is_safe
        prob.SetRobotActiveDOFs()
        if not traj_is_safe(result.GetTraj(), self.ravenPlanner.robot):
            rospy.loginfo('Trajopt trajectory is not safe. Trajopt failed!')
            rospy.loginfo('Still returning trajectory')
            #return

        IPython.embed()

        return self.ravenPlanner.trajoptTrajToDicts(result.GetTraj())
        """
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
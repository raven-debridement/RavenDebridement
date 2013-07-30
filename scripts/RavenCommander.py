#!/usr/bin/env python

import roslib; roslib.load_manifest('master-control')
import numpy as np
import os
import random

import rospy

import tf
import tf.transformations as tft

from geometry_msgs.msg import *
from std_msgs.msg import Header

import code

import tfx
from raven_2_msgs.msg import *
from raven_2_trajectory.trajectory_player import TrajectoryPlayer, Stage

import thread

# rename so no conflict with raven_2_msgs.msg.Constants
import Constants as MyConstants
import Util
from RavenArm import RavenArm
from ARImageDetection import ARImageDetectionClass


class RavenCommander:
    """
    Class for controlling the end effectors of the Raven
    """

    def __init__ (self, armName):
        self.armName = armName

        self.commandFrame = MyConstants.Frames.Link0
        
        self.ravenArm = RavenArm(self.armName)

        rospy.sleep(1)
 
    #############################
    # start, pause, and stop ####
    #############################
        
    def start(self):
        """
        Starts the raven arm (releases the e-brake)
        (does nothing if raven arm is already running)
        """
        return self.ravenArm.start()

    def pause(self):
        """
        Pauses by clearing the stages
        """
        self.ravenArm.clearStages()

    def isPaused(self):
        """
        True if no stages
        """
        return len(self.player.stages) == 0
        
    def stop(self):
        return self.ravenArm.stop()


    #####################
    # command methods   #
    #####################

    def goToGripperPose(self, endPose, startPose=None, duration=None, speed=None, ignoreOrientation=False):
        """        
        Move to endPose

        If startPose is not specified, default to current pose according to tf
        (w.r.t. Link0)
        """
        if startPose == None:
            startPose = self.getGripperPose()
            if startPose == None:
                return

        startPose = Util.convertToFrame(tfx.pose(startPose), self.commandFrame)
        endPose = Util.convertToFrame(tfx.pose(endPose), self.commandFrame)


        if ignoreOrientation:
            endPose.orientation = startPose.orientation

        self.ravenArm.clearStages()
        self.ravenArm.goToPose(endPose, start=startPose, duration=duration, speed=speed)

    def goToGripperPoseDelta(self, deltaPose, startPose=None, duration=None, speed=None, ignoreOrientation=False):
        """
        Move by deltaPose

        If startPose is not specified, default to current pose according to tf
        (w.r.t. Link0)
        """
        if startPose == None:
            startPose = self.getGripperPose()
            if startPose == None:
                return

        # convert to tfx format
        startPose = Util.convertToFrame(tfx.pose(startPose), self.commandFrame)
        deltaPose = Util.convertToFrame(tfx.pose(deltaPose), self.commandFrame)


        endPosition = startPose.position + deltaPose.position
        endQuatMat = startPose.orientation.matrix * deltaPose.orientation.matrixx

        endPose = tfx.pose(endPosition, endQuatMat)

        self.goToGripperPose(endPose, startPose=startPose, duration=duration, speed=speed, ignoreOrientation=ignoreOrientation)

            
    def closeGripper(self,duration=2.5):
        self.ravenArm.clearStages()
        self.ravenArm.closeGripper(duration=duration)
                
    def openGripper(self,duration=2.5):
        self.ravenArm.clearStages()
        self.ravenArm.openGripper(duration=duration)

    def setGripper(self, value, duration=2.5):
        self.ravenArm.clearStages()
        self.ravenArm.setGripper(value,duration=duration)


    #######################
    # state info methods  #
    #######################

    def getGripperPose(self,frame=MyConstants.Frames.Link0):
        """
        Returns gripper pose w.r.t. frame

        geometry_msgs.msg.Pose
        """
        gripperPose = None

        if self.currentPose != None:
            gripperPose = tfx.pose(self.currentPose)
            tf_pose_to_frame = tfx.lookupTransform(frame, gripperPose.frame, wait=10)
            gripperPose = tf_pose_to_frame * gripperPose
            

        return gripperPose

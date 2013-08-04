#!/usr/bin/env python

import roslib; roslib.load_manifest('RavenDebridement')
import numpy as np
import os, sys
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
from RavenDebridement.Utils import Constants as MyConstants
from RavenDebridement.Utils import Util
from RavenController import RavenController, RavenPlanner
from RavenDebridement.ImageProcessing.ARImageDetection import ARImageDetector


class RavenArm:
    """
    Class for controlling the end effectors of the Raven
    """

    def __init__ (self, armName):
        self.armName = armName

        self.commandFrame = MyConstants.Frames.Link0
        
        self.ravenController = RavenController(self.armName)

 
    #############################
    # start, pause, and stop ####
    #############################
        
    def start(self):
        """
        Starts the raven arm (releases the e-brake)
        (does nothing if raven arm is already running)
        """
        return self.ravenController.start()

    def pause(self):
        """
        Pauses by clearing the stages
        """
        self.ravenController.clearStages()

    def isPaused(self):
        """
        True if no stages
        """
        return len(self.ravenController.stages) == 0
        
    def stop(self):
        return self.ravenController.stop()

    #######################
    # trajectory commands #
    #######################

    def executePoseTrajectory(self, stampedPoses):
        """
        stampedPoses is a list of tfx poses with time stamps
        """

        prevTime = rospy.Time.now()
        prevPose = None

        for stampedPose in stampedPoses:
            duration = stampedPose.stamp - prevTime

            pose = tfx.pose(stampedPose.position, stampedPose.orientation)
            self.goToGripperPose(pose, startPose=prevPose, block=False, duration=duration)

            prevTime = stampedPose.stamp
            prevPose = pose

    def executeDeltaPoseTrajectory(self, stampedDeltaPoses, startPose=None):
        """
        stampedDeltaPoses is a list of tfx poses with time stamps

        each stampedDeltaPose is endPose - startPose, where
        each endPose is different but the startPose is the same

        This function is intended to be used where each
        endPose is from the vision system and the one
        startPose is the gripper position according to vision
        """
        if startPose == None:
            startPose = self.getGripperPose()
            if startPose == None:
                return
        
        prevTime = rospy.Time.now()

        for stampedDeltaPose in stampedDeltaPoses:
            duration = stampedDeltaPose.stamp - prevTime

            deltaPose = tfx.pose(stampedDeltaPose.position, stampedDeltaPose.orientation)
            self.goToGripperPoseDelta(deltaPose, startPose=startPose, block=False, duration=duration)

            prevTime = stampedDeltaPose.stamp

    def executeJointTrajectory(self, stampedJoints):
        """
        stampedJoints is a tuple of ((stamp, joints), ....)

        joints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        """
        
        prevTime = rospy.Time.now()
        prevJoints = None

        for stamp, joints in stampedJoints:
            duration = stamp - prevTime
            joints = dict(joints) # so we don't clobber, just in case
            
            self.goToJoints(joints, startJoints=prevJoints, block=False, duration=duration)

            prevTime = stamp
            prevJoints = joints

    #####################
    # command methods   #
    #####################

    def goToGripperPose(self, endPose, startPose=None, block=True, duration=None, speed=None, ignoreOrientation=False):
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

        self.ravenController.clearStages()
        self.ravenController.goToPose(endPose, start=startPose, duration=duration, speed=speed)

        if block:
            return self.blockUntilPaused()
        return True

    def goToGripperPoseDelta(self, deltaPose, startPose=None, block=True, duration=None, speed=None, ignoreOrientation=False):
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
        endQuatMat = startPose.orientation.matrix * deltaPose.orientation.matrix

        endPose = tfx.pose(endPosition, endQuatMat)

        self.goToGripperPose(endPose, startPose=startPose, block=block, duration=duration, speed=speed, ignoreOrientation=ignoreOrientation)


    def goToJoints(self, desJoints, startJoints=None, block=True, duration=None, speed=None):
        """
        desJoints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        """
        
        self.ravenController.goToJoints(desJoints, duration=duration, speed=speed)

        if block:
            return self.blockUntilPaused()
        return True
            
    def closeGripper(self,duration=2.5, block=True):
        self.ravenController.clearStages()
        self.ravenController.closeGripper(duration=duration)
        
        if block:
            rospy.sleep(duration)

                
    def openGripper(self,duration=2.5, block=True):
        self.ravenController.clearStages()
        self.ravenController.openGripper(duration=duration)
        
        if block:
            rospy.sleep(duration)


    def setGripper(self, value, duration=2.5, block=True):
        self.ravenController.clearStages()
        self.ravenController.setGripper(value,duration=duration)
        
        if block:
            rospy.sleep(duration)


    #######################
    # state info methods  #
    #######################

    def getGripperPose(self,frame=MyConstants.Frames.Link0):
        """
        Returns gripper pose w.r.t. frame

        geometry_msgs.msg.Pose
        """
        gripperPose = self.ravenController.currentPose

        if gripperPose != None:
            gripperPose = Util.convertToFrame(tfx.pose(gripperPose), frame)

        return gripperPose


    #################
    # other methods #
    #################
    
    def blockUntilPaused(self, timeoutTime=999999):
        timeout = Util.Timeout(timeoutTime)
        timeout.start()

        while not self.isPaused():
            if rospy.is_shutdown() or timeout.hasTimedOut():
                return False
            rospy.sleep(.05)

        return True




def testOpenCloseGripper(close=True,arm=MyConstants.Arm.Left):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(1)

    ravenArm.start()
    rospy.loginfo('Setting the gripper')
    if close:
        ravenArm.closeGripper()
    else:
        ravenArm.openGripper()

    rospy.loginfo('Press enter to exit')
    raw_input()

def testMoveToHome(arm=MyConstants.Arm.Left):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    imageDetector = ARImageDetector()
    rospy.sleep(4)

    ravenArm.start()

    rospy.loginfo('Press enter to go to home')
    raw_input()

    desPose = imageDetector.getHomePose()
    ravenArm.goToGripperPose(desPose)
    
    rospy.loginfo('Press enter to exit')
    raw_input()
    
def testGoToJoints(arm=MyConstants.Arm.Right):
    rospy.init_node('rave_arm_node',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)

    endAngles = tfx.tb_angles(0,90,0)
    endPose = tfx.tb_angles([-.136,-.017,-.068], endAngles, frame=MyConstants.Frames.Link0)

    ravenArm.start()

    rospy.loginfo('Press enter to go to endPose using joint commands')
    raw_input()

    desJoints = ravenPlanner.getJointsFromPose(endPose)

    rospy.loginfo('Found joints')
    for jointType, jointPos in desJoints.items():
        print("jointType: {0}, jointPos: {1}".format(jointType,jointPos))

    rospy.loginfo('Press enter to move')
    raw_input()
    
    ravenArm.goToJoints(desJoints)

    rospy.loginfo('Press enter to exit')
    raw_input()
    

if __name__ == '__main__':
    #testOpenCloseGripper(close=True)
    #testMoveToHome()
    testGoToJoints()

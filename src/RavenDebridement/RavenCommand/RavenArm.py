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

import tfx
from raven_2_msgs.msg import *
from raven_2_trajectory.trajectory_player import TrajectoryPlayer, Stage

import openravepy as rave

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
            duration = (stampedPose.stamp - prevTime).to_sec()

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
            duration = (stampedDeltaPose.stamp - prevTime).to_sec()

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
            duration = (stamp - prevTime).to_sec()
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
        
        self.ravenController.goToJoints(desJoints, startJoints=startJoints, duration=duration, speed=speed)

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

    def getCurrentJoints(self):
        """
        Returns is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        """
        return self.ravenController.getCurrentJoints()



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




def testOpenCloseGripper(close=True,arm=MyConstants.Arm.Right):
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

def testMoveToHome(arm=MyConstants.Arm.Right):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    imageDetector = ARImageDetector()
    rospy.sleep(4)

    ravenArm.start()

    rospy.loginfo('Press enter to go to home')
    raw_input()

    #desPose = imageDetector.getReceptaclePose()
    desPose = imageDetector.getHomePose()
    ravenArm.goToGripperPose(desPose)
    
    rospy.loginfo('Press enter to exit')
    raw_input()
    
def testGoToJoints(arm=MyConstants.Arm.Right):
    rospy.init_node('rave_arm_node',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)

    angle = tfx.tb_angles(90,90,0)
    endPose = tfx.pose([-.134, -.013, -.068], angle,frame=MyConstants.Frames.Link0)
    # z -.068

    ravenArm.start()

    rospy.loginfo('Press enter to go to endPose using joint commands')
    raw_input()

    desJoints = ravenPlanner.getJointsFromPose(endPose)
    currJoints = ravenArm.ravenController.currentJoints

    rospy.loginfo('Found joints')
    for jointType, jointPos in desJoints.items():
        print("desired: jointType = {0}, jointPos = {1}".format(jointType,jointPos))
        print("current: jointType = {0}, jointPos = {1}".format(jointType,currJoints[jointType]))

    #code.interact(local=locals())

    rospy.loginfo('Press enter to move')
    raw_input()
    
    ravenArm.goToJoints(desJoints)

    #code.interact(local=locals())

    rospy.loginfo('Press enter to exit')
    raw_input()

def testGoToPose(arm=MyConstants.Arm.Right):
    rospy.init_node('raven_arm_node',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(2)

    endAngles = tfx.tb_angles(0,90,0)
    endPose = tfx.pose([-.136,-.017,-.075], endAngles, frame=MyConstants.Frames.Link0)

    ravenArm.start()

    rospy.loginfo('Press enter to go to endPose')
    raw_input()

    ravenArm.goToGripperPose(endPose)

    rospy.loginfo('Press enter to exit')
    raw_input()
    

def testTrajopt(arm=MyConstants.Arm.Right):
    rospy.init_node('test_trajopt',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)

    angle = tfx.tb_angles(90,90,0)
    endPose = tfx.pose([-.133, -.015, -.1], angle,frame=MyConstants.Frames.Link0)
    # z -.072

    startJoints = ravenArm.getCurrentJoints()

    endJoints = ravenPlanner.getJointsFromPose(endPose)

    rospy.loginfo('desired and start joint positions')
    for jointType, jointPos in endJoints.items():
        print("desired: jointType = {0}, jointPos = {1}".format(jointType,(180.0/math.pi)*jointPos))
        print("current: jointType = {0}, jointPos = {1}".format(jointType,(180.0/math.pi)*startJoints[jointType]))

    rospy.loginfo('Press enter to call trajopt')
    raw_input()

    jointTraj = ravenPlanner.getTrajectoryFromPose(startJoints, endPose)

    for jointDict in jointTraj:
        print(jointDict)

    """
    for trajIndex in range(len(jointTraj)):
        jointWaypoint = jointTraj[trajIndex]
        print(list((180.0/math.pi)*jointWaypoint))

    endJointPositions = jointTraj[-1]
    endTrajJoints = dict(zip(startJoints.keys()[:-1], list(endJointPositions)))     
    """

    ravenPlanner.env.SetViewer('qtcoin')
    #ravenPlanner.updateOpenraveJoints(jointTraj[-1])

    rospy.loginfo('Successful use of trajopt')
    code.interact(local=locals())

def testExecuteJointTrajectory(arm=MyConstants.Arm.Right):
    rospy.init_node('test_trajopt',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)


    if arm == MyConstants.Arm.Right:
        toolframe = MyConstants.Frames.RightTool
    else:
        toolframe = MyConstants.Frames.LeftTool

    currPose = tfx.pose([0,0,0], frame=toolframe)
    tf_tool_to_link0 = tfx.lookupTransform(MyConstants.Frames.Link0, currPose.frame, wait=5)
    currPose = tf_tool_to_link0 * currPose

    angle = tfx.tb_angles(0,90,0)
    endPosition = currPose.position
    endPosition.x -= .03
    endPose = tfx.pose(endPosition, angle,frame=MyConstants.Frames.Link0)


    startJoints = ravenArm.getCurrentJoints()

    ####### TEMP
    # so adding box will work, don't normally need to do though
    ravenPlanner.updateOpenraveJoints(startJoints)
    box = rave.RaveCreateKinBody(ravenPlanner.env,'')
    box.SetName('testbox')
    box.InitFromBoxes(np.array([[-.02,0,0,0.005,0.01,0.015]]),True)
    #code.interact(local=locals())
    ee = ravenPlanner.manip.GetEndEffectorTransform()
    ee[:3,:3] = np.identity(3)
    box.SetTransform(ee)
    ravenPlanner.env.Add(box,True)
    #ravenPlanner.env.SetViewer('qtcoin')
    rospy.loginfo('Box created, press enter')
    raw_input()
    #return
    ###########

    #ravenPlanner.env.SetViewer('qtcoin')

    rospy.loginfo('Press enter to call trajopt')
    raw_input()
    rospy.sleep(4)

    jointTraj = ravenPlanner.getTrajectoryFromPose(startJoints, endPose)
    
    if jointTraj == None:
        return

    duration = 40.0
    waypointDuration = duration / float(len(jointTraj))
    
    now = rospy.Time.now()
    stamps = [now + rospy.Duration(i*waypointDuration) for i in range(len(jointTraj))]

    stampedJoints = zip(stamps, jointTraj)
    
    rospy.loginfo('Have stamped joints, ready to execute')

    #ravenArm.start()
    
    #ravenArm.executeJointTrajectory(stampedJoints)
    
    
    #ravenPlanner.updateOpenraveJoints(jointTraj[0])
    code.interact(local=locals())

    rospy.loginfo('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    #testOpenCloseGripper(close=True)
    #testMoveToHome()
    #testGoToJoints()
    #testGoToPose()
    #testTrajopt()
    testExecuteJointTrajectory()

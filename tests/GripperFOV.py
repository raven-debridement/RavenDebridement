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
from RavenDebridement.ImageProcessing.ImageDetection import ImageDetector
from RavenDebridement.RavenCommand.RavenArm import RavenArm

import IPython

class GripperFOVClass():
    
    def __init__(self, armName):
        print('Initializing GripperFOVClass')
        self.armName = armName
        self.ravenArm = RavenArm(self.armName)
        self.imageDetector = ImageDetector()
        self.camFrame = '/left_BC'
        
        self.desiredOrientation = tfx.tb_angles(90,0,0) # in self.camFrame
        
        self.posFrame = MyConstants.Frames.Link0
        self.rotFrame = MyConstants.Frames.RightTool if self.armName == MyConstants.Arm.Right else MyConstants.Frames.LeftTool
        
        self.pub = rospy.Publisher('orig_gripper_pose',PoseStamped)
        
        self.file = open('/tmp/GripperFOVTest.txt','w')
        
        print('Initialized GripperFOVClass')
        
    def currentPose(self):
        currTfPose = Util.convertToFrame(self.ravenArm.getGripperPose(), self.camFrame)
        currDetectedPose = Util.convertToFrame(self.imageDetector.getGripperPose(self.armName), self.camFrame)
        
        currPoseInCamera = tfx.pose(currTfPose.position, currDetectedPose.orientation)
        
        return currPoseInCamera
    
    def homePose(self):
        homePose = self.currentPose()
        homePose.orientation = self.desiredOrientation
        return homePose

    def findNewGripper(self, timeout=1):
        timeout = Util.Timeout(timeout)
        self.imageDetector.ignoreOldGripper(self.armName)
        timeout.start()
        while not timeout.hasTimedOut():
            if self.imageDetector.hasFoundNewGripper(self.armName):
                return tfx.pose(self.imageDetector.getGripperPose(self.armName))
            
        return None

    def rotateUntilUnseen(self, axis, rotateBy):
        firstPose = gripperPose = Util.convertToFrame(self.findNewGripper(), MyConstants.Frames.Link0)
        firstTfPose = Util.convertToFrame(self.ravenArm.getGripperPose(), MyConstants.Frames.Link0)
        self.pub.publish(firstPose.msg.PoseStamped())
        
        if axis == 'pitch':
            tb_rotate = tfx.tb_angles(0,rotateBy,0)
            origAngle = tfx.tb_angles(firstPose.orientation).pitch_deg
        elif axis == 'roll':
            tb_rotate = tfx.tb_angles(0,0,rotateBy)
            origAngle = tfx.tb_angles(firstPose.orientation).roll_deg
        else:
            return 0.0
        
        totalRotation = 0.0
        rotatePose = tfx.pose([0,0,0], tb_rotate)
        
        
        lastOrientation = tfx.pose(self.ravenArm.getGripperPose()).orientation
        lastPose = None
        while gripperPose is not None:
            #gripperPose = Util.convertToFrame(gripperPose, self.rotFrame)
            #tfPose = tfx.pose([0,0,0],frame=self.rotFrame)
            #tapeRotation = gripperPose.orientation
            #tfRotation = tfPose.orientation
            #IPython.embed()
            #return
        
            #tfGripperPose = tfx.pose(self.ravenArm.getGripperPose())
            #gripperPose = Util.convertToFrame(gripperPose, tfGripperPose.frame)
            #endPose = tfx.pose(tfGripperPose.position, rotatePose.orientation.matrix * gripperPose.orientation.matrix)
            #self.ravenArm.goToGripperPose(endPose, duration=1)
            
            #endPose = Util.endPose(gripperPose,tfx.pose([0,0,0],rotatePose.orientation, frame=self.camFrame),frame=MyConstants.Frames.Link0)
            #deltaPose = tfx.pose([0,0,0], endPose.orientation)
            #self.ravenArm.goToGripperPoseDelta(deltaPose, duration=1)
            
            tfGripperPose = tfx.pose(self.ravenArm.getGripperPose())
            tfGripperPose = Util.convertToFrame(tfGripperPose, self.camFrame)
            endPose = Util.endPose(tfGripperPose, tfx.pose([0,0,0],tb_rotate,frame=self.camFrame))
            endPose = Util.convertToFrame(endPose, MyConstants.Frames.Link0)
            endPose.position = firstTfPose.position
            
            self.ravenArm.goToGripperPose(endPose, duration=1)
            print('Rotating...')
            rospy.sleep(.25)
            
            #self.ravenArm.goToGripperPoseDelta(rotatePose, duration=1)
            
            totalRotation += rotateBy
            #print('Total rotation = {0}'.format(totalRotation))
            
            
            currOrientation = tfx.pose(self.ravenArm.getGripperPose()).orientation
            if lastOrientation is not None:
                angleBetween = Util.angleBetweenQuaternions(lastOrientation.msg.Quaternion(), currOrientation.msg.Quaternion())
                #print('angleBetween = {0}'.format(angleBetween))
                if angleBetween < rotateBy*.5:
                    self.file.write('Hit physical limit\n')
                    print('Hit physical limit')
                    break
            lastOrientation = currOrientation
            
            rospy.sleep(.5)
            lastPose = Util.convertToFrame(gripperPose, self.camFrame)
            gripperPose = self.findNewGripper()
            
        if lastPose is not None:
            firstPose = Util.convertToFrame(firstPose, self.camFrame)
            if axis == 'pitch':
                firstPitch = tfx.tb_angles(firstPose.orientation).pitch_deg
                lastPitch = tfx.tb_angles(lastPose.orientation).pitch_deg
                return copysign(fabs(lastPitch-firstPitch),rotateBy)
            elif axis == 'roll':
                firstRoll = tfx.tb_angles(firstPose.orientation).roll_deg
                lastRoll = tfx.tb_angles(lastPose.orientation).roll_deg
                return copysign(fabs(lastRoll-firstRoll),rotateBy)
                   
        return totalRotation

    def runTest(self):
        self.ravenArm.start()
        rospy.sleep(2)
        
        currPoseInCamera = self.currentPose()
        homePose = self.homePose()
        
        deltaPose = Util.deltaPose(currPoseInCamera, homePose, self.posFrame, self.rotFrame)
        
        
        self.ravenArm.goToGripperPoseDelta(deltaPose, duration=2)
        
        # self.ravenArm.goToGripperPoseDelta(tfx.pose([0,0,0],tfx.tb_angles(0,30,0)),duration=2)
        #rotatePose = tfx.pose([0,0,0],tfx.tb_angles(0,45,0))
        
        #IPython.embed()
        #return
        
        
        # z must always stay the same
        z = currPoseInCamera.position.z
        
        xMove = .03
        yMove = .01
        gridMoves = [[-xMove, 0, 0],
                     [0, yMove, 0],
                     [xMove, 0, 0],
                     [0, -yMove, 0]]
        rotationAxes = ['pitch', 'roll']
        
        for gridMove in gridMoves:
            print('Go to next place on grid, press enter')
            raw_input()
            
            deltaPose = Util.deltaPose(self.currentPose(), self.homePose()+gridMove, self.posFrame, self.rotFrame)
            homePose = Util.endPose(self.ravenArm.getGripperPose(), deltaPose)
            self.ravenArm.goToGripperPose(homePose)
            
            self.file.write('Home pose {0}\n'.format(homePose))
            print('Home pose {0}'.format(homePose))
            for axis in rotationAxes:
                if rospy.is_shutdown():
                    return
                
                rotateBy = 5.0
                self.file.write('Rotating in positive direction on {0} axis\n'.format(axis))
                print('Rotate in positive direction on {0} axis'.format(axis))
                #raw_input()
                maxRotation = self.rotateUntilUnseen(axis, rotateBy)
                self.ravenArm.goToGripperPose(homePose, duration=1)
                rospy.sleep(1)
                self.file.write('Rotating in negative direction on {0} axis\n'.format(axis))
                print('Rotate in negative direction on {0} axis'.format(axis))
                #raw_input()
                minRotation = self.rotateUntilUnseen(axis, -rotateBy)
                self.ravenArm.goToGripperPose(homePose, duration=1)
                rospy.sleep(1)
                
                self.file.write('{0} range of motion is ({1},{2})\n'.format(axis,minRotation,maxRotation))
                print('{0} range of motion is ({1},{2})'.format(axis,minRotation,maxRotation))
            
        
        self.file.close()
        
        self.ravenArm.stop()
        
        
if __name__ == '__main__':
    rospy.init_node('GripperFOVTest',anonymous=True)
    print('InitNode')
    gripperFOV = GripperFOVClass(MyConstants.Arm.Right)
    rospy.sleep(3)
    gripperFOV.runTest()
        
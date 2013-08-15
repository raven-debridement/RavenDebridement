#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math

from geometry_msgs.msg import *
from raven_2_msgs.msg import *

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.RavenCommand.RavenPlanner import RavenPlanner, Request

import tfx

import IPython

class Test():
    def __init__(self, arm=Constants.Arm.Right):
        rospy.Subscriber(Constants.GripperTape.Topic, PoseStamped, self.tapeCallback)

        self.record = False

        self.tapePoses = []
        self.tfPoses = []
        self.deltaPoses = []

        if arm == Constants.Arm.Left:
            self.toolFrame = Constants.Frames.LeftTool
        else:
            self.toolFrame = Constants.Frames.RightTool

        self.recordCount = 0

    def startRecording(self):
        self.record = True
        self.recordCount = 0

    def stopRecording(self):
        self.record = False

    def printPoses(self):
        for tapePose, tfPose, deltaPose in zip(self.tapePoses, self.tfPoses, self.deltaPoses):
            print('\n\n\nTime stamp: {0}'.format(tapePose.stamp))
        
            print('\ntapePose')
            print(tapePose.position)
            print(tapePose.tb_angles)

            print('\ntfPose')
            print(tfPose.position)
            print(tfPose.tb_angles)

            print('\ndeltaPose')
            print(deltaPose.position)
            print(deltaPose.tb_angles)

    def tapeCallback(self, msg):
        if not self.record:
            return

        self.recordCount += 1
        rospy.loginfo('Received tape pose number {0}'.format(self.recordCount))
        
        tapePose = tfx.pose(msg)
        stamp = tapePose.stamp

        tf_tape_to_link0 = tfx.lookupTransform(Constants.Frames.Link0, tapePose.frame, wait=4)
        tapePose = tf_tape_to_link0 * tapePose

        tfPose = tfx.pose([0,0,0], frame=self.toolFrame, stamp=stamp)
        tf_tfpose_to_link0 = tfx.lookupTransform(Constants.Frames.Link0, tfPose.frame, wait=4)
        tfPose = tf_tfpose_to_link0 * tfPose
        
        deltaPose = tfx.pose(Util.deltaPose(tfPose, tapePose))
        
        self.tapePoses.append(tapePose)
        self.tfPoses.append(tfPose)
        self.deltaPoses.append(deltaPose)
        
        
def testPoseDiff(arm=Constants.Arm.Left):
    rospy.init_node('testPoseDiff',anonymous=True)
    t = Test(arm)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    
    rospy.loginfo('Press enter to start recording')
    raw_input()
    
    t.startRecording()
    
    rospy.loginfo('Press enter to stop recording')
    raw_input()
    
    t.stopRecording()
    
    rospy.loginfo('Press enter to print poses')
    raw_input()
    
    t.printPoses()
    
    rospy.loginfo('Press enter to compare joints')
    raw_input()
    
    tapePose = t.tapePoses[-1]
    
    ravenJoints = ravenArm.getCurrentJoints()
    ikJoints = ravenPlanner.getJointsFromPose(tapePose)
    
    for jointType in ravenPlanner.rosJointTypes:
        ravenJoint = (180.0/math.pi)*ravenJoints[jointType]
        ikJoint = (180.0/math.pi)*ikJoints[jointType]
        print('JointType = {0}'.format(jointType))
        print('ravenJoint = {0}'.format(ravenJoint))
        print('ikJoint = {0}'.format(ikJoint))
        print('Joint difference = {0}\n'.format(abs(ravenJoint-ikJoint)))
        
    
    rospy.loginfo('Press enter to exit')
    raw_input()
    
def execAndRecordTraj(arm=Constants.Arm.Left):
    rospy.init_node('execAndRecordTraj',anonymous=True)
    rospy.sleep(1)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    record = Test(arm)
    rospy.sleep(2)
    
    ravenArm.start()
    
    rospy.loginfo('Classes are initialized, press enter')
    raw_input()
    

    startPose = tfx.pose(ravenArm.getGripperPose())    
    
    x = .03
    y = .06
    z = .01
    deltaPoses = []
    #deltaPoses.append(tfx.pose([x,0, -z],frame=Constants.Frames.Link0))
    deltaPoses.append(tfx.pose([0,-y, 0],frame=Constants.Frames.Link0))
    #deltaPoses.append(tfx.pose([-x,0,z],frame=Constants.Frames.Link0))
    deltaPoses.append(tfx.pose([0,y, 0],frame=Constants.Frames.Link0))

    #record.startRecording()
    while not rospy.is_shutdown():
        for deltaPose in deltaPoses:
            startPose = tfx.pose(ravenArm.getGripperPose())
            endPose = tfx.pose(Util.endPose(startPose, deltaPose))
            #ravenArm.goToGripperPose(endPose,speed=.04)
            poseTraj = ravenPlanner.getTrajectoryFromPose(endPose,n_steps=10)
            rospy.loginfo('startPose')
            rospy.loginfo(startPose)
            rospy.loginfo('endPose')
            rospy.loginfo(endPose)
            rospy.loginfo('Used trajopt, now move')
            raw_input()
            ravenArm.executePoseTrajectory(poseTraj,speed=.04)
            #ravenArm.executeJointTrajectory(jointTraj, speed=.5)
            #endJoints = ravenPlanner.getJointsFromPose(endPose)
            #ravenArm.goToJoints(endJoints,speed=1.5)
    record.stopRecording()
            
    rospy.loginfo('Press enter to print and exit')
    raw_input()
    
    ravenArm.stop()
    
    record.printPoses()
    
    IPython.embed()
    
if __name__ == '__main__':
    #testPoseDiff()
    execAndRecordTraj()
    
    
    

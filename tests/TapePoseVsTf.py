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
from RavenDebridement.RavenCommand.RavenPlanner import RavenPlanner

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
        
        
def testPoseDiff(arm=Constants.Arm.Right):
    rospy.init_node('testPoseDiff',anonymous=True)
    t = Test(arm)
    
    rospy.loginfo('Press enter to start recording')
    raw_input()
    
    t.startRecording()
    
    rospy.loginfo('Press enter to stop recording')
    raw_input()
    
    t.stopRecording()
    
    rospy.loginfo('Press enter to print poses')
    raw_input()
    
    t.printPoses()
    
    rospy.loginfo('Press enter to exit')
    raw_input()
    
def execAndRecordTraj(arm=Constants.Arm.Right):
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
    y = .03
    z = .03
    deltaPoses = []
    deltaPoses.append(tfx.pose([x,0, -z],frame=Constants.Frames.Link0))
    deltaPoses.append(tfx.pose([0,-y, 0],frame=Constants.Frames.Link0))
    deltaPoses.append(tfx.pose([-x,0,z],frame=Constants.Frames.Link0))
    deltaPoses.append(tfx.pose([0,y, 0],frame=Constants.Frames.Link0))

    record.startRecording()
    while not rospy.is_shutdown() and record.recordCount < 100:
        for deltaPose in deltaPoses:
            startPose = tfx.pose(ravenArm.getGripperPose())
            endPose = Util.endPose(startPose, deltaPose)
            jointTraj = ravenPlanner.getTrajectoryFromPose(endPose)
            #rospy.loginfo('Press enter to execute')
            #raw_input()
            ravenArm.executeJointTrajectory(jointTraj, speed=.5)
    record.stopRecording()
            
    rospy.loginfo('Press enter to print and exit')
    raw_input()
    
    ravenArm.stop()
    
    record.printPoses()
    
    IPython.embed()
    
if __name__ == '__main__':
    #testPoseDiff()
    execAndRecordTraj()
    
    
    
"""


"""

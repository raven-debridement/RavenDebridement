#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math

from geometry_msgs.msg import *
from raven_2_msgs.msg import *

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants

import tfx

import code

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

    def startRecording(self):
        self.record = True

    def stopRecording(self):
        self.record = False

    def print(self):
        for tapePose, tfPose, deltaPose in zip(self.tapePoses, self.tfPoses, self.deltaPoses):
            print('Time stamp: {0}'.format())
        
            print('tapePose')
            print(tapePose.position)
            print(tapePose.tb_angles)

            print('tfPose')
            print(tfPose.position)
            print(tfPose.tb_angles)

            print('deltaPose')
            print(deltaPose.position)
            print(deltaPose.tb_angles)

    def tapeCallback(self, msg):
        if not self.record:
            return

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

        print('Time stamp: {0}'.format(stamp))
        
        print('tapePose')
        print(tapePose.position)
        print(tapePose.tb_angles)

        print('tfPose')
        print(tfPose.position)
        print(tfPose.tb_angles)

        print('deltaPose')
        print(deltaPose.position)
        print(deltaPose.tb_angles)

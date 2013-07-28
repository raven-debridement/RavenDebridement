#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy

import tf
import tf.transformations as tft
import tfx

from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion
from raven_pose_estimator.srv import ThreshRed

from scripts import Util
from scripts import Constants
from scripts.GripperControl import GripperControlClass
from scripts.ImageDetection import ImageDetectionClass
from scripts.ARImageDetection import ARImageDetectionClass

import code

class PickupTriangleTest():
    def __init__(self, arm=Constants.Arm.Left):
        self.arm = arm

        if self.arm == Constants.Arm.Left:
            self.toolframe = Constants.Frames.LeftTool
        else:
            self.toolframe = Constants.Frames.RightTool

        self.transFrame = Constants.Frames.Link0
        self.rotFrame = self.toolframe
            
        self.gripperControl = GripperControlClass(arm, tf.TransformListener())
        self.imageDetector = ARImageDetectionClass()
                
        self.homePose = self.imageDetector.getHomePose()

        # ar_frame is the target
        self.ar_frame = '/stereo_33'
        self.objectPose = tfx.pose([0,0,0], frame=self.ar_frame)
        tf_ar_to_link0 = tfx.lookupTransform(Constants.Frames.Link0, self.ar_frame, wait=5)
        self.objectPose = tf_ar_to_link0 * self.objectPose
        self.objectPose = tfx.pose(self.objectPose.position, tfx.tb_angles(-90,90,0))
        self.objectPose.position.y -= .002

        rospy.sleep(3)


    def moveNearTriangle(self):
        rospy.loginfo('Moving near the triangle')

        deltaPose = Util.deltaPose(self.gripperpose, self.objectPose, self.transFrame, self.rotFrame)
        deltaPose.position.z += .05

        
        self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose)

        while not self.gripperControl.isPaused() and not rospy.is_shutdown():
            rospy.sleep(.1)

    def servoNearTriangle(self):
        rospy.loginfo('Servoing near the triangle')

        while not self.imageDetector.hasFoundGripper(self.arm) and not rospy.is_shutdown():
            rospy.loginfo('Searching for gripper')
            rospy.sleep(.1)

        rospy.loginfo('Found gripper')
        gripperPose = self.imageDetector.getGripperPose(self.arm)

        while not self.imageDetector.hasFoundObject() and not rospy.is_shutdown():
            rospy.loginfo('Searching for object')
            rospy.sleep(.1)

        rospy.loginfo('Found object')
        self.objectPose = self.imageDetector.getObjectPose() # in link 0
        self.objectPose.pose.position.z += 0.02

        transBound = .008
        rotBound = float("inf")

        maxMovement = .015

        while not Util.withinBounds(gripperPose, self.objectPose, transBound, rotBound, self.transFrame, self.rotFrame) and not rospy.is_shutdown():
            if self.gripperControl.isPaused():
                rospy.sleep(1)
                self.publishObjPose(self.objectPose)
                if self.imageDetector.hasFoundNewGripper(self.arm):
                    rospy.loginfo('paused and found new gripper')
                    gripperPose = self.imageDetector.getGripperPose(self.arm)
                    deltaPose = tfx.pose(Util.deltaPose(gripperPose, self.objectPose, Constants.Frames.Link0, self.toolframe))
                    deltaPose.position = deltaPose.position.capped(maxMovement)
                    deltaPose0Link = tfx.pose(Util.deltaPose(gripperPose, self.objectPose, Constants.Frames.Link0, Constants.Frames.Link0))
                    deltaPose0Link.position = deltaPose.position.capped(maxMovement)
                    self.publishDeltaPose(deltaPose0Link, tfx.pose(gripperPose))
                    rospy.loginfo('pres enter to move')
                    raw_input()
                    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, ignoreOrientation=True, duration=4)

            rospy.sleep(.1)

def open_test():
    rospy.init_node('open_test',anonymous=True)
    test = ServoVsOpenTest()
    test.openTest()

def servo_test():
    rospy.init_node('servo_test',anonymous=True)
    test = ServoVsOpenTest()
    test.servoTest()


if __name__ == '__main__':
    #open_test()
    servo_test()
    #code.interact(local=locals())

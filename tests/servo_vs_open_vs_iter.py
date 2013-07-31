#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('RavenDebridement')
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

class ServoVsOpenVsIterTest():
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
        
        self.gripperOpenCloseDuration = 2.5

        rospy.sleep(3)


    def goHome(self):
        self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), self.homePose.pose, ignoreOrientation=True)


        while not self.gripperControl.isPaused() and not rospy.is_shutdown():
            rospy.sleep(.1)

        rospy.loginfo('Opening the gripper')
        self.gripperControl.openGripper(duration=self.gripperOpenCloseDuration)
        rospy.sleep(self.gripperOpenCloseDuration)


    def openTest(self):
        self.gripperControl.start()

        self.goHome()

        while not rospy.is_shutdown():
            if self.imageDetector.hasFoundObject:

                objectPose = self.imageDetector.getObjectPose()

                ravenGripperPose = self.gripperControl.getGripperPose(Constants.Frames.Link0)

                rospy.loginfo('Press enter')
                raw_input()

                self.gripperControl.goToGripperPose(ravenGripperPose, objectPose)
                break
            
            rospy.sleep(.1)

        rospy.spin()


    def oneServoTest(self):
        self.gripperControl.start()

        self.goHome()

        while not rospy.is_shutdown():
            if self.imageDetector.hasFoundGripper(self.arm) and self.imageDetector.hasFoundObject():

                gripperPose = self.imageDetector.getGripperPose(self.arm)
                objectPose = self.imageDetector.getObjectPose()

                deltaPose = Util.deltaPose(gripperPose, objectPose, self.transFrame, self.rotFrame)

                ravenGripperPose = self.gripperControl.getGripperPose(Constants.Frames.Link0)

                rospy.loginfo('Press enter')
                raw_input()

                self.gripperControl.goToGripperPoseDelta(ravenGripperPose, deltaPose)
                break
            
            rospy.sleep(.1)

        rospy.spin()

    def multipleServoTest(self):

        self.gripperControl.start()

        self.goHome()

        while not self.imageDetector.hasFoundGripper(self.arm) and not rospy.is_shutdown():
            rospy.loginfo('Searching for gripper')
            rospy.sleep(.1)

        rospy.loginfo('Found gripper')
        gripperPose = self.imageDetector.getGripperPose(self.arm)

        while not self.imageDetector.hasFoundObject() and not rospy.is_shutdown():
            rospy.loginfo('Searching for object')
            rospy.sleep(.1)

        rospy.loginfo('Found object')
        self.objectPose = self.imageDetector.getObjectPose()

        transBound = .008
        rotBound = float("inf")

        maxMovement = .015

        while not Util.withinBounds(gripperPose, self.objectPose, transBound, rotBound, self.transFrame, self.rotFrame) and not rospy.is_shutdown():
            if self.gripperControl.isPaused():
                rospy.sleep(.5)
                if self.imageDetector.hasFoundNewGripper(self.arm):
                    rospy.loginfo('paused and found new gripper')
                    gripperPose = self.imageDetector.getGripperPose(self.arm)
                    deltaPose = tfx.pose(Util.deltaPose(gripperPose, self.objectPose, Constants.Frames.Link0, self.toolframe))
                    deltaPose.position = deltaPose.position.capped(maxMovement)
                    
                    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose)

            rospy.sleep(.1)

def open_test():
    rospy.init_node('open_test',anonymous=True)
    test = ServoVsOpenVsIterTest()
    test.openTest()

def one_servo_test():
    rospy.init_node('servo_test',anonymous=True)
    test = ServoVsOpenVsIterTest()
    test.oneServoTest()

def multiple_servo_test():
    rospy.init_node('multiple_servo_test',anonymous=True)
    test = ServoVsOpenVsIterTest()
    test.multipleServoTest()



if __name__ == '__main__':
    #open_test()
    #one_servo_test()
    multiple_servo_test()

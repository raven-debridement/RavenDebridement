#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion, TransformStamped, Point
from sensor_msgs.msg import Image
from ar_pose.msg import *
import tf
import tf.transformations as tft
import math

import Util
import Constants
from ImageDetection import ImageDetectionClass

from threading import Lock

ids_to_joints = {0: Constants.AR.Frames.Grasper1,
                 1: Constants.AR.Frames.Grasper2,
                 4: Constants.AR.Frames.Cube1,
                 5: Constants.AR.Frames.Cube2,
                 6: Constants.AR.Frames.Cube3,
                 7: Constants.AR.Frames.Cube4}

class ARImageDetectionClass(ImageDetectionClass):
    
      class State():
            CalibrateLeft = 0
            CalibrateRight = 1
            Calibrated = 2
            Calibrating = 3 # waiting for signal to calibrate left or right

      """
      Used to detect object, grippers, and receptacle
      """
      def __init__(self, normal=None):
            self.objectPoint = None
            
            #gripper pose. Must both have frame_id of respective tool frame
            self.leftGripperPose = None
            self.rightGripperPose = None
            #receptacle point. Must have frame_id of global (or main camera) frame
            #is the exact place to drop off (i.e. don't need to do extra calcs to move away)
            self.receptaclePoint = None
            #table normal. Must be according to global (or main camera) frame
            if normal != None:
                  self.normal = normal
            else:
                  # default to straight up
                  self.normal = Util.makeQuaternion(.5**.5, 0, -.5**.5, 0)

            self.state = None
            self.transforms = {}

            # image processing to find object
            self.listener = tf.TransformListener()
            #self.objectProcessing = ImageProcessingClass()

            self.locks = dict()
            self.locks['ar_pose'] = Lock()

            # Temporary. For finding the receptacle
            rospy.Subscriber(Constants.StereoClick.StereoName, PointStamped, self.stereoCallback)
            # Get grippers using AR
            self.debugArCount = 0
            rospy.Subscriber(Constants.AR.Stereo, ARMarkers, self.arCallback)

      def setState(self, state):
            self.state = state

      def arCallback(self, msg):
            self.locks['ar_pose'].acquire()
            markers = msg.markers
            #rospy.loginfo(len(markers))
            for marker in markers:
                #arframe = Constants.StereoAR + "_" + str(marker.id)
                frame = ids_to_joints[marker.id]
                if frame == Constants.AR.Frames.Grasper1 or frame == Constants.AR.Frames.Grasper2:
                    #self.arHandler(marker, "left")
                    self.arHandlerWithOrientation(marker, "left")
                elif ids_to_joints[marker.id] == Constants.Arm.Right:
                    self.arHandlerWithOrientation(marker, "right")
            self.locks['ar_pose'].release() 

      def debugAr(self, gp):
        self.debugArCount += 1
        if self.debugArCount % 10 == 0:
            print self.listener.transformPose(Constants.Frames.RightTool, gp)

      def arHandlerWithOrientation(self, marker, armname):
        pose = PoseStamped()
        pose.header.stamp = marker.header.stamp
        pose.header.frame_id = marker.header.frame_id
        pose.pose = marker.pose.pose
        if armname == "left":
            self.leftGripperPose = pose
        else:
            self.rightGripperPose = pose

      def isCalibrated(self):
            return self.state == ARImageDetectionClass.State.Calibrated

def test2():
      """
      Prints when an objectPoint has been detected
      Mostly a test of the ImageProcessing class
      """
      rospy.init_node('image_detection_node')
      imageDetector = ARImageDetectionClass()
      while not rospy.is_shutdown():
            objectPoint = imageDetector.getObjectPoint()
            if objectPoint != None:      
                  print(objectPoint)
            else:
                  print('Not Found')
            rospy.sleep(.5)

def testCalibration():
    rospy.init_node('image_detection_node')
    imageDetector = ARImageDetectionClass()
    while not rospy.is_shutdown():
        raw_input("press any key to calibrate")
        if not imageDetector.isCalibrated():
            imageDetector.setState(ARImageDetectionClass.State.CalibrateLeft)
        rospy.sleep(.5)

def testFound():
    rospy.init_node('ar_image_detection')
    imageDetector = ARImageDetectionClass()
      
    while not rospy.is_shutdown():
        if imageDetector.hasFoundGripper(Constants.Arm.Left):
            rospy.loginfo('Found left arm')
        if imageDetector.hasFoundGripper(Constants.Arm.Right):
            rospy.loginfo('Found right arm')
            
        if imageDetector.hasFoundObject():
            rospy.loginfo('Found object')
            
        rospy.loginfo('Spinning')
        rospy.sleep(.5)
      

if __name__ == '__main__':
    #testCalibration()
    testFound()

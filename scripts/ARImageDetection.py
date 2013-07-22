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
import tfx
import tf.transformations as tft
import math
import numpy as np

import Util
import Constants
from ImageDetection import ImageDetectionClass

from threading import Lock

import code

APPROX_TIME = 0.1 # in sec
id_map = {32: Constants.AR.Frames.Object}

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
            self.registerObjectPublisher()

            #gripper pose. Must both have frame_id of respective tool frame
            self.leftGripperPose = None
            self.rightGripperPose = None

            self.newLeftGripperPose = False
            self.newRightGripperPose = False


            #receptacle point. Must have frame_id of link_0
            #is the exact place to drop off (i.e. don't need to do extra calcs to move away
            self.receptaclePoint = tfx.point([-.017,-.019,-.097],frame=Constants.Frames.Link0).msg.PointStamped()
            

            #table normal. Must be according to global (or main camera) frame
            if normal != None:
                  self.normal = normal
            else:
                  # default to straight up
                  # self.normal = Util.makeQuaternion(.5**.5, 0, -.5**.5, 0)
                  #self.normal = tfx.tb_angles(0,0,-90).msg
                  self.normal = tfx.tb_angles(-90,90,0).msg
                  #self.normal = tfx.tb_angles(0,0,90).msg


            self.state = None
            self.transforms = {}
            self.tapeMsg = None
            self.gripperPoseIsEstimate = False
            self.estimatedPosePub = rospy.Publisher('estimated_pose', PoseStamped)

            self.listener = tf.TransformListener()

            self.locks = dict()
            self.locks['ar_pose'] = Lock()

            # Make gripper location more robust using AR
            self.debugArCount = 0
            rospy.Subscriber(Constants.AR.Stereo, ARMarkers, self.arCallback)

            # foam callback
            rospy.Subscriber(Constants.Foam.Topic, PointStamped, self.foamCallback)

            # tape callback
            rospy.Subscriber(Constants.GripperTape.Topic, PoseStamped, self.tapeCallback)

            rospy.sleep(2)
        
      def setState(self, state):
            self.state = state

      def arCallback(self, msg):
            self.locks['ar_pose'].acquire()
            markers = msg.markers
            for marker in markers:
                markerType = id_map.get(marker.id)
                if markerType != None:
                    continue
                self.arHandlerWithOrientation(marker, "left")
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

        self.markerPose = pose

        if self.tapePoseRecent(marker.header.stamp, APPROX_TIME):
            self.registerTransform(pose, marker.id)
        elif not self.tapePoseRecent(rospy.Time.now(), 0):
            transform = self.transforms.setdefault(marker.id) # gets the transform or returns None
            if transform != None:
                estimatedPose = transform
                estimatedPose.header.stamp = rospy.Time.now()
                self.estimatedPosePub.publish(estimatedPose)
                if armname == "left":
                    self.leftGripperPose = estimatedPose
                    self.newLeftGripperPose = True
                else:
                    self.rightGripperPose = estimatedPose
                    self.newRightGripperPose = True
                self.gripperPoseIsEstimate = True

      def registerTransform(self, pose, id_):
        rospy.loginfo('registering transform for id %d', id_)
        stereoFrame = 'stereo_' + str(id_)
        self.tapeMsg.header.stamp = self.listener.getLatestCommonTime(stereoFrame, self.tapeMsg.header.frame_id)
        transform = self.listener.transformPose(stereoFrame, self.tapeMsg)
        self.transforms[id_] = transform

      def tapePoseRecent(self, comparisonStamp, maxTime):
        if not self.tapeMsg:
            return False

        tapeStamp = self.tapeMsg.header.stamp
        timeDiff = (comparisonStamp - tapeStamp).to_sec()
        return timeDiff < maxTime

      def isCalibrated(self):
            return self.state == ARImageDetectionClass.State.Calibrated

def testTransforms():
    rospy.init_node('ar_image_detection')
    imageDetector = ARImageDetectionClass()
    imageDetector.markerPose = None

    #print 'about to spin'
    #rospy.spin()
    while not rospy.is_shutdown():
        if imageDetector.markerPose:
            imageDetector.tapeMsg = imageDetector.markerPose
            imageDetector.tapeMsg.pose.position.x += 0.1
            imageDetector.tapeMsg.pose.orientation.y += 0.1
            print imageDetector.tapeMsg.pose.position.x, imageDetector.markerPose.pose.position.x    
        rospy.sleep(.5)

def testObjectPoint():
      """
      Prints when an objectPoint has been detected
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

def testRotation():
    rospy.init_node('ar_image_detection')

    imageDetector = ARImageDetectionClass()
    listener = tf.TransformListener()
    tf_br = tf.TransformBroadcaster()
    

    while not rospy.is_shutdown():
          if imageDetector.hasFoundGripper(Constants.Arm.Left):
                obj = tfx.pose([0,0,0], imageDetector.normal).msg.PoseStamped()
                gripper = imageDetector.getGripperPose(Constants.Arm.Left)

                print('gripper')
                print(gripper)

                # obj_tb = tfx.tb_angles(obj.pose.orientation)
                gripper_tb = tfx.tb_angles(gripper.pose.orientation)
                print "gripper ori", gripper_tb
                obj_tb = tfx.tb_angles(imageDetector.normal)
                print "obj ori", obj_tb
                pt = gripper.pose.position
                ori = imageDetector.normal
                tf_br.sendTransform((pt.x, pt.y, pt.z), (ori.x, ori.y, ori.z, ori.w),
                                    gripper.header.stamp, '/des_pose', Constants.AR.Frames.Base)
                
                
                between = Util.angleBetweenQuaternions(ori, gripper_tb.msg)
                print('Angle between')
                print(between)

                quat = tft.quaternion_multiply(gripper_tb.quaternion, tft.quaternion_inverse(obj_tb.quaternion))
                print 'new', tfx.tb_angles(quat)

                #rot = gripper_tb.rotation_to(ori)
                rot = gripper_tb.rotation_to(obj_tb)
                print('Rotation from gripper to obj')
                print(rot)

                deltaPoseTb = tfx.pose(Util.deltaPose(gripper, obj)).orientation
                print('deltaPose')
                print(deltaPoseTb)

                deltaPose = tfx.pose([0,0,0], deltaPoseTb).msg.PoseStamped()
                time = listener.getLatestCommonTime('0_link', 'tool_L')
                deltaPose.header.stamp = time
                deltaPose.header.frame_id = '0_link'
                deltaPose = listener.transformPose('tool_L', deltaPose)
                print "transformed", tfx.tb_angles(deltaPose.pose.orientation)

                endQuatMat = gripper_tb.matrix * rot.matrix
                print 'desOri', tfx.tb_angles(endQuatMat)
                

          rospy.sleep(1)

if __name__ == '__main__':
    #testCalibration()
    #testFound()
    #testFoundGripper()
    #testObjectPoint()
    #testRotation()
    testTransforms()

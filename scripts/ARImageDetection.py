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

ids_to_joints = {73: Constants.AR.Frames.Grasper1,
                 53: Constants.AR.Frames.Grasper2,
                 22: Constants.AR.Frames.Right,
                 13: Constants.AR.Frames.Cube1,
                 87: Constants.AR.Frames.Cube2,
                 93: Constants.AR.Frames.Cube3,
                 12: Constants.AR.Frames.Cube4,
                 33: Constants.AR.Frames.Object}

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

            # image processing to find object
            self.listener = tf.TransformListener()
            #self.objectProcessing = ImageProcessingClass()

            self.locks = dict()
            self.locks['ar_pose'] = Lock()

            # Temporary. For finding the receptacle
            # rospy.Subscriber(Constants.StereoClick.StereoName, PointStamped, self.stereoCallback)
            # Get grippers using AR
            self.debugArCount = 0
            rospy.Subscriber(Constants.AR.Stereo, ARMarkers, self.arCallback)

            # foam callback
            # will bring back after debugging foam segmentation
            rospy.Subscriber(Constants.Foam.Topic, PointStamped, self.foamCallback)

            # tape callback
            rospy.Subscriber(Constants.GripperTape.Topic, PoseStamped, self.tapeCallback)

            rospy.sleep(2)
        
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
                elif frame == Constants.AR.Frames.Right:
                    self.arHandlerWithOrientation(marker, "right")
                elif frame == Constants.AR.Frames.Object:
                    pose = PoseStamped()
                    pose.header.stamp = marker.header.stamp
                    pose.header.frame_id = marker.header.frame_id
                    pose.pose = marker.pose.pose

                    #self.listener.waitForTransform(Constants.AR.Frames.Base,marker.header.frame_id,pose.header.stamp,rospy.Duration(5))
                    #pose = self.listener.transformPose(Constants.AR.Frames.Base,pose)

                    tf_frame_to_base = tfx.lookupTransform(Constants.AR.Frames.Base, marker.header.frame_id, wait=10)
                    pose = tf_frame_to_base * tfx.pose(pose)
                    pose = pose.msg.PoseStamped()
                    

                    point = PointStamped()
                    point.header.stamp = marker.header.stamp
                    point.header.frame_id = Constants.AR.Frames.Base
                    point.point = pose.pose.position
                    # move pick-up position back
                    point.point.y -= .01
                    point.point.z += .01
                    #self.normal = pose.pose.orientation
                    #self.normal = Util.reverseQuaternion(pose.pose.orientation)

                    #tf_frame_to_53 = tfx.lookupTransform('/stereo_53',marker.header.frame_id)
                    #point = tfx.point(tf_frame_to_53 * tfx.point(point.point)).msg.PointStamped()
                    #point.header.frame_id = '/stereo_53'
                    #point.header.stamp = marker.header.stamp

                    # TEMP
                    self.objectPoint = point
                    
                    #self.normal = tfx.tb_angles(marker.pose.pose.orientation).msg
                    #self.normal = tfx.tb_angles(tfx.tb_angles(0,90,0).matrix*tfx.tb_angles(marker.pose.pose.orientation).matrix).msg
                    #self.normal = Util.reverseQuaternion(marker.pose.pose.orientation)

                    #self.normal = tfx.tb_angles(0,0,-90).msg
                elif ids_to_joints[marker.id] == Constants.Arm.Right:
                    self.arHandlerWithOrientation(marker, "right")
            self.locks['ar_pose'].release() 

      def calibrate(self, markers):
          d = {}
          for marker in markers:
             d[marker.id] = marker
          if d[2] and d[6]:
             transforms[(6,2)] = Utils.transform(d[6], d[2])
          

      def debugAr(self, gp):
        self.debugArCount += 1
        if self.debugArCount % 10 == 0:
            print self.listener.transformPose(Constants.Frames.RightTool, gp)

      def arHandlerWithOrientation(self, marker, armname):
        pose = PoseStamped()
        pose.header.stamp = marker.header.stamp
        pose.header.frame_id = marker.header.frame_id
        pose.pose = marker.pose.pose

        """
        # rotation y axis 90 deg z axis -90 deg
        poseMat = tfx.tb_angles(pose.pose.orientation).matrix
        rot = tft.rotation_matrix(-math.pi/2,[1,0,0])[0:3,0:3]
        #print('rot')
        #print(rot)
        rot1 = tfx.tb_angles(0,0,-90).matrix
        #print('rot1')
        #print(rot1)
        newOrientation = np.dot(rot1,poseMat)
        #newOrientation = np.dot(tft.rotation_matrix(math.pi/2,[0,0,1])[0:3,0:3],newOrientation)
        pose.pose = tfx.pose(pose.pose.position,newOrientation).msg.Pose()
        """

        # try a different way
        # frame='/stereo_33',stamp=marker.header.stamp
        #pose = tfx.pose([0,0,0],tfx.tb_angles(0,0,-0).matrix,frame='/stereo_53',stamp=marker.header.stamp)
        #pose = self.listener.transformPose('left_optical_frame',pose.msg.PoseStamped())
        #self.listener.waitForTransform(Constants.AR.Frames.Base,'/stereo_53',marker.header.stamp,rospy.Duration(5))
        #pose = self.listener.transformPose(Constants.AR.Frames.Base,pose.msg.PoseStamped())

        # pose = tfx.pose([0,0,0],tfx.tb_angles(0,0,-0).matrix,frame='/stereo_33',stamp=marker.header.stamp)
        # pose = tfx.pose([0,0,0],tfx.tb_angles(0,0,-90).matrix,frame='/stereo_33',stamp=marker.header.stamp)
        #pose = self.listener.transformPose('left_optical_frame',pose.msg.PoseStamped())

        """
        try:
            self.listener.waitForTransform(Constants.AR.Frames.Base,'/stereo_53',marker.header.stamp,rospy.Duration(5))
            pose = self.listener.transformPose(Constants.AR.Frames.Base,pose)
        except Exception as e:
            print e
            return
        """

        # pose = self.listener.transformPose(Constants.AR.Frames.Base,pose.msg.PoseStamped())

        #tf_frame_to_53 = tfx.lookupTransform('/stereo_53',marker.header.frame_id,True)
        #pose = tfx.pose(tf_frame_to_53 * pose).msg.PoseStamped()
        #pose.header.frame_id = '/stereo_53'
        #pose.header.stamp = marker.header.stamp

        #pose.pose.orientation = Util.reverseQuaternion(pose.pose.orientation)


        if armname == "left":
            self.leftGripperPose = pose
            self.newLeftGripperPose = True
        else:
            self.rightGripperPose = pose
            self.newRightGripperPose = True

      def isCalibrated(self):
            return self.state == ARImageDetectionClass.State.Calibrated

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
    testRotation()

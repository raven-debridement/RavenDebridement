#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy

import tf
import tf.transformations as tft
import tfx

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from raven_2_msgs.msg import *
from visualization_msgs.msg import Marker

import Util
import Constants

import code


class ImageDetectionClass():
    """
    Used to detect object, grippers, and receptacle
    """

    def __init__(self, normal=None):
        # PointStamped
        self.objectPoint = None

        # gripper pose. Must both have frame_id of respective tool frame
        self.leftGripperPose = None
        self.rightGripperPose = None

        self.newLeftGripperPose = False
        self.newRightGripperPose = False
        #receptacle point. Must have frame_id of link_0
        #is the exact place to drop off (i.e. don't need to do extra calcs to move away
        # WARNING this is hardcoded for the left arm only
        self.receptaclePoint = tfx.point([-.025,-.004,-.093],frame=Constants.Frames.Link0).msg.PointStamped()
        #table normal. Must be according to global (or main camera) frame
        if normal != None:
            self.normal = normal
        else:
            # default to straight up
            self.normal = Util.makeQuaternion(.5**.5, 0, -.5**.5, 0)

        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()
        self.registerObjectPublisher()

        # hardcode receptacle point

        #######################
        # MAY NEED TO USE LOCKS
        #######################

        # Temporary. Will eventually be placed with real image detection
        # Will subscribe to camera feeds eventually 
        rospy.Subscriber(Constants.StereoClick.StereoName, PointStamped, self.stereoCallback)
        #rospy.Subscriber(Constants.RavenTopics.RavenState, RavenState, self.ravenStateCallback)

        rospy.Subscriber(Constants.Foam.Topic, PointStamped, self.foamCallback)
        rospy.Subscriber(Constants.GripperTape.Topic, PoseStamped, self.tapeCallback)

    def registerObjectPublisher(self):
        object_topic = "object_marker"
        self.objPublisher = rospy.Publisher(object_topic, Marker)
        self.objMarker = None

    def tapeCallback(self, msg):
        # TEMP hard coded for left gripper
        self.newLeftGripperPose = True
        self.tapeMsg = msg
        self.leftGripperPose = msg
        self.gripperPoseIsEstimate = False

    def foamCallback(self, msg):
        self.listener.waitForTransform(Constants.AR.Frames.Base,msg.header.frame_id,msg.header.stamp,rospy.Duration(5))
        self.objectPoint = self.listener.transformPoint(Constants.AR.Frames.Base,msg)

        """
        tfxMsg = tfx.point(msg)
        tf_msgframe_to_base = tfx.lookupTransform(Constants.AR.Frames.Base, tfxMsg.frame, wait=10)
        tfxMsg = tf_msgframe_to_base * tfxMsg
        self.objectPoint = tfxMsg.msg.PointStamped()
        """

        pose = self.getObjectPose()
        pos = pose.position
        ori = pose.orientation
        self.tf_br.sendTransform((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w),
                                 rospy.Time.now(), 'object_frame', pose.header.frame_id)
        marker = Util.createMarker(self.getObjectPose(), 0)
        self.objPublisher.publish(marker)
        
        # to account for gripper being open
        #self.objectPoint.point.z += .004

        marker = Util.createMarker(self.getObjectPose(), 1)
        self.objPublisher.publish(marker)

    def stereoCallback(self, msg):
        """
        Temporary. First click sets receptaclePoint, all others are objectPoints
        
        Also, always sets gripper poses
        """
        if self.receptaclePoint == None:
            self.receptaclePoint = msg 
            self.receptaclePoint.point.z += .1
        else:
            #msg.point.z -= .03 # so gripper doesn't pick up right on the edge
            self.objectPoint = msg
            
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.002
            marker.scale.y = 0.002
            marker.scale.z = 0.002
            marker.color.a = 1.0
            marker.color.r = 255
            marker.color.g = 255
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = msg.point.x
            marker.pose.position.y = msg.point.y
            marker.pose.position.z = msg.point.z
            self.objMarker = marker
            self.objPublisher.publish(marker)
        
    def ravenStateCallback(self, msg):
        # gripperPoses in own frames
        rgp = PoseStamped()
        rgp.header.stamp = msg.header.stamp
        rgp.header.frame_id = Constants.ToolFrame.Right
        rgp.pose.orientation.w = 1
        self.rightGripperPose = rgp
        
        lgp = PoseStamped()
        lgp.header.stamp = msg.header.stamp
        lgp.header.frame_id = Constants.ToolFrame.Left
        lgp.pose.orientation.w = 1
        self.leftGripperPose = lgp

    def gripperPoseEstimated(self):
        return self.gripperPoseIsEstimate

    def hasFoundObject(self):
        return self.objectPoint != None

    def getObjectPose(self,desFrame=None):
        """
        Returns object point plus the table normal as the orientation
        
        Returns None if no object found
        """
        if not self.hasFoundObject():
            return None

        objectPoint = tfx.point(self.objectPoint, stamp=rospy.Time.now())
        tf_point_to_normal = tfx.lookupTransform(Constants.Frames.Link0, objectPoint.frame, wait=10)
        objectPoint = tf_point_to_normal * objectPoint


        objectPose = tfx.pose(self.objectPoint, self.normal)


        """
        if desFrame != None:
            tf_obj_to_tool = tfx.lookupTransform(desFrame, objectPose.frame, wait=5)
            objectPose = tf_obj_to_tool * objectPose
        """

        return objectPose.msg.PoseStamped()
        
    def getObjectPoint(self,desFrame=None):
        """
        May update to take argument currPos, and then choose object closest to currPos
            
        Also, keep track of object points and not object poses because we assume the object will be on a flat table.

        Returns None if no object found
        """
        if not self.hasFoundObject():
            return None

        
        objectPoint = tfx.pose(self.getObjectPose(desFrame)).position

        return objectPoint.msg.PointStamped()
            
      
    def removeObjectPoint(self):
        #Debug tool to remove object point
        self.objectPoint = None
      

    def hasFoundGripper(self, armName):
        """
        armName must be from Constants.Arm
        """
        if armName == Constants.Arm.Left:
            return (self.leftGripperPose != None)
        else:
            return (self.rightGripperPose != None)

    def hasFoundNewGripper(self, armName):
        """
        armName must be from Constants.Arm

        Returns true if new gripper location since
        last call to getGripperPose
        """
        if not self.hasFoundGripper(armName):
            return False

        if armName == Constants.Arm.Left:
            return self.newLeftGripperPose
        else:
            return self.newRightGripperPose

    def getGripperPose(self, armName):
        """
        armName must be from Constants.Arm
        
        returns PoseStamped with frame_id of gripperName
        """
        if not self.hasFoundGripper(armName):
            return None

        # may want to make a copy of gripper pose eventually
        if armName == Constants.Arm.Left:
            self.newLeftGripperPose = False
            return self.leftGripperPose
            #frame = Constants.Frames.LeftTool
            #gripper = tfx.pose(self.leftGripperPose)
        else:
            self.newRightGripperPose = False
            return self.rightGripperPose
            #frame = Constants.Frames.RightTool
            #gripper =  tfx.pose(self.rightGripperPose)

        """
        tf_arm_to_tool = tfx.lookupTransform(frame, gripper.frame, wait=5)
        gripper = tf_arm_to_tool * gripper

        return gripper.msg.PoseStamped()
        """

    def getGripperPoint(self, armName):
        """
        armName must be from Constants.Arm
          
        returns PointStamped with frame_id of gripperName
        """
        if not self.hasFoundGripper(armName):
            return None

        return tfx.pose(self.getGripperPose(armName)).position.msg.PointStamped()
      
      
    def hasFoundReceptacle(self):
        return (self.receptaclePoint != None)

    def getReceptaclePose(self):
        """
        Returns PoseStamped with position of centroid of receptacle and
        orientation of the table normal
        """
        receptaclePoint = self.getReceptaclePoint()

        return tfx.pose(self.receptaclePoint, self.normal).msg.PoseStamped()
        #return Util.pointStampedToPoseStamped(self.receptaclePoint, self.normal)

    def getReceptaclePoint(self):
        """
        Returns PointStamped of the centroid of the receptacle
        """
        self.receptaclePoint.header.stamp = rospy.Time.now()
        return self.receptaclePoint

def test():     
    rospy.init_node('image_detection_node')
    imageDetector = ImageDetectionClass()
    while not rospy.is_shutdown():
        if imageDetector.hasFoundObject():
            objectPoint = imageDetector.getObjectPoint()
            print(objectPoint)
        else:
            print('Not Found')

        rospy.sleep(.5)
      

if __name__ == '__main__':
    test()

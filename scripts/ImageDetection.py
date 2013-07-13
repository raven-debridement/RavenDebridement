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
        #receptacle point. Must have frame_id of global (or main camera) frame
        #is the exact place to drop off (i.e. don't need to do extra calcs to move away
        self.receptaclePoint = None
        #table normal. Must be according to global (or main camera) frame
        if normal != None:
            self.normal = normal
        else:
            # default to straight up
            self.normal = Util.makeQuaternion(.5**.5, 0, -.5**.5, 0)

        #######################
        # MAY NEED TO USE LOCKS
        #######################

        # Temporary. Will eventually be placed with real image detection
        # Will subscribe to camera feeds eventually 
        rospy.Subscriber(Constants.StereoClick.StereoName, PointStamped, self.stereoCallback)
        #rospy.Subscriber(Constants.RavenTopics.RavenState, RavenState, self.ravenStateCallback)

    def registerObjectPublisher(self):
        object_topic = "object_marker"
        self.objPublisher = rospy.Publisher(object_topic, Marker)
        self.objMarker = None

    def stereoCallback(self, msg):
        """
        Temporary. First click sets receptaclePoint, all others are objectPoints
        
        Also, always sets gripper poses
        """
        if self.receptaclePoint == None:
            self.receptaclePoint = msg 
            self.receptaclePoint.point.z += .2
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


    def hasFoundObject(self):
        return self.objectPoint != None

    def getObjectPose(self):
        """
        Returns object point plus the table normal as the orientation
        
        Returns None if no object found
        """
        if not self.hasFoundObject():
            return None
        
        objectPoint = self.getObjectPoint()
        return Util.pointStampedToPoseStamped(objectPoint, self.normal)
      
    def getObjectPoint(self):
        """
        May update to take argument currPos, and then choose object closest to currPos
            
        Also, keep track of object points and not object poses because we assume the object will be on a flat table.

        Returns None if no object found
        """
        if not self.hasFoundObject():
            return None

        
        # may want to make a copy of self.objectPoint later
        objectPoint = self.objectPoint
        objectPoint.header.stamp = rospy.Time.now()
        
        return objectPoint
            
      
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

    def getGripperPose(self, armName):
        """
        armName must be from Constants.Arm
        
        returns PoseStamped with frame_id of gripperName
        """
        if not self.hasFoundGripper(armName):
            return None

        # may want to make a copy of gripper pose eventually
        if armName == Constants.Arm.Left:
            return self.leftGripperPose
        else:
            return self.rightGripperPose


    def getGripperPoint(self, armName):
        """
        armName must be from Constants.Arm
          
        returns PointStamped with frame_id of gripperName
        """
        if not self.hasFoundGripper(armName):
            return None

        return Util.poseStampedToPointStamped(self.getGripperPose(armName))
      
      
    def hasFoundReceptacle(self):
        return (self.receptaclePoint != None)

    def getReceptaclePose(self):
        """
        Returns PoseStamped with position of centroid of receptacle and
        orientation of the table normal
        """
        return Util.pointStampedToPoseStamped(self.receptaclePoint, self.normal)

    def getReceptaclePoint(self):
        """
        Returns PointStamped of the centroid of the receptacle
        """
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

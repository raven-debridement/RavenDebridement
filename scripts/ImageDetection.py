#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy

import tf
import tf.transformations as tft
import tfx

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion

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
        rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

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

        # gripperPoses in own frames
        rgp = PoseStamped()
        rgp.header.stamp = msg.header.stamp
        rgp.header.frame_id = ConstantsClass.ToolFrame.Right
        rgp.pose.orientation.w = 1
        self.rightGripperPose = rgp
        
        lgp = PoseStamped()
        lgp.header.stamp = msg.header.stamp
        lgp.header.frame_id = ConstantsClass.ToolFrame.Left
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
        objectPoint = imageDetector.getObjectPoint()
        if objectPoint != None:      
            print(objectPoint)
        else:
            print('Not Found')
        rospy.sleep(.5)
      

if __name__ == '__main__':
    test()

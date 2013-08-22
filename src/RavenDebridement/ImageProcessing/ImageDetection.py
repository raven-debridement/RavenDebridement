#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import tf
import tf.transformations as tft
import tfx

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from raven_2_msgs.msg import *
from visualization_msgs.msg import Marker

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants

import code


class ImageDetector():
    """
    Used to detect object, grippers, and receptacle
    """

    def __init__(self, normal=None):

        self.objectPoint = None

        # gripper pose. Must both have frame_id of respective tool frame
        self.leftGripperPose = None
        self.rightGripperPose = None

        self.newLeftGripperPose = False
        self.newRightGripperPose = False

        # home position. likely in front of the camera, close
        self.homePoint = tfx.point([.005,.016,-.095],frame=Constants.Frames.Link0).msg.PointStamped()
            
        #receptacle point. Must have frame_id of link_0
        #is the exact place to drop off (i.e. don't need to do extra calcs to move away
        self.receptaclePoint = tfx.point([.039, .045, -.173],frame=Constants.Frames.Link0).msg.PointStamped()

        #table normal. Must be according to global (or main camera) frame
        if normal != None:
            self.normal = normal
        else:
            # for tape side
            self.normal = tfx.tb_angles(-90,90,0).msg
            # for non-tape side
            #self.normal = tfx.tb_angles(90,90,0).msg

        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()
        self.registerObjectPublisher()
        
        rospy.Subscriber(Constants.Foam.Topic, PointStamped, self.foamCallback)
        rospy.Subscriber(Constants.GripperTape.Topic, PoseStamped, self.tapeCallback)

        rospy.sleep(2)

    def registerObjectPublisher(self):
        object_topic = "object_marker"
        self.objPublisher = rospy.Publisher(object_topic, Marker)
        self.objMarker = None

    def tapeCallbackLeft(self, msg):
        # TEMP hard coded for left gripper
        self.newLeftGripperPose = True
        #self.newRightGripperPose = True
        self.tapeMsg = msg
        self.leftGripperPose = msg
        #self.rightGripperPose = msg
        self.gripperPoseIsEstimate = False

    def tapeCallbackRight(self, msg):
        # TEMP hard coded for left gripper
        self.newRightGripperPose = True
        #self.newRightGripperPose = True
        self.tapeMsg = msg
        self.rightGripperPose = msg
        #self.rightGripperPose = msg
        self.gripperPoseIsEstimate = False

    def foamCallback(self, msg):
        time = self.listener.getLatestCommonTime(Constants.AR.Frames.Base, msg.header.frame_id)
        msg.header.stamp = time
        self.objectPoint = self.listener.transformPoint(Constants.AR.Frames.Base,msg)

        """
        tfxMsg = tfx.point(msg)
        tf_msgframe_to_base = tfx.lookupTransform(Constants.AR.Frames.Base, tfxMsg.frame, wait=10)
        tfxMsg = tf_msgframe_to_base * tfxMsg
        self.objectPoint = tfxMsg.msg.PointStamped()
        """

        pose = self.getObjectPose()
        pos = pose.pose.position
        ori = pose.pose.orientation
        self.tf_br.sendTransform((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w),
                                 rospy.Time.now(), 'object_frame', pose.header.frame_id)
        marker = Util.createMarker(self.getObjectPose(), 0)
        self.objPublisher.publish(marker)
        
        
    def gripperPoseEstimated(self):
        return self.gripperPoseIsEstimate

    def hasFoundObject(self):
        return self.objectPoint != None

    def getObjectPose(self):
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
        """
        Call this in order to wait for the newest object point
        """
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

    def ignoreOldGripper(self, armName):
        if armName == Constants.Arm.Left:
            self.newLeftGripperPose = False
        else:
            self.newRightGripperPose = False

    def getGripperPose(self, armName):
        """
        armName must be from Constants.Arm
        
        returns PoseStamped with frame_id of gripperName
        """
        if not self.hasFoundGripper(armName):
            return None

        if armName == Constants.Arm.Left:
            self.newLeftGripperPose = False
            return self.leftGripperPose
        else:
            self.newRightGripperPose = False
            return self.rightGripperPose



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
        
    def getReceptaclePoint(self):
        """
        Returns PointStamped of the centroid of the receptacle
        """
        self.receptaclePoint.header.stamp = rospy.Time.now()
        return self.receptaclePoint

    def hasFoundHome(self):
        return (self.homePoint != None)

    def getHomePose(self):
        """
        Returns PoseStamped with position of the home position and
        orientation of the table normal
        """
        homePoint = self.getHomePoint()

        return tfx.pose(self.homePoint, self.normal).msg.PoseStamped()
        
    def getHomePoint(self):
        """
        Returns PointStamped of the home position
        """
        self.homePoint.header.stamp = rospy.Time.now()
        return self.homePoint


def test():     
    rospy.init_node('image_detection_node')
    imageDetector = ImageDetector()
    while not rospy.is_shutdown():
        if imageDetector.hasFoundObject():
            objectPoint = imageDetector.getObjectPoint()
            print(objectPoint)
        else:
            print('Not Found')

        rospy.sleep(.5)
      

if __name__ == '__main__':
    test()

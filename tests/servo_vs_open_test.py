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

class ServoVsOpenTest():
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
                
        # manually add by checking print_state
        self.startPose = None

        # ar_frame is the target
        self.ar_frame = '/stereo_32'
        self.objectPose = tfx.pose([0,0,0],tfx.tb_angles(-90,90,0),frame=self.ar_frame)
        
        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()
        self.delta_pub = rospy.Publisher('delta_pose', PoseStamped)
        self.obj_pub = rospy.Publisher('object_pose', PoseStamped)

        rospy.sleep(3)

    def publishObjPose(self, pose):
        self.obj_pub.publish(pose)
        """
        pos = pose.position
        ori = pose.orientation
        self.tf_br.sendTransform((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w),
                                 rospy.Time.now(), 'object_frame', pose.frame)
        """

    def publishDeltaPose(self, delta_pose, gripper_pose):
        frame = gripper_pose.frame
        time = self.listener.getLatestCommonTime(frame, Constants.Frames.Link0)
        pose_stamped = gripper_pose.msg.PoseStamped()
        pose_stamped.header.stamp = time
        gripper_pose = self.listener.transformPose(Constants.Frames.Link0, gripper_pose.msg.PoseStamped())

        pose = PoseStamped()
        pose.header.frame_id = Constants.Frames.Link0
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = (gripper_pose.pose.position + delta_pose.position).msg.Point()

        ori = tfx.pose(gripper_pose).orientation.matrix * delta_pose.orientation.matrix
        pose.pose.orientation = tfx.pose([0,0,0], ori).orientation.msg.Quaternion()
        self.delta_pub.publish(pose)
        
    def openTest(self):
        self.gripperControl.start()

        while not rospy.is_shutdown():
            if self.imageDetector.hasFoundGripper(self.arm):

                gripperPose = self.imageDetector.getGripperPose(self.arm)
                objectPose = tfx.pose(self.objectPose,stamp=rospy.Time.now())
                
                self.publishObjPose(self.objectPose)

                deltaPose = Util.deltaPose(gripperPose, objectPose, self.transFrame, self.rotFrame)
                deltaPose.position.z += .02

                ravenGripperPose = self.gripperControl.getGripperPose(Constants.Frames.Link0)

                rospy.loginfo('Press enter to move to ' + self.ar_frame)
                raw_input()

                self.gripperControl.goToGripperPoseDelta(ravenGripperPose, deltaPose)
                break
            
            rospy.sleep(.1)

        rospy.spin()

    def servoTest(self):

        self.gripperControl.start()

        homePose = tfx.pose([-0.047, -0.029, -0.116],{'yaw':79.9, 'pitch':47.1, 'roll':-1.7}, frame=Constants.Frames.Link0)
        self.gripperControl.setHomePose(homePose)
        print 'home pose', self.gripperControl.getHomePose()
        self.gripperControl.goToHomePose()

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

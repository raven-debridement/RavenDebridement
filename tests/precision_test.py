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

class PrecisionTest():
    def __init__(self):
        self.stereoClickPose = None
        self.foamPose = None

        rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoClickCallback)
        rospy.Subscriber('foam_points', PointStamped, self.foamCallback)

    def stereoClickCallback(self, msg):
        self.stereoClickPose = tfx.pose(msg).msg.PoseStamped()

    def foamCallback(self, msg):
        self.foamPose = tfx.pose(msg).msg.PoseStamped()

    def run(self):
        arm = Constants.Arm.Left
        listener = tf.TransformListener()
        ar_frames = ['stereo_36', 'stereo_33', 'stereo_35']
    
        if arm == Constants.Arm.Left:
            toolframe = Constants.Frames.LeftTool
        else:
            toolframe = Constants.Frames.RightTool

        transFrame = Constants.Frames.Link0
        rotFrame = toolframe
            
        rospy.init_node('precision_test_node',anonymous=True)
        gripperControl = GripperControlClass(arm, listener)
        imageDetector = ARImageDetectionClass()
        rospy.sleep(4)

        gripperControl.start()

        """
        while not rospy.is_shutdown():
            if imageDetector.hasFoundGripper(arm) and self.foamPose != None:
                rospy.loginfo('Press enter to move')
                raw_input()

                gripperPose = imageDetector.getGripperPose(arm)
                
                    
                deltaPose = Util.deltaPose(gripperPose, self.foamPose, transFrame, rotFrame)
                deltaPose.position.z += .01
                code.interact(local=locals())

                duration = 6
                gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(Constants.Frames.Link0), deltaPose, duration=duration, ignoreOrientation=True)
                rospy.sleep(duration)
                
                self.foamPose = None
                

            rospy.sleep(.5)
        """
        raw_input()
        for ar_frame in ar_frames:
            while not rospy.is_shutdown():
                if imageDetector.hasFoundGripper(arm):
                    rospy.loginfo('Press enter to move to ' + ar_frame)
                    rospy.sleep(1)
                    #raw_input()

                    gripperPose = imageDetector.getGripperPose(arm)
                    
                    arPose = tfx.pose([0,0,0],tfx.tb_angles(-90,90,0),frame=ar_frame)
                    
                    deltaPose = Util.deltaPose(gripperPose, arPose, transFrame, rotFrame)
                    deltaPose.position.z += .02
                    #code.interact(local=locals())

                    duration = 6
                    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(Constants.Frames.Link0), deltaPose, duration=duration, ignoreOrientation=False)
                    rospy.sleep(duration)
                    break

                rospy.sleep(.5)
        

if __name__ == '__main__':
    pt = PrecisionTest()
    pt.run()

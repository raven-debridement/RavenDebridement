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
        self.rotFrame = toolframe
            
        self.gripperControl = GripperControlClass(arm, listener)
        self.imageDetector = ARImageDetectionClass()
                
        # manually add by checking print_state
        self.startPose = None

        # ar_frame is the target
        self.ar_frame = '/stereo_32'
        self.objectPose = tfx.pose([0,0,0],tfx.tb_angles(-90,90,0),frame=self.ar_frame)

        rospy.sleep(3)
        
    def openTest(self):
        gripperControl.start()

        while not rospy.is_shutdown():
            if self.imageDetector.hasFoundGripper(self.arm):
                rospy.loginfo('Press enter to move to ' + self.ar_frame)
                raw_input()

                gripperPose = self.imageDetector.getGripperPose(self.arm)
                objectPose = tfx.pose(self.objectPose,stamp=rospy.Time.now())

                deltaPose = Util.deltaPose(gripperPose, objectPose, self.transFrame, self.rotFrame)
                deltaPose.z += .02

                ravenGripperPose = self.gripperControl.getGripperPose(Constants.Frames.Link0)
                self.gripperControl.goToGripperPoseDelta(ravenGripperPose, deltaPose)
                break
            
            rospy.sleep(.1)

        rospy.spin()


def open_test():
    rospy.init_node('open_test',anonymous=True)
    test = ServoVsOpenTest()
    test.openTest()

if __name__ == '__main__':
    open_test()

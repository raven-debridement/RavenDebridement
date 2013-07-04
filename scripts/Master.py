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
from GripperControl import GripperControlClass

class MasterClass():
    """
    Contains the master pipeline for master-control in the run method

    The general pipeline is as follows:
    - identify the receptacle, object, and gripper
    - open the gripper
    - move to a point near the object
    - servo to the object
    - close the gripper
    - move up with the object
    - move to the receptacle
    - open the gripper
    ... repeat

    The whole pipeline is not currently implemented. The above
    is the final objective.

    If any of the steps fails, the loop goes back to the beginning

    Each pipeline staged is logged using rospy.loginfo
    """
    def __init__(self, armName, imageDetector):
        """
        armName must be from Constants.Arm
        imageDetector is instance of ImageDetectionClass
        """
        self.armName = armName
        
        if (armName == Constants.Arm.Left):
            self.gripperName = Constants.Arm.Left
            self.toolframe = Constants.Frames.LeftTool
            #self.calibrateGripperState = ImageDetectionClass.State.CalibrateLeft
        else:
            self.gripperName = Constants.Arm.Right
            self.toolframe = Constants.Frames.RightTool
            #self.calibrateGripperState = ImageDetectionClass.State.CalibrateRight

        self.listener = tf.TransformListener()

        # initialize the three main control mechanisms
        # image detection, gripper control, and arm control
        self.imageDetector = imageDetector
        #self.imageDetector.setState(self.calibrateGripperState)
        self.gripperControl = GripperControlClass(self.armName, self.listener)
        
    def run(self):
        """
        Loops through the pipeline
        """
        while not rospy.is_shutdown():
            # can change rate
            rospy.sleep(.5)

            # delay between parts of the pipeline
            delay = .5
            # timeout class with 15 second timeout, can change
            timeout = Util.TimeoutClass(15)

            # bounds, can change for each particular
            # pipeline section. keep loose for testing

            # translation bound in meters
            transBound = .1
            # rotation bound in radians
            rotBound = float("inf")


            
            rospy.loginfo('Searching for the receptacle')
            if not self.imageDetector.hasFoundReceptacle():
                continue



            rospy.loginfo('Searching for object point')
            # find object point and pose
            if not self.imageDetector.hasFoundObject():
                continue
            objectPose = self.imageDetector.getObjectPose()
            objectPoint = poseStampedToPointStamped(objectPose)
            


            rospy.loginfo('Searching for ' + self.gripperName)
            # find gripper point
            if self.imageDetector.hasFoundGripper(self.gripperName):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                gripperPoint = poseStampedToPointStamped(gripperPose)
            else:
                continue

            rospy.loginfo('Opening the gripper')
            # open gripper
            if not self.gripperControl.openGripper():
                continue
            
            
            
            rospy.loginfo('Moving to the object point')
            # go to object point
            success = True
            timeout.start()
            while not withinBounds(gripperPose, objectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                self.gripperControl.goToGripperPose(objectPose)
                
                if timeout.hasTimedOut():
                    success = False
                    break
                # must be at least 2 Hz
                rospy.sleep(.1)

            if not success:
                continue
            


            rospy.sleep(delay)
            rospy.loginfo('Closing the gripper')
            # close gripper (consider not all the way)
            if not self.commandGripper.setGripper(.5):
                continue

            

            rospy.sleep(delay)
            rospy.loginfo('Moving vertical with the object')
            # move vertical with the object
            transBound = .05
            rotBound = .1
            vertObjectPose = PoseStamped(objectPose.header, objectPose.pose)
            vertObjectPose.pose.position.z += .1

            # currently have set to no planning, can change
            self.armControl.goToArmPose(vertObjectPose, False)







def mainloop():
    """
    Gets an instance of the MasterClass
    for the left arm and executes the
    run loop
    """
    rospy.init_node('master_node')
    imageDetector = ImageDetectionClass()
    master = MasterClass(Constants.Arm.Left, imageDetector)
    master.run()

if __name__ == '__main__':
    mainloop()
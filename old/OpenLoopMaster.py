#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy
import math

import tf
import tf.transformations as tft
import tfx

from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion
from raven_pose_estimator.srv import ThreshRed

import Util
import Constants
from GripperControl import GripperControlClass
from ImageDetection import ImageDetectionClass
from ARImageDetection import ARImageDetectionClass

import code

class MasterClass():
    """
    Contains the master pipeline for master-control in the run method

    See control_pipeline.jpeg for the pipeline overview
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

        self.receptaclePose = None
        self.homePose = None
        self.objectPose = None
        self.gripperPose = None

        # translation frame
        self.transFrame = Constants.Frames.Link0
        # rotation frame
        self.rotFrame = self.toolframe

        # height offset for foam
        self.objectHeightOffset = -.005

        # in cm/sec, I think
        self.openLoopSpeed = .03

        self.gripperOpenCloseDuration = 2.5

        # debugging outputs
        self.des_pose_pub = rospy.Publisher('desired_pose', PoseStamped)
        self.obj_pub = rospy.Publisher('object_pose', PoseStamped)

        self.timeout = Util.Timeout(999999999)

    def findReceptacle(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.findReceptacle
        successMethod = successMethod or self.findHome

        rospy.loginfo('Searching for the receptacle')
        if not self.imageDetector.hasFoundReceptacle():
            rospy.loginfo('Did not find receptacle')
            return failMethod
        self.receptaclePose = self.imageDetector.getReceptaclePose()

        rospy.loginfo('Found receptacle')
        return successMethod

    def findHome(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.findHome
        successMethod = successMethod or self.moveToReceptacle

        rospy.loginfo('Searching for the home position')
        if not self.imageDetector.hasFoundHome():
            rospy.loginfo('Did not find home position')
            return failMethod
        self.homePose = self.imageDetector.getHomePose()

        rospy.loginfo('Found home position')
        return successMethod


    def findObject(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.findObject
        successMethod = successMethod or self.findGripper

        rospy.loginfo('Searching for object point')
        # find object point and pose
        if not self.imageDetector.hasFoundObject():
            rospy.loginfo('Did not find object')
            return failMethod
        # get object w.r.t. toolframe
        self.objectPose = self.imageDetector.getObjectPose(self.toolframe)

        objectPose = tfx.pose(self.objectPose)
        tf_obj_to_link0 = tfx.lookupTransform(Constants.Frames.Link0, objectPose.frame, wait=5)
        objectPose = tf_obj_to_link0 * objectPose
        self.objectPose = objectPose.msg.PoseStamped()

        self.objectPose.pose.position.z += self.objectHeightOffset
        self.publishObjectPose(self.objectPose)

        rospy.loginfo('Found object')
        return successMethod
        
    def findGripper(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.rotateGripper
        successMethod = successMethod or self.moveNearObject

        rospy.loginfo('Searching for ' + self.gripperName)
        # find gripper point
        self.imageDetector.ignoreOldGripper(self.gripperName)
        rospy.sleep(1)
        if (not self.imageDetector.hasFoundGripper(self.gripperName)) or (not self.imageDetector.hasFoundNewGripper(self.gripperName)):
            rospy.loginfo('Did not find gripper')
            return failMethod
        self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)

        rospy.loginfo('Found gripper')
        return successMethod

    def rotateGripper(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.findGripper
        successMethod = successMethod or self.findGripper

        rotateBy = -30
        
        rospy.loginfo('Rotating the gripper by ' + str(rotateBy) + ' degrees')
        duration = 2
        deltaPose = tfx.pose([0,0,.001], tfx.tb_angles(0,0,rotateBy))
        self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
        rospy.sleep(duration)

        return successMethod
    
    def moveNearObject(self, failMethod=None, successMethod=None):
        """
        Goes near the object

        Returns servoToObject
        """
        failMethod = failMethod or self.moveNearObject
        successMethod = successMethod or self.servoToObject
        
        rospy.loginfo('Moving near the object point')
        # go to near the object point
        
        nearObjectPose = tfx.pose(self.objectPose).msg.Pose()
        nearObjectPose.position.z += .03

        # TEMP
        self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), nearObjectPose, speed=self.openLoopSpeed)
        #self.gripperControl.goToGripperPoseDelta(self.gripperPose.pose, deltaPose, duration=duration)    
        #rospy.sleep(duration)

        while not self.gripperControl.isPaused() and not rospy.is_shutdown():
            rospy.sleep(.1)

        return successMethod

    def servoToObject(self, failMethod=None, successMethod=None):
        """
        In close-loop, servos to the object. Then, closes the gripper.
        """
        failMethod2 = lambda: self.findGripper(self.moveToHome, self.servoToObject)
        failMethod1 = lambda: self.findObject(self.moveToHome, failMethod2)
        failMethod0 = lambda vertAmount: self.moveVertical(self.moveToReceptacle, failMethod1, vertAmount)
        failMethod = failMethod or failMethod0
        successMethod = successMethod or self.moveVertical

        rospy.loginfo('Servoing to the object point')
        # servo to the object point

        self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), self.objectPose, speed=self.openLoopSpeed)
            
        while not self.gripperControl.isPaused() and not rospy.is_shutdown():
            rospy.sleep(.1)

        rospy.loginfo('Closing the gripper')
        # close gripper (consider not all the way)
        self.gripperControl.closeGripper(duration=self.gripperOpenCloseDuration)
        rospy.sleep(self.gripperOpenCloseDuration)
            
        return successMethod

    def moveVertical(self, failMethod=None, successMethod=None, vertAmount=.05):
        """
        Move vertical in open-loop
        """
        failMethod = failMethod or self.checkPickup
        successMethod = successMethod or self.checkPickup

        rospy.loginfo('Moving vertical with the object')
        # move vertical with the object

        nearObjectPose = tfx.pose(self.objectPose).msg.Pose()
        nearObjectPose.position.z += vertAmount

        # TEMP
        self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), nearObjectPose, speed=self.openLoopSpeed)
        
        while not self.gripperControl.isPaused() and not rospy.is_shutdown():
            rospy.sleep(.1)

        return successMethod
    
    def checkPickup(self, failMethod=None, successMethod=None):
        """
        Checks if the grasper picked up a red foam piece
        """
        failMethod = failMethod or (lambda: self.findObject(self.moveToHome, (lambda: self.findGripper(self.moveToHome, self.servoToObject))))
        successMethod = successMethod or (lambda: self.moveToHome(self.moveToReceptacle, self.moveToReceptacle))

        rospy.loginfo('Check if red foam piece successfully picked up')
	rospy.sleep(1)

        try:
            rospy.wait_for_service(Constants.Services.isFoamGrasped, timeout=5)
            foamGraspedService = rospy.ServiceProxy(Constants.Services.isFoamGrasped, ThreshRed)
            isFoamGrasped = foamGraspedService(0).output
            
            if isFoamGrasped == 1:
                rospy.loginfo('Successful pickup!')
                return successMethod
            else:
                rospy.loginfo('Failed pickup')
                
                rospy.loginfo('Opening the gripper')
                # open gripper (consider not all the way)
                self.gripperControl.openGripper(duration=self.gripperOpenCloseDuration)
                rospy.sleep(self.gripperOpenCloseDuration)

                return failMethod
        except:
            rospy.loginfo('Service exception, assuming successful pickup')
            return successMethod
            
            
    
    def moveToReceptacle(self, failMethod=None, successMethod=None):
        """
        Move to the receptacle in open-loop
        Then, open the gripper
        """
        failMethod = failMethod or self.moveToHome
        successMethod = successMethod or self.moveToHome

        rospy.loginfo('Moving to receptacle')
        # move to receptacle with object
        
        
        self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), self.receptaclePose.pose, ignoreOrientation=True, speed=self.openLoopSpeed)


        while not self.gripperControl.isPaused() and not rospy.is_shutdown():
            rospy.sleep(.1)

        #rospy.sleep(duration+2)


        rospy.loginfo('Opening the gripper')
        # open gripper (consider not all the way)
        self.gripperControl.openGripper(duration=self.gripperOpenCloseDuration)
        rospy.sleep(self.gripperOpenCloseDuration)

        
        return successMethod

    def moveToHome(self, failMethod=None, successMethod=None):
        """
        Move to the home position in open-loop
        """
        failMethod = failMethod or self.findObject
        successMethod = successMethod or self.findObject

        rospy.loginfo('Moving to home position')
        # move to home position
        
        self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), self.homePose.pose, ignoreOrientation=True, speed=self.openLoopSpeed)


        while not self.gripperControl.isPaused() and not rospy.is_shutdown():
            rospy.sleep(.1)


        self.imageDetector.removeObjectPoint()
        
        return successMethod
    
    def run(self):
        """
        Loops through the pipeline

        The pipeline consists of the above methods.
        Each method has a fail and success return method.
        """
        delay = .1
        currentStage = self.findReceptacle
        
        self.gripperControl.start()

        while not rospy.is_shutdown():
            rospy.loginfo('Next stage')
            #rospy.loginfo('Press enter')
            #raw_input()
            currentStage = currentStage()
            rospy.sleep(delay)

    def publishObjectPose(self, pose):
        self.obj_pub.publish(pose)

    def publishDesiredPose(self, delta_pose, gripper_pose):
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
        self.des_pose_pub.publish(pose)




def mainloop():
    """
    Gets an instance of the MasterClass
    for the left arm and executes the
    run loop
    """
    rospy.init_node('master_node',anonymous=True)
    imageDetector = ARImageDetectionClass()
    master = MasterClass(Constants.Arm.Left, imageDetector)
    master.run()



if __name__ == '__main__':
    mainloop()
    

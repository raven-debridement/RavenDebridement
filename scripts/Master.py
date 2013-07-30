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
from RavenArm import RavenArm
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
        #self.gripperControl = GripperControlClass(self.armName, self.listener)
        self.ravenArm = RavenArm(self.armName)

        self.receptaclePose = None
        self.homePose = None
        self.objectPose = None
        self.gripperPose = None

        # translation frame
        self.transFrame = Constants.Frames.Link0
        # rotation frame
        self.rotFrame = self.toolframe

        # height offset for foam
        self.objectHeightOffset = .005

        # in cm/sec, I think
        self.openLoopSpeed = .04

        self.gripperOpenCloseDuration = 2.5

        # debugging outputs
        self.des_pose_pub = rospy.Publisher('desired_pose', PoseStamped)
        self.obj_pub = rospy.Publisher('object_pose', PoseStamped)

        self.timeout = Util.Timeout(30)
        self.findGripperTimeout = Util.Timeout(1)

        self.rotateBy = -30

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

        # reset rotateBy
        self.rotateBy = -30

        rospy.loginfo('Searching for object point')
        # find object point and pose
        if not self.imageDetector.hasFoundObject():
            rospy.loginfo('Did not find object')
            return failMethod
        # get object w.r.t. toolframe
        self.objectPose = self.imageDetector.getObjectPose(self.toolframe)
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

        self.findGripperTimeout.start()
        while (not self.imageDetector.hasFoundGripper(self.gripperName)) or (not self.imageDetector.hasFoundNewGripper(self.gripperName)):
            if self.findGripperTimeout.hasTimedOut():
                rospy.loginfo('Did not find gripper')
                return failMethod
            rospy.sleep(.05)

        self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)

        rospy.loginfo('Found gripper')
        return successMethod

    def rotateGripper(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.findGripper
        successMethod = successMethod or self.findGripper
        
        rospy.loginfo('Rotating the gripper by ' + str(self.rotateBy) + ' degrees')
        deltaPose = tfx.pose([0,0,.001], tfx.tb_angles(0,0,self.rotateBy))
        self.ravenArm.goToGripperPoseDelta(deltaPose, duration=2)

        self.rotateBy = -math.copysign(abs(self.rotateBy)+5, self.rotateBy)

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
        
        deltaPose = Util.deltaPose(self.gripperPose, self.objectPose, self.transFrame, self.rotFrame)
        deltaPose.position.z += .03
        
        self.ravenArm.goToGripperPoseDelta(deltaPose, speed=self.openLoopSpeed)

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
            
        transBound = .008
        rotBound = float("inf")

        maxMovement = .015
        deltaPose = uncappedDeltaPose = tfx.pose([0,0,0])

        # if can't find gripper at start, go back to receptacle
        rospy.sleep(1)
        if self.imageDetector.hasFoundGripper(self.gripperName) and self.imageDetector.hasFoundNewGripper(self.gripperName):
            self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
        else:
            return self.moveToHome
            
        self.timeout.start()
        while not Util.withinBounds(self.gripperPose, self.objectPose, transBound, rotBound, self.transFrame, self.rotFrame):
                
            if self.imageDetector.hasFoundNewGripper(self.gripperName):
                rospy.loginfo('paused and found new gripper')
                self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                deltaPose = uncappedDeltaPose = tfx.pose(Util.deltaPose(self.gripperPose, self.objectPose, Constants.Frames.Link0, self.toolframe))
                    
                deltaPose.position = deltaPose.position.capped(maxMovement)
                #if abs(deltaPose.position.z) > maxMovement:
                #    deltaPose.position.z = math.copysign(maxMovement, deltaPose.position.z)

                deltaPose0Link = tfx.pose(Util.deltaPose(self.gripperPose, self.objectPose, Constants.Frames.Link0, Constants.Frames.Link0))
                deltaPose0Link.position = deltaPose.position.capped(maxMovement)
                self.publishDesiredPose(deltaPose0Link, tfx.pose(self.gripperPose))

                rospy.loginfo('delta pose')
                rospy.loginfo(deltaPose)

                self.ravenArm.goToGripperPoseDelta(deltaPose)
                rospy.sleep(1)
            else:
                rospy.loginfo('paused but did NOT find new gripper')
                deltaPose.position = uncappedDeltaPose.position - deltaPose.position
                self.ravenArm.goToGripperPoseDelta(deltaPose, ignoreOrientation=True)
                break
                    
                    
            if self.timeout.hasTimedOut() or rospy.is_shutdown():
                rospy.loginfo('Timed out')
                self.objectHeightOffset = self.objectHeightOffset - .0005 if self.objectHeightOffset > 0 else 0
                return failMethod(.02)
                
            rospy.sleep(.1)

        rospy.loginfo('Closing the gripper')
        # close gripper (consider not all the way)
        self.ravenArm.closeGripper(duration=self.gripperOpenCloseDuration)
            
        return successMethod

    def moveVertical(self, failMethod=None, successMethod=None, vertAmount=.05):
        """
        Move vertical in open-loop
        """
        failMethod = failMethod or self.checkPickup
        successMethod = successMethod or self.checkPickup

        rospy.loginfo('Moving vertical with the object')
        # move vertical with the object
        deltaPose = tfx.pose([0,0,vertAmount]).msg.Pose()
        
        self.ravenArm.goToGripperPoseDelta(deltaPose, speed=self.openLoopSpeed)
        
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
                self.ravenArm.openGripper(duration=self.gripperOpenCloseDuration)
                
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
                
        self.ravenArm.goToGripperPose(self.receptaclePose.pose, ignoreOrientation=True, speed=self.openLoopSpeed)


        rospy.loginfo('Opening the gripper')
        # open gripper (consider not all the way)
        self.ravenArm.openGripper(duration=self.gripperOpenCloseDuration)
                
        return successMethod

    def moveToHome(self, failMethod=None, successMethod=None):
        """
        Move to the home position in open-loop
        """
        failMethod = failMethod or self.findObject
        successMethod = successMethod or self.findObject

        rospy.loginfo('Moving to home position')
        # move to home position
        
        self.ravenArm.goToGripperPose(self.homePose.pose, ignoreOrientation=True, speed=self.openLoopSpeed)

        # so when finding object, find newest one
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
        
        self.ravenArm.start()

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

def rotateTest():
    rospy.init_node('master_node',anonymous=True)
    imageDetector = ARImageDetectionClass()
    master = MasterClass(Constants.Arm.Left, imageDetector)
    master.ravenArm.start()

    while not rospy.is_shutdown():
        rospy.loginfo('Press enter')
        raw_input()
        master.rotateGripper()

    master.ravenArm.stop()


if __name__ == '__main__':
    #mainloop()
    rotateTest()

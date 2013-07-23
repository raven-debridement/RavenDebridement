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

import Util
import Constants
from GripperControl import GripperControlClass
from ImageDetection import ImageDetectionClass
from ARImageDetection import ARImageDetectionClass

import code

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

        self.receptaclePose = None
        self.objectPose = None
        self.gripperPose = None

        # translation frame
        self.transFrame = Constants.Frames.Link0
        # rotation frame
        self.rotFrame = self.toolframe

        # height offset for foam
        self.objectHeightOffset = .004

        self.timeout = Util.TimeoutClass(999999999)

    def findReceptacle(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.findReceptacle
        successMethod = successMethod or self.findObject

        rospy.loginfo('Searching for the receptacle')
        if not self.imageDetector.hasFoundReceptacle():
            rospy.loginfo('Did not find receptacle')
            return failMethod
        self.receptaclePose = self.imageDetector.getReceptaclePose()

        rospy.loginfo('Found receptacle')
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
        self.objectPose.pose.position.z += self.objectHeightOffset

        rospy.loginfo('Found object')
        return successMethod
        
    def findGripper(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.rotateGripper
        successMethod = successMethod or self.moveNearObject

        rospy.loginfo('Searching for ' + self.gripperName)
        # find gripper point
        rospy.sleep(1)
        if not self.imageDetector.hasFoundGripper(self.gripperName):
            rospy.loginfo('Did not find gripper')
            return failMethod
        self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)

        rospy.loginfo('Found gripper')
        return successMethod

    def rotateGripper(self, failMethod=None, successMethod=None):
        failMethod = failMethod or self.findGripper
        successMethod = successMethod or self.findGripper

        rotateBy = 30
        
        rospy.loginfo('Rotating the gripper by ' + str(rotateBy) + ' degrees')
        duration = .5
        deltaPose = tfx.pose([0,0,0], tfx.tb_Angles(rotateBy,0,0))
        self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration))
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
            
        duration = 6
        
        deltaPose = Util.deltaPose(self.gripperPose, self.objectPose, self.transFrame, self.rotFrame)
        deltaPose.position.z += .03
        
        # TEMP
        self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
        #self.gripperControl.goToGripperPoseDelta(self.gripperPose.pose, deltaPose, duration=duration)    
        rospy.sleep(duration)

        return successMethod

    def servoToObject(self, failMethod=None, successMethod=None):
        """
        In close-loop, servos to the object. Then, closes the gripper.
        """
        failMethod = failMethod or (lambda: self.moveVertical(self.moveToReceptacle, self.findObject(self.moveToReceptacle, self.findGripper(self.moveToReceptacle, self.servoToObject))))
        successMethod = successMethod or self.moveVertical

        rospy.loginfo('Servoing to the object point')
        # servo to the object point
            
        transBound = .008
        rotBound = float("inf")

        maxMovement = .01
        deltaPose = uncappedDeltaPose = tfx.pose([0,0,0])

        # if can't find gripper at start, go back to receptacle
        if self.imageDetector.hasFoundGripper(self.gripperName):
            self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
        else:
            return self.moveToReceptacle
            
        self.timeout.start()
        while not Util.withinBounds(self.gripperPose, self.objectPose, transBound, rotBound, self.transFrame, self.rotFrame):
                
            if self.gripperControl.isPaused():
                rospy.loginfo('is paused!')
                    
            if self.imageDetector.hasFoundNewGripper(self.gripperName):
                rospy.loginfo('has found new gripper!')

            if self.gripperControl.isPaused():
                rospy.sleep(1)
                rospy.loginfo('press enter to move')
                raw_input()
                if self.imageDetector.hasFoundNewGripper(self.gripperName):
                    rospy.loginfo('paused and found new gripper')
                    self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                    deltaPose = uncappedDeltaPose = tfx.pose(Util.deltaPose(self.gripperPose, self.objectPose, Constants.Frames.Link0, self.toolframe))
                    deltaPose.position = deltaPose.position.capped(maxMovement)
                    #code.interact(local=locals())
                    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, ignoreOrientation=True)
                else:
                    rospy.loginfo('paused but did NOT find new gripper')
                    deltaPose.position = deltaPose.position - uncappedDeltaPose.position
                    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, ignoreOrientation=True)
                    break
                
                
                print('deltaPose')
                print(deltaPose)
                    
                
                    
            if self.timeout.hasTimedOut() or rospy.is_shutdown():
                rospy.loginfo('Timed out')
                self.objectHeightOffset = self.objectHeightOffset - .001 if self.objectHeightOffset > 0 else 0
                return failMethod
                
            rospy.sleep(.1)

        rospy.loginfo('Closing the gripper')
        # close gripper (consider not all the way)
        duration = 5
        self.gripperControl.closeGripper(duration=duration)
        rospy.sleep(duration)
            
        return successMethod

    def moveVertical(self, failMethod=None, successMethod=None):
        """
        Move vertical in open-loop
        """
        failMethod = failMethod or self.checkPickup
        successMethod = successMethod or self.checkPickup

        rospy.loginfo('Moving vertical with the object')
        # move vertical with the object
        deltaPose = tfx.pose([0,0,.05]).msg.Pose()
        duration = 5

        self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
        rospy.sleep(duration)
        
        return successMethod
    
    def checkPickup(self, failMethod=None, successMethod=None):
        """
        Checks if the grasper picked up a red foam piece
        """
        failMethod = failMethod or self.servoToObject
        successMethod = successMethod or self.moveToReceptacle

        rospy.loginfo('Check if red foam piece successfully picked up')

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
                duration = 5
                self.gripperControl.openGripper(duration=duration)
                rospy.sleep(duration)

                return failMethod
        except:
            return successMethod
            
            
    
    def moveToReceptacle(self, failMethod=None, successMethod=None):
        """
        Move to the receptacle in open-loop
        Then, open the gripper
        """
        failMethod = failMethod or self.moveToReceptacle
        successMethod = successMethod or self.findObject

        rospy.loginfo('Moving to receptacle')
        # move to receptacle with object
        duration = 10

        #if self.imageDetector.hasFoundGripper(self.gripperName):
        #    self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
        #    deltaPose = Util.deltaPose(self.gripperPose, self.receptaclePose, Constants.Frames.Link0, self.toolframe)

        #    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
        #else:
        
        self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), self.receptaclePose.pose, duration=duration)

        rospy.sleep(duration+2)


        rospy.loginfo('Opening the gripper')
        # open gripper (consider not all the way)
        duration = 5
        self.gripperControl.openGripper(duration=duration)
        rospy.sleep(duration)
        
        return successMethod
    
    def run(self):
        """
        Loops through the pipeline

        The pipeline consists of the above methods.
        Each method has a fail and success return method.
        """
        delay = 1
        currentStage = self.findReceptacle
        
        self.gripperControl.start()

        while not rospy.is_shutdown():
            rospy.loginfo('Press enter')
            raw_input()
            currentStage = currentStage()
            rospy.sleep(delay)
            

    def runOLD(self):
        """
        Loops through the pipeline
        """
        self.gripperControl.start()

        while not rospy.is_shutdown():
            
            # delay between parts of the pipeline
            delay = 2

            # can change rate
            rospy.sleep(delay)

           
            # bounds, can change for each particular
            # pipeline section. keep loose for testing

            # translation bound in meters
            transBound = float("inf")
            # rotation bound in radians
            rotBound = float("inf")

            
            

            rospy.loginfo('Searching for the receptacle')
            if not self.imageDetector.hasFoundReceptacle():
                continue
            self.receptaclePose = self.imageDetector.getReceptaclePose()



            rospy.loginfo('Searching for object point')
            # find object point and pose
            if not self.imageDetector.hasFoundObject():
                continue
            # get object w.r.t. toolframe
            self.objectPose = self.imageDetector.getObjectPose(self.toolframe)
            


            rospy.loginfo('Searching for ' + self.gripperName)
            # find gripper point
            if self.imageDetector.hasFoundGripper(self.gripperName):
                self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
            else:
                continue
            
            
            rospy.sleep(delay)
            rospy.loginfo('Moving near the object point')
            rospy.loginfo('Press enter')
            raw_input()
            # go to near the object point
            
            duration = 6
        
            deltaPose = Util.deltaPose(self.gripperPose, self.objectPose, self.transFrame, self.rotFrame)
            deltaPose.position.z += .03
        

            # TEMP
            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
            #self.gripperControl.goToGripperPoseDelta(self.gripperPose.pose, deltaPose, duration=duration)    
            rospy.sleep(duration)
  

            rospy.loginfo('Opening the gripper')
            rospy.loginfo('Press enter')
            raw_input()
            # open gripper
            duration=5
            self.gripperControl.openGripper(duration=duration)
            rospy.sleep(duration)



            rospy.sleep(delay)
            rospy.loginfo('Servoing to the object point')
            rospy.loginfo('Press enter')
            raw_input()
            # servo to the object point
            
            transBound = .008
            rotBound = float("inf")
            
            
            success = True
            self.timeout.start()
            while not Util.withinBounds(self.gripperPose, self.objectPose, transBound, rotBound, self.transFrame, self.rotFrame):
                
                if self.gripperControl.isPaused():
                    rospy.loginfo('is paused!')
                    
                if self.imageDetector.hasFoundNewGripper(self.gripperName):
                    rospy.loginfo('has found new gripper!')

                if self.gripperControl.isPaused() and self.imageDetector.hasFoundNewGripper(self.gripperName):
                    rospy.loginfo('paused and found new gripper')
                    #rospy.loginfo('press enter to move')
                    #raw_input()
                    rospy.sleep(1)
                    self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                    deltaPose = tfx.pose(Util.deltaPose(self.gripperPose, self.objectPose, Constants.Frames.Link0, self.toolframe))
                    deltaPose.position = deltaPose.position*.9
                    print('deltaPose!!!!!')
                    print(deltaPose)
                    #code.interact(local=locals())
                    deltaPose = deltaPose.msg.Pose()
                    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, ignoreOrientation=False)
                    
                if self.timeout.hasTimedOut() or rospy.is_shutdown():
                    rospy.loginfo('Timed Out')
                    success = False
                    break
                
                rospy.sleep(.1)

            if not success:
                continue


            rospy.sleep(delay)
            rospy.loginfo('Closing the gripper')
            rospy.loginfo('Press enter')
            raw_input()
            # close gripper (consider not all the way)
            duration = 5
            self.gripperControl.closeGripper(duration=duration)
            rospy.sleep(duration)

            
            rospy.sleep(delay)
            rospy.loginfo('Moving vertical with the object')
            rospy.loginfo('Press enter')
            raw_input()
            # move vertical with the object
            deltaPose = tfx.pose([0,0,.05]).msg.Pose()
            duration = 5

            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
            rospy.sleep(duration)



            rospy.sleep(delay)
            rospy.loginfo('Moving to receptacle')
            rospy.loginfo('Press enter')
            raw_input()
            # move to receptacle with object
            duration = 10

            self.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
            deltaPose = Util.deltaPose(self.gripperPose, self.receptaclePose, Constants.Frames.Link0, self.toolframe)

            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration, ignoreOrientation=True)
            #self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), self.receptaclePose.pose, duration=duration, ignoreOrientation=True)
            rospy.sleep(duration)


            
            rospy.sleep(delay)
            rospy.loginfo('Opening the gripper')
            rospy.loginfo('Press enter')
            raw_input()
            # close gripper (consider not all the way)
            duration = 5
            self.gripperControl.openGripper(duration=duration)
            rospy.sleep(duration)



            # TEMP do only one loop
            #rospy.loginfo('Press enter to exit')
            #raw_input()
            #return

                        
            # for debugging purposes
            self.imageDetector.removeObjectPoint()

        self.gripperControl.stop()




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

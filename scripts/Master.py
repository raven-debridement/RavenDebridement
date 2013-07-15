#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy

import tf
import tf.transformations as tft
import tfx

from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion

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
        
    def run(self):
        """
        Loops through the pipeline
        """
        self.gripperControl.start()

        while not rospy.is_shutdown():
            
            # delay between parts of the pipeline
            delay = 2

            # can change rate
            rospy.sleep(delay)

            # timeout class with 15 second timeout, can change
            timeout = Util.TimeoutClass(60)

            # bounds, can change for each particular
            # pipeline section. keep loose for testing

            # translation bound in meters
            transBound = float("inf")
            # rotation bound in radians
            rotBound = float("inf")

            
            
            rospy.loginfo('Opening the gripper')
            # open gripper
            duration=5
            self.gripperControl.openGripper(duration=duration)
            rospy.sleep(duration)
            


            rospy.loginfo('Searching for the receptacle')
            if not self.imageDetector.hasFoundReceptacle():
                continue
            receptaclePose = self.imageDetector.getReceptaclePose()



            rospy.loginfo('Searching for object point')
            # find object point and pose
            if not self.imageDetector.hasFoundObject():
                continue
            objectPose = self.imageDetector.getObjectPose()
            objectPoint = Util.poseStampedToPointStamped(objectPose)
            


            rospy.loginfo('Searching for ' + self.gripperName)
            # find gripper point
            if self.imageDetector.hasFoundGripper(self.gripperName):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                gripperPoint = Util.poseStampedToPointStamped(gripperPose)
            else:
                continue
            
            
            rospy.sleep(delay)
            rospy.loginfo('Moving near the object point')
            rospy.loginfo('Press enter')
            #raw_input()
            # go to near the object point
            
            duration = 6
        
            #self.listener.waitForTransform('/stereo_33','/stereo_53',rospy.Time.now(), rospy.Duration(5))
            #gripperPose = self.listener.transformPose('/stereo_53',gripperPose)
            #objectPose = self.listener.transformPose('/stereo_53',objectPose)
            nearObjectPose = tfx.pose(objectPose).msg.PoseStamped()
            nearObjectPose.pose.position.z += .03
            deltaPose = Util.deltaPose(gripperPose.pose, nearObjectPose.pose)
            #deltaPose.orientation = Util.reverseQuaternion(deltaPose.orientation)

        
            #deltaPose = Util.deltaPose(objectPose.pose, gripperPose.pose)
            #deltaPose.position.z += .03
            #deltaPose = tfx.pose(deltaPose.position,[0,0,0,1]).msg.Pose()
            #deltaPose.position = Point()

            #code.interact(local=locals())
            #return
            
            
            # TEMP
            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
            #self.gripperControl.goToGripperPoseDelta(gripperPose.pose, deltaPose)
            
            rospy.sleep(duration)
            
  


            rospy.sleep(delay)
            rospy.loginfo('Servoing to the object point')
            rospy.loginfo('Press enter')
            #raw_input()
            # servo to the object point
            
            transBound = .007
            rotBound = float("inf")

            """
            gripperPose = self.imageDetector.getGripperPose(self.gripperName)
            deltaPose = Util.deltaPose(gripperPose.pose, objectPose.pose)
            
            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose)
            
            
            success = True
            while not Util.withinBounds(gripperPose, objectPose, transBound, rotBound):

                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                                
                rospy.sleep(.5)

                if rospy.is_shutdown():
                    return

            if not success:
                continue
            """
            
            
            success = True
            timeout.start()
            while not Util.withinBounds(gripperPose, objectPose, transBound, rotBound):
                rospy.loginfo('press enter to pause')
                #raw_input()
                self.gripperControl.pause()
                rospy.sleep(.5)
                rospy.loginfo('press enter to move')
                #raw_input()
                if self.imageDetector.hasFoundNewGripper(self.gripperName):
                    gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                    deltaPose = tfx.pose(Util.deltaPose(gripperPose.pose, objectPose.pose))
                    #deltaPose.capped(deltaPose.position.norm_2*.5)
                    deltaPose.position = deltaPose.position*1
                    deltaPose = deltaPose.msg.Pose()
                    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose)

                
                if timeout.hasTimedOut() or rospy.is_shutdown():
                    rospy.loginfo('Timed Out')
                    success = False
                    break
                
                rospy.sleep(6)

            if not success:
                continue


            rospy.sleep(delay)
            rospy.loginfo('Closing the gripper')
            rospy.loginfo('Press enter')
            #raw_input()
            # close gripper (consider not all the way)
            self.gripperControl.closeGripper()
            rospy.sleep(5)

            
            rospy.sleep(delay)
            rospy.loginfo('Moving vertical with the object')
            rospy.loginfo('Press enter')
            #raw_input()
            # move vertical with the object
            deltaPose = tfx.pose([0,0,.05]).msg.Pose()
            duration = 5

            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
            rospy.sleep(duration)



            rospy.sleep(delay)
            rospy.loginfo('Moving to receptacle')
            rospy.loginfo('Press enter')
            #raw_input()
            # move to receptacle with object
            duration = 5

            self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), receptaclePose.pose, duration=duration)
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
    #code.interact(local=locals())
    #return
    imageDetector = ARImageDetectionClass()
    master = MasterClass(Constants.Arm.Left, imageDetector)
    master.run()

if __name__ == '__main__':
    mainloop()

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
            timeout = Util.TimeoutClass(999999999)

            # bounds, can change for each particular
            # pipeline section. keep loose for testing

            # translation bound in meters
            transBound = float("inf")
            # rotation bound in radians
            rotBound = float("inf")

            # translation frame
            transFrame = Constants.Frames.Link0
            # rotation frame
            rotFrame = self.toolframe
            

            rospy.loginfo('Searching for the receptacle')
            if not self.imageDetector.hasFoundReceptacle():
                continue
            receptaclePose = self.imageDetector.getReceptaclePose()



            rospy.loginfo('Searching for object point')
            # find object point and pose
            if not self.imageDetector.hasFoundObject():
                continue
            # get object w.r.t. toolframe
            objectPose = self.imageDetector.getObjectPose(self.toolframe)
            objectPoint = tfx.pose(objectPose).position.msg.PointStamped()
            


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
            raw_input()
            # go to near the object point
            
            duration = 6
        
            nearObjectPose = tfx.pose(objectPose).msg.PoseStamped()
            deltaPose = Util.deltaPose(gripperPose, nearObjectPose, transFrame, rotFrame)
            deltaPose.position.z += .03
        
            #deltaPose = Util.deltaPose(objectPose.pose, gripperPose.pose)
            #deltaPose.position.z += .03
            #deltaPose = tfx.pose(deltaPose.position,[0,0,0,1]).msg.Pose()
            #deltaPose.position = Point()

            
            #desAngle = tfx.pose(objectPose).orientation.msg.Quaternion()
            #toolGripperPose = tfx.pose(gripperPose,frame=self.toolframe)
            #tf_toolframe_to_objframe = tfx.lookupTransform(objectPose.header.frame_id, self.toolframe, wait=10)
            #toolGripperPose = tf_toolframe_to_objframe * toolGripperPose
            #currAngle = tfx.pose(toolGripperPose).orientation.msg.Quaternion()
            #between = Util.angleBetweenQuaternions(desAngle,currAngle)

            #g = self.gripperControl.getGripperPose(frame=Constants.Frames.Link0)
            #o = tfx.pose(g.position,objectPose.pose.orientation).msg.PoseStamped()
            #self.gripperControl.goToGripperPose(g, o, duration=duration)
            #raw_input()
            #return
            
            gp = tfx.pose(gripperPose)#tfx.pose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0))
            #tf_gpframe_to_toolframe = tfx.lookupTransform(self.toolframe, gp.frame, wait=10)
            #gp = tf_gpframe_to_toolframe * gp
            tf_gpframe_to_link0 = tfx.lookupTransform(Constants.Frames.Link0, gp.frame,wait=10)
            gp = tf_gpframe_to_link0 * gp


            obj = tfx.pose(objectPose)
            #tf_objframe_to_toolframe = tfx.lookupTransform(self.toolframe, obj.frame, wait=10)
            #obj = tf_objframe_to_toolframe * obj

            print('normal')
            print(tfx.tb_angles(obj.orientation))
            print('gripper')
            print(tfx.tb_angles(gp.orientation))
            

            #deltaQuat = tft.quaternion_multiply(tfx.tb_angles(obj.orientation).quaternion, tft.quaternion_inverse(tfx.tb_angles(gp.orientation).quaternion))
            deltaQuat = tft.quaternion_multiply(tft.quaternion_inverse(tfx.tb_angles(gp.orientation).quaternion), tfx.tb_angles(obj.orientation).quaternion)
            #deltaPose = tfx.pose(obj.position-gp.position, deltaQuat).msg.Pose()
            
            deltaRot = tfx.tb_angles(deltaPose.orientation)
            print('between')
            print(deltaRot)

            # tool_l to link

            print(deltaPose)
            code.interact(local=locals())
            #return

            # TEMP
            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
            #self.gripperControl.goToGripperPoseDelta(gp.msg.Pose(), deltaPose, duration=duration)
            #self.gripperControl.goToGripperPoseDelta(gripperPose.pose, deltaPose, duration=duration)
            
            rospy.sleep(duration)
            
            raw_input()
            return
  

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
            
            transBound = .007
            rotBound = float("inf")
            
            
            success = True
            timeout.start()
            while not Util.withinBounds(gripperPose, objectPose, transBound, rotBound, transFrame, rotFrame):
                
                if self.gripperControl.isPaused():
                    rospy.loginfo('is paused!')
                    
                if self.imageDetector.hasFoundNewGripper(self.gripperName):
                    rospy.loginfo('has found new gripper!')

                if self.gripperControl.isPaused() and self.imageDetector.hasFoundNewGripper(self.gripperName):
                    rospy.loginfo('paused and found new gripper')
                    rospy.loginfo('press enter to move')
                    raw_input()
                    tmpGripperPose = self.imageDetector.getGripperPose(self.gripperName)
                    #if not Util.withinBounds(gripperPose, tmpGripperPose, .0001, float("inf")):
                    gripperPose = tmpGripperPose
                    deltaPose = tfx.pose(Util.deltaPose(gripperPose, objectPose, Constants.Frames.Link0, self.toolframe))
                    deltaPose.position = deltaPose.position*.9
                    print('deltaPose!!!!!')
                    print(deltaPose)
                    code.interact(local=locals())
                    deltaPose = deltaPose.msg.Pose()
                    self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, ignoreOrientation=True)
                    #else:
                    #    rospy.loginfo('but the same!!!!!!!!!')
                
                if timeout.hasTimedOut() or rospy.is_shutdown():
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
            duration = 5

            gripperPose = self.imageDetector.getGripperPose(self.gripperName)
            deltaPose = Util.deltaPose(gripperPose, receptaclePose, Constants.Frames.Link0, self.toolframe)

            self.gripperControl.goToGripperPoseDelta(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), deltaPose, duration=duration)
            #self.gripperControl.goToGripperPose(self.gripperControl.getGripperPose(frame=Constants.Frames.Link0), receptaclePose.pose, duration=duration, ignoreOrientation=True)
            rospy.sleep(duration)


            
            rospy.sleep(delay)
            rospy.loginfo('Opening the gripper')
            rospy.loginfo('Press enter')
            raw_input()
            # close gripper (consider not all the way)
            duration = 5
            self.gripperControl.openGripper(duration=duration)
            rospy.sleep(duration)


            rospy.sleep(delay)
            rospy.loginfo('Closing the gripper')
            rospy.loginfo('Press enter')
            raw_input()
            # close gripper (consider not all the way)
            duration = 5
            self.gripperControl.closeGripper(duration=duration)
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

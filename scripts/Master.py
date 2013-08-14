#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math

import tf
import tf.transformations as tft
import tfx

import smach
import smach_ros
smach.set_shutdown_check(rospy.is_shutdown)

from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion
from raven_pose_estimator.srv import ThreshRed

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.RavenCommand.RavenPlanner import RavenPlanner
from RavenDebridement.RavenCommand.RavenBSP import RavenBSP
from RavenDebridement.ImageProcessing.ARImageDetection import ARImageDetector

import code

PAUSE_BETWEEN_STATES = True

def pause_func(myclass):
    rospy.loginfo('In {0} method. Press enter to continue'.format(myclass))
    raw_input()

class FindReceptacle(smach.State):
    def __init__(self, imageDetector):
        smach.State.__init__(self, outcomes = ['success','failure'], output_keys = ['receptaclePose'])
        self.imageDetector = imageDetector
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Searching for the receptacle')
        if not self.imageDetector.hasFoundReceptacle():
            rospy.loginfo('Did not find receptacle')
            return 'failure'
        userdata.receptaclePose = self.imageDetector.getReceptaclePose()

        rospy.loginfo('Found receptacle')
        return 'success'

class FindHome(smach.State):
    def __init__(self, imageDetector):
        smach.State.__init__(self, outcomes = ['success','failure'], output_keys = ['homePose'])
        self.imageDetector = imageDetector
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Searching for the home position')
        if not self.imageDetector.hasFoundHome():
            rospy.loginfo('Did not find home position')
            return 'failure'
        userdata.homePose = self.imageDetector.getHomePose()

        rospy.loginfo('Found home position')
        return 'success'

class FindObject(smach.State):
    def __init__(self, imageDetector, toolframe, obj_pub):
        smach.State.__init__(self, outcomes = ['success','failure'], input_keys = ['objectHeightOffset'], output_keys = ['objectPose','rotateBy'])
        self.imageDetector = imageDetector
        self.toolframe = toolframe
        self.obj_pub = obj_pub
    
    def publishObjectPose(self, pose):
        self.obj_pub.publish(pose)
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        # reset rotateBy
        userdata.rotateBy = -30

        rospy.loginfo('Searching for object point')
        # find object point and pose
        if not self.imageDetector.hasFoundObject():
            rospy.loginfo('Did not find object')
            return 'failure'
        # get object w.r.t. toolframe
        objectPose = self.imageDetector.getObjectPose(self.toolframe)
        objectPose.pose.position.z += userdata.objectHeightOffset
        self.publishObjectPose(objectPose)

        userdata.objectPose = objectPose

        rospy.loginfo('Found object')
        return 'success'

class FindGripper(smach.State):
    def __init__(self, imageDetector, gripperName):
        smach.State.__init__(self, outcomes = ['success','failure'], output_keys = ['gripperPose'])
        self.imageDetector = imageDetector
        self.gripperName = gripperName
        
        self.findGripperTimeout = Util.Timeout(1)
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Searching for ' + self.gripperName)
        # find gripper point
        self.imageDetector.ignoreOldGripper(self.gripperName)


        self.findGripperTimeout.start()
        while (not self.imageDetector.hasFoundGripper(self.gripperName)) or (not self.imageDetector.hasFoundNewGripper(self.gripperName)):
            if self.findGripperTimeout.hasTimedOut():
                rospy.loginfo('Did not find gripper')
                return 'failure'
            rospy.sleep(.05)

        userdata.gripperPose = self.imageDetector.getGripperPose(self.gripperName)

        rospy.loginfo('Found gripper')
        return 'success'

class RotateGripper(smach.State):
    def __init__(self, ravenArm):
        smach.State.__init__(self, outcomes = ['success','failure'], io_keys = ['rotateBy'])
        self.ravenArm = ravenArm
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Rotating the gripper by ' + str(userdata.rotateBy) + ' degrees')
        deltaPose = tfx.pose([0,0,.001], tfx.tb_angles(0,0,userdata.rotateBy))
        self.ravenArm.goToGripperPoseDelta(deltaPose, duration=2)

        userdata.rotateBy = -math.copysign(abs(userdata.rotateBy)+5, userdata.rotateBy)

        return 'success'

class PlanTrajToObject(smach.State):
    def __init__(self, ravenArm, ravenPlanner, stepsPerMeter, transFrame, rotFrame, gripperOpenCloseDuration):
        smach.State.__init__(self, outcomes = ['reachedObject', 'notReachedObject','failure'],
                             input_keys = ['gripperPose','objectPose'],
                             output_keys = ['jointTraj','vertAmount'],)
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        self.gripperOpenCloseDuration = gripperOpenCloseDuration

    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Planning trajectory from gripper to object')

        transBound = .006
        rotBound = float("inf")
        if Util.withinBounds(userdata.gripperPose, userdata.objectPose, transBound, rotBound, self.transFrame, self.rotFrame):
            rospy.loginfo('Closing the gripper')
            self.ravenArm.closeGripper(duration=self.gripperOpenCloseDuration)
            userdata.vertAmount = .04
            return 'reachedObject'

        deltaPose = tfx.pose(Util.deltaPose(userdata.gripperPose, userdata.objectPose, self.transFrame, self.rotFrame))

        endPose = Util.endPose(self.ravenArm.getGripperPose(), deltaPose)
        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        jointTraj = self.ravenPlanner.getTrajectoryFromPose(endPose, n_steps=n_steps)
        
        rospy.loginfo('deltaPose')
        rospy.loginfo(deltaPose)
        rospy.loginfo('n_steps')
        rospy.loginfo(n_steps)

        if jointTraj == None:
            return 'failure'

        userdata.jointTraj = jointTraj
        return 'notReachedObject'

class MoveTowardsObject(smach.State):
    def __init__(self, ravenArm, stepsPerMeter, maxServoDistance):
        smach.State.__init__(self, outcomes = ['success'], input_keys = ['jointTraj'])
        self.ravenArm = ravenArm
        self.stepsPerMeter = stepsPerMeter
        self.maxServoDistance = maxServoDistance

    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
            pause_func(self)

        rospy.loginfo('Moving towards the object')
        jointTraj = userdata.jointTraj

        # limit distance moved
        endTrajStep = min(int(self.stepsPerMeter*self.maxServoDistance)+1, len(jointTraj))
        jointTraj = jointTraj[:endTrajStep]
        
        rospy.loginfo('Total steps')
        rospy.loginfo(len(jointTraj))
        rospy.loginfo('endTrajStep')
        rospy.loginfo(endTrajStep)

        self.ravenArm.executeJointTrajectory(jointTraj,block=True)

        return 'success'

class MoveNearObject(smach.State):
    """
    Goes near the object
    """
    def __init__(self, ravenArm, ravenPlanner, transFrame, rotFrame, openLoopSpeed):
        smach.State.__init__(self, outcomes = ['success','failure'], input_keys=['gripperPose','objectPose'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        self.openLoopSpeed = openLoopSpeed
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Moving near the object point')
        # go to near the object point
        
        deltaPose = Util.deltaPose(userdata.gripperPose, userdata.objectPose, self.transFrame, self.rotFrame)
        deltaPose.position.z += .03

        endPose = Util.endPose(self.ravenArm.getGripperPose(), deltaPose)
        endJointTraj = self.ravenPlanner.getTrajectoryFromPose(endPose)

        if endJointTraj != None:
            self.ravenArm.executeJointTrajectory(endJointTraj)
        
        return 'success'

class ServoToObject(smach.State):
    """
    In closed-loop, servos to the object. Then, closes the gripper.
    """
    def __init__(self, imageDetector, ravenArm, ravenPlanner, gripperName, transFrame, rotFrame, toolframe, gripperOpenCloseDuration, listener, des_pose_pub):
        smach.State.__init__(self, outcomes = ['success','gripperNotFound','timedOut'], 
                             input_keys=['objectPose'], 
                             output_keys=['vertAmount'],
                             io_keys=['gripperPose', 'objectHeightOffset'])
        self.imageDetector = imageDetector
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.gripperName = gripperName
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        self.toolframe = toolframe
        self.gripperOpenCloseDuration = gripperOpenCloseDuration
        self.listener = listener
        self.des_pose_pub = des_pose_pub
        
        self.timeout = Util.Timeout(999999999)

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
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Servoing to the object point')
        # servo to the object point
            
        transBound = .008
        rotBound = float("inf")

        maxMovement = .015
        deltaPose = uncappedDeltaPose = tfx.pose([0,0,0])

        # if can't find gripper at start, go back to receptacle
        rospy.sleep(1)
        if self.imageDetector.hasFoundGripper(self.gripperName) and self.imageDetector.hasFoundNewGripper(self.gripperName):
            userdata.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
        else:
            return 'gripperNotFound'
            
        self.timeout.start()
        while not Util.withinBounds(userdata.gripperPose, userdata.objectPose, transBound, rotBound, self.transFrame, self.rotFrame):
                
            if self.ravenArm.isPaused():
                rospy.sleep(1)
                if self.imageDetector.hasFoundNewGripper(self.gripperName):
                    rospy.loginfo('paused and found new gripper')
                    userdata.gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                    deltaPose = uncappedDeltaPose = tfx.pose(Util.deltaPose(userdata.gripperPose, userdata.objectPose, Constants.Frames.Link0, self.toolframe))
                    
                    deltaPose.position = deltaPose.position.capped(maxMovement)
                    #if abs(deltaPose.position.z) > maxMovement:
                    #    deltaPose.position.z = math.copysign(maxMovement, deltaPose.position.z)

                    deltaPose0Link = tfx.pose(Util.deltaPose(userdata.gripperPose, userdata.objectPose, Constants.Frames.Link0, Constants.Frames.Link0))
                    deltaPose0Link.position = deltaPose.position.capped(maxMovement)
                    self.publishDesiredPose(deltaPose0Link, tfx.pose(userdata.gripperPose))

                    rospy.loginfo('delta pose')
                    rospy.loginfo(deltaPose)

                    #code.interact(local=locals())

                    endPose = Util.endPose(self.ravenArm.getGripperPose(), deltaPose)
                    endJointTraj = self.ravenPlanner.getTrajectoryFromPose(endPose)

                    code.interact(local=locals())
                    if endJointTraj != None:
                        self.ravenArm.executeJointTrajectory(endJointTraj)

                    rospy.sleep(1)
                else:
                    rospy.loginfo('paused but did NOT find new gripper')
                    deltaPose.position = uncappedDeltaPose.position - deltaPose.position
                    self.ravenArm.goToGripperPoseDelta(deltaPose, ignoreOrientation=True)
                    break
                    
                    
            if self.timeout.hasTimedOut() or rospy.is_shutdown():
                rospy.loginfo('Timed out')
                userdata.objectHeightOffset = userdata.objectHeightOffset - .0005 if userdata.objectHeightOffset > 0 else 0
                userdata.vertAmount = .04
                return 'timedOut'
                
            rospy.sleep(.1)

        rospy.loginfo('Closing the gripper')
        # close gripper (consider not all the way)
        self.ravenArm.closeGripper(duration=self.gripperOpenCloseDuration)
            
        userdata.vertAmount = .04
        
        return 'success'

class MoveVertical(smach.State):
    """
    Move vertical in open-loop
    """
    def __init__(self, ravenArm, ravenPlanner, openLoopSpeed):
        smach.State.__init__(self, outcomes = ['success','failure'], input_keys=['vertAmount'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.openLoopSpeed = openLoopSpeed
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Moving vertical with the object')
        # move vertical with the object
        deltaPose = tfx.pose([0,0,userdata.vertAmount]).msg.Pose()
        
        endPose = Util.endPose(self.ravenArm.getGripperPose(), deltaPose)
        endJointTraj = self.ravenPlanner.getTrajectoryFromPose(endPose)

        if endJointTraj != None:
            self.ravenArm.executeJointTrajectory(endJointTraj)
        
        return 'success'

class CheckPickup(smach.State):
    """
    Checks if the grasper picked up a red foam piece
    """
    def __init__(self, ravenArm, gripperOpenCloseDuration):
        smach.State.__init__(self, outcomes = ['success','failure'])
        self.ravenArm = ravenArm
        self.gripperOpenCloseDuration = gripperOpenCloseDuration
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

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
                
                return 'failure'
        except:
            rospy.loginfo('Service exception, assuming successful pickup')
            return 'success'

class MoveToReceptacle(smach.State):
    """
    Move to the receptacle in open-loop
    Then, open the gripper
    """
    def __init__(self, ravenArm, ravenPlanner, openLoopSpeed, gripperOpenCloseDuration):
        smach.State.__init__(self, outcomes = ['success','failure'],input_keys=['receptaclePose'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.openLoopSpeed = openLoopSpeed
        self.gripperOpenCloseDuration = gripperOpenCloseDuration
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Moving to receptacle')
        # move to receptacle with object

        currPose = tfx.pose(self.ravenArm.getGripperPose())
        receptaclePose = tfx.pose(userdata.receptaclePose)    
        #ignore orientation
        receptaclePose.orientation = currPose.orientation

        endJointTraj = self.ravenPlanner.getTrajectoryFromPose(receptaclePose)

        if endJointTraj != None:
            self.ravenArm.executeJointTrajectory(endJointTraj)
        

        rospy.loginfo('Opening the gripper')
        # open gripper (consider not all the way)
        self.ravenArm.openGripper(duration=self.gripperOpenCloseDuration)
        return 'success'

class MoveToHome(smach.State):
    """
    Move to the home position in open-loop
    """
    def __init__(self, ravenArm, ravenPlanner, imageDetector, openLoopSpeed):
        smach.State.__init__(self, outcomes=['success','failure'], input_keys=['homePose'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.imageDetector = imageDetector
        self.openLoopSpeed = openLoopSpeed
    
    def execute(self, userdata):
        if PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Moving to home position')
        # move to home position

        currPose = tfx.pose(self.ravenArm.getGripperPose())
        homePose = tfx.pose(userdata.homePose)    
        #ignore orientation
        homePose.orientation = currPose.orientation

        endJointTraj = self.ravenPlanner.getTrajectoryFromPose(homePose)

        if endJointTraj != None:
            self.ravenArm.executeJointTrajectory(endJointTraj)
        
        # so when finding object, find newest one
        self.imageDetector.removeObjectPoint()
        
        return 'success'

class MasterClass(object):
    def __init__(self, armName, imageDetector):
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
        self.ravenArm = RavenArm(self.armName)
        #self.ravenPlanner = RavenPlanner(self.armName)
        self.ravenPlanner = RavenBSP(self.armName)

        # translation frame
        self.transFrame = Constants.Frames.Link0
        # rotation frame
        self.rotFrame = self.toolframe

        # height offset for foam
        self.objectHeightOffset = .004

        # in cm/sec, I think
        self.openLoopSpeed = .01

        self.gripperOpenCloseDuration = 2.5

        # for Trajopt planning, 2 steps/cm
        self.stepsPerMeter = 200

        # move no more than 5cm per servo
        self.maxServoDistance = .05

        # debugging outputs
        self.des_pose_pub = rospy.Publisher('desired_pose', PoseStamped)
        self.obj_pub = rospy.Publisher('object_pose', PoseStamped)
        
        self.sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['objectHeightOffset'])
        
        with self.sm:
            smach.StateMachine.add('findReceptable', FindReceptacle(self.imageDetector), 
                                   transitions = {'success': 'findHome',
                                                  'failure': 'findReceptable'})
            smach.StateMachine.add('findHome', FindHome(self.imageDetector),
                                   transitions = {'success': 'moveToReceptacle',
                                                 'failure': 'findHome'})
            smach.StateMachine.add('findObject', FindObject(self.imageDetector, self.toolframe, self.obj_pub),
                                   transitions = {'success': 'findGripper',
                                                  'failure': 'findObject'})
            smach.StateMachine.add('findGripper', FindGripper(self.imageDetector, self.gripperName),
                                   transitions = {'success': 'planTrajToObject',
                                                  'failure': 'rotateGripper'})
            smach.StateMachine.add('rotateGripper', RotateGripper(self.ravenArm),
                                   transitions = {'success': 'findGripper',
                                                  'failure': 'findGripper'})
            smach.StateMachine.add('planTrajToObject', PlanTrajToObject(self.ravenArm, self.ravenPlanner, self.stepsPerMeter,
                                                                        self.transFrame, self.rotFrame, self.gripperOpenCloseDuration),
                                   transitions = {'reachedObject': 'objectServoSuccessMoveVertical',
                                                  'notReachedObject': 'moveTowardsObject',
                                                  'failure': 'moveToHome'})
            smach.StateMachine.add('moveTowardsObject', MoveTowardsObject(self.ravenArm, self.stepsPerMeter, self.maxServoDistance),
                                   transitions = {'success': 'findGripper'})
            #smach.StateMachine.add('moveNearObject', MoveNearObject(self.ravenArm, self.ravenPlanner, self.transFrame, self.rotFrame, self.openLoopSpeed),
            #                       transitions = {'success': 'servoToObject',
            #                                      'failure': 'moveNearObject'})
            #smach.StateMachine.add('servoToObject', ServoToObject(self.imageDetector, self.ravenArm, self.ravenPlanner, 
            #                                                      self.gripperName, self.transFrame, self.rotFrame, self.toolframe,
            #                                                      self.gripperOpenCloseDuration, self.listener, self.des_pose_pub),
            #                       transitions = {'success': 'objectServoSuccessMoveVertical',
            #                                      'gripperNotFound': 'moveToHome',
            #                                      'timedOut': 'objectServoFailureMoveVertical'})
            smach.StateMachine.add('objectServoSuccessMoveVertical',MoveVertical(self.ravenArm, self.ravenPlanner, self.openLoopSpeed),
                                   transitions = {'success': 'checkPickup',
                                                  'failure': 'checkPickup'})
            smach.StateMachine.add('checkPickup', CheckPickup(self.ravenArm, self.gripperOpenCloseDuration),
                                   transitions = {'success': 'pickupSuccessMoveToHome',
                                                  'failure': 'findObject'})#'pickupFailureFindObject'})
            smach.StateMachine.add('pickupSuccessMoveToHome', MoveToHome(self.ravenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                   transitions = {'success': 'moveToReceptacle',
                                                  'failure': 'moveToReceptacle'})
            #smach.StateMachine.add('pickupFailureFindObject', FindObject(self.imageDetector, self.toolframe, self.obj_pub),
            #                       transitions = {'success': 'pickupFailureFindObjectSuccessFindGripper',
            #                                      'failure': 'moveToHome'})
            #smach.StateMachine.add('pickupFailureFindObjectSuccessFindGripper', FindGripper(self.imageDetector, self.gripperName),
            #                       transitions = {'success': 'servoToObject',
            #                                      'failure': 'moveToHome'})
            #smach.StateMachine.add('objectServoFailureMoveVertical',MoveVertical(self.ravenArm, self.ravenPlanner, self.openLoopSpeed),
            #                       transitions = {'success': 'objectServoFailureMoveVerticalSuccessFindObject',
            #                                      'failure': 'moveToReceptacle'})
            #smach.StateMachine.add('objectServoFailureMoveVerticalSuccessFindObject', 
            #                       FindObject(self.imageDetector, self.toolframe, self.obj_pub),
            #                       transitions = {'success': 'moveToHome',
            #                                      'failure': 'objectServoFailureMoveVerticalSuccessFindObjectFailureFindGripper'})
            #smach.StateMachine.add('objectServoFailureMoveVerticalSuccessFindObjectFailureFindGripper',
            #                       FindGripper(self.imageDetector, self.gripperName),
            #                       transitions = {'success': 'servoToObject',
            #                                      'failure': 'moveToHome'})
            smach.StateMachine.add('moveToReceptacle', MoveToReceptacle(self.ravenArm, self.ravenPlanner, self.openLoopSpeed, self.gripperOpenCloseDuration),
                                   transitions = {'success': 'moveToHome',
                                                  'failure': 'moveToHome'})
            smach.StateMachine.add('moveToHome', MoveToHome(self.ravenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                   transitions = {'success': 'findObject',
                                                  'failure': 'findObject'})
    
    def run(self):
        self.ravenArm.start()
        sis = smach_ros.IntrospectionServer('master_server', self.sm, '/SM_ROOT')
        sis.start()
        userData = smach.UserData()
        userData['objectHeightOffset'] = self.objectHeightOffset
        

        try:
            outcome = self.sm.execute(userData)
        except:
            pass

        self.ravenArm.stop()


def mainloop():
    """
    Gets an instance of the MasterClass
    for the left arm and executes the
    run loop
    """
    rospy.init_node('master_node',anonymous=True)
    imageDetector = ARImageDetector()
    master = MasterClass(Constants.Arm.Left, imageDetector)
    master.run()

def rotateTest():
    rospy.init_node('master_node',anonymous=True)
    imageDetector = ARImageDetector()
    master = MasterClass(Constants.Arm.Left, imageDetector)
    master.ravenArm.start()

    while not rospy.is_shutdown():
        rospy.loginfo('Press enter')
        raw_input()
        master.rotateGripper()

    master.ravenArm.stop()


if __name__ == '__main__':
    mainloop()
    

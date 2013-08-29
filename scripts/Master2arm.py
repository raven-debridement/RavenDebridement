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
from RavenDebridement.RavenCommand.RavenPlanner2 import RavenPlanner
from RavenDebridement.RavenCommand.RavenBSP import RavenBSP
from RavenDebridement.ImageProcessing.ImageDetection import ImageDetector
from RavenDebridement.ImageProcessing.FoamAllocator import FoamAllocator,\
    ArmFoamAllocator

import threading

import IPython

def pause_func(myclass):
    rospy.loginfo('In {0} method. Press enter to continue'.format(myclass))
    raw_input()

class DoNothing(smach.State):
    def __init__(self, ravenArm, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['success','failure'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Doing nothing')

        transBound = .006
        rotBound = float("inf")
        endPose = self.ravenArm.getGripperPose()
        n_steps = 5
        print 'requesting trajectory for otherarm', self.ravenArm.name
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps)
        
        if poseTraj == None:
            return 'failure'

        return 'success'

class MoveUp(smach.State):
    def __init__(self, ravenArm, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['success','failure'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Moving up')

        transBound = .006
        rotBound = float("inf")
        endPose = self.ravenArm.getGripperPose() + [0,0,0.01]
        n_steps = 5
        print 'requesting trajectory for otherarm', self.ravenArm.name
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps)
        
        if poseTraj == None:
            return 'failure'
        
        self.ravenArm.executePoseTrajectory(poseTraj,block=True)

        return 'success'

class MoveDown(smach.State):
    def __init__(self, ravenArm, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['success','failure'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Moving down')

        transBound = .006
        rotBound = float("inf")
        endPose = self.ravenArm.getGripperPose() + [0,0,-0.01]
        n_steps = 5
        print 'requesting trajectory for otherarm', self.ravenArm.name
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps)
        
        if poseTraj == None:
            return 'failure'
        
        self.ravenArm.executePoseTrajectory(poseTraj,block=True)

        return 'success'

class FindReceptacle(smach.State):
    def __init__(self, armName, imageDetector):
        smach.State.__init__(self, outcomes = ['success','failure'], output_keys = ['receptaclePose'])
        self.armName = armName
        self.imageDetector = imageDetector
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Searching for the receptacle')
        if not self.imageDetector.hasFoundReceptacle(self.armName):
            rospy.loginfo('Did not find receptacle')
            return 'failure'
        userdata.receptaclePose = self.imageDetector.getReceptaclePose(self.armName)
        
        rospy.loginfo('Found receptacle')
        return 'success'

class FindHome(smach.State):
    def __init__(self, armName, imageDetector):
        smach.State.__init__(self, outcomes = ['success','failure'], output_keys = ['homePose'])
        self.armName = armName
        self.imageDetector = imageDetector
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Searching for the home position')
        if not self.imageDetector.hasFoundHome(self.armName):
            rospy.loginfo('Did not find home position')
            return 'failure'
        userdata.homePose = self.imageDetector.getHomePose(self.armName)

        rospy.loginfo('Found home position')
        return 'success'

class FindObject(smach.State):
    def __init__(self, foamSegmenter, toolframe, obj_pub):
        smach.State.__init__(self, outcomes = ['success','failure'], input_keys = ['objectHeightOffset'], output_keys = ['rotateBy'], io_keys=['objectPose'])
        self.foamSegmenter = foamSegmenter
        self.toolframe = toolframe
        self.obj_pub = obj_pub
    
    def publishObjectPose(self, pose):
        self.obj_pub.publish(pose)
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        # reset rotateBy
        userdata.rotateBy = -30
        
        new = userdata.objectPose is None

        rospy.loginfo('Searching for object point')
        # find object point and pose
        if not self.foamSegmenter.hasFoam(new=new):
            rospy.sleep(1)
        if not self.foamSegmenter.hasFoam(new=new):
            rospy.loginfo('Did not find object')
            return 'failure'
        # get object w.r.t. toolframe
        objectPose = self.foamSegmenter.allocateFoam(new=new)
        objectPose.position.z += userdata.objectHeightOffset
        self.publishObjectPose(objectPose.msg.PoseStamped())
        
        print objectPose

        userdata.objectPose = objectPose.msg.PoseStamped()

        rospy.loginfo('Found object')
        return 'success'

class FindGripper(smach.State):
    def __init__(self, imageDetector, gripperName):
        smach.State.__init__(self, outcomes = ['success','failure'], output_keys = ['gripperPose'])
        self.imageDetector = imageDetector
        self.gripperName = gripperName
        
        self.findGripperTimeout = Util.Timeout(3)
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)
           
        # due to image processing lag
        rospy.sleep(.5)

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
        smach.State.__init__(self, outcomes = ['success'], io_keys = ['rotateBy'])
        self.ravenArm = ravenArm
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
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
                             output_keys = ['poseTraj','vertAmount'],)
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        self.gripperOpenCloseDuration = gripperOpenCloseDuration
        
        self.objectPosePub = rospy.Publisher('traj_obj_pose',PoseStamped)
        self.endPosePub = rospy.Publisher('traj_end_pose',PoseStamped)

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Planning trajectory from gripper to object')
        
        objectPose = tfx.pose(userdata.objectPose)
        gripperPose = tfx.pose(userdata.gripperPose)
        calcGripperPose = tfx.pose(self.ravenArm.getGripperPose())
        
        #objectPose.orientation = gripperPose.orientation
        
        self.objectPosePub.publish(objectPose.msg.PoseStamped())
        
        transBound = .006
        rotBound = float("inf")
        if Util.withinBounds(gripperPose, objectPose, transBound, rotBound, self.transFrame, self.rotFrame):
            rospy.loginfo('Closing the gripper')
            self.ravenArm.closeGripper(duration=self.gripperOpenCloseDuration)
            userdata.vertAmount = .04
            return 'reachedObject'

        deltaPose = tfx.pose(Util.deltaPose(gripperPose, objectPose, self.transFrame, self.rotFrame))
        
        endPoseForPub = Util.endPose(gripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.endPosePub.publish(endPoseForPub.msg.PoseStamped())

        endPose = Util.endPose(calcGripperPose, deltaPose, frame=Constants.Frames.Link0)
        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps)
        
        print('objectPose')
        print(objectPose)
        print('gripperPose')
        print(gripperPose)
        print('calcGripperPose')
        print(calcGripperPose)
        print('endPoseForPub')
        print(endPoseForPub)
        print('endPose')
        print(endPose)
        print('deltaPose')
        print(deltaPose)
        
        # gripperPose + deltaPose = objectPose

        if poseTraj == None:
            return 'failure'

        userdata.poseTraj = poseTraj
        return 'notReachedObject'

class MoveTowardsObject(smach.State):
    def __init__(self, ravenArm, stepsPerMeter, maxServoDistance):
        smach.State.__init__(self, outcomes = ['success'], input_keys = ['poseTraj'])
        self.ravenArm = ravenArm
        self.stepsPerMeter = stepsPerMeter
        self.maxServoDistance = maxServoDistance

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)

        rospy.loginfo('Moving towards the object')
        poseTraj = userdata.poseTraj

        # limit distance moved
        endTrajStep = min(int(self.stepsPerMeter*self.maxServoDistance)+1, len(poseTraj))
        poseTraj = poseTraj[:endTrajStep]
        
        rospy.loginfo('Total steps')
        rospy.loginfo(len(poseTraj))
        rospy.loginfo('endTrajStep')
        rospy.loginfo(endTrajStep)

        self.ravenArm.executePoseTrajectory(poseTraj,block=True)

        return 'success'


class MoveVertical(smach.State):
    """
    Move vertical in open-loop
    """
    def __init__(self, ravenArm, ravenPlanner, openLoopSpeed):
        smach.State.__init__(self, outcomes = ['success'], input_keys=['vertAmount'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.openLoopSpeed = openLoopSpeed
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Moving vertical with the object')
        # move vertical with the object
        deltaPose = tfx.pose([0,0,userdata.vertAmount]).msg.Pose()
        
        endPose = Util.endPose(self.ravenArm.getGripperPose(), deltaPose)
        endPoseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose)

        if endPoseTraj != None:
            self.ravenArm.executePoseTrajectory(endPoseTraj)
        
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
        if MasterClass.PAUSE_BETWEEN_STATES:
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
        smach.State.__init__(self, outcomes = ['success'],input_keys=['receptaclePose'], output_keys=['objectPose'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.openLoopSpeed = openLoopSpeed
        self.gripperOpenCloseDuration = gripperOpenCloseDuration
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Moving to receptacle')
        # move to receptacle with object

        currPose = tfx.pose(self.ravenArm.getGripperPose())
        receptaclePose = tfx.pose(userdata.receptaclePose)    
        #ignore orientation
        receptaclePose.orientation = currPose.orientation

        print 'getting trajectory'
        endPoseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, receptaclePose)
        print 'got receptacle trajectory'

        if endPoseTraj != None:
            print 'executing receptacle trajectory'
            self.ravenArm.executePoseTrajectory(endPoseTraj)
        

        rospy.loginfo('Opening the gripper')
        # open gripper (consider not all the way)
        self.ravenArm.openGripper(duration=self.gripperOpenCloseDuration)
        #self.ravenArm.setGripper(0.5)
        
        userdata.objectPose = None
        return 'success'

class MoveToHome(smach.State):
    """
    Move to the home position in open-loop
    """
    def __init__(self, ravenArm, ravenPlanner, imageDetector, openLoopSpeed):
        smach.State.__init__(self, outcomes=['success'], input_keys=['homePose'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.imageDetector = imageDetector
        self.openLoopSpeed = openLoopSpeed
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Moving to home position')
        # move to home position

        currPose = tfx.pose(self.ravenArm.getGripperPose())
        homePose = tfx.pose(userdata.homePose)    
        #ignore orientation
        homePose.orientation = currPose.orientation

        endPoseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, homePose)

        if endPoseTraj != None:
            self.ravenArm.executePoseTrajectory(endPoseTraj)
        
        # so when finding object, find newest one
        self.imageDetector.removeObjectPoint()
        
        return 'success'

class MasterClass(object):
    PAUSE_BETWEEN_STATES = True
    LEFT_ARM_THREAD = None
    RIGHT_ARM_THREAD = None
    
    def __init__(self, armName, ravenArm, ravenPlanner, imageDetector, foamAllocator):
        self.armName = armName
        
        if armName == Constants.Arm.Left:
            self.gripperName = Constants.Arm.Left
            self.toolframe = Constants.Frames.LeftTool
            #self.calibrateGripperState = ImageDetectionClass.State.CalibrateLeft
            self.otherArmName = Constants.Arm.Right
            self.otherGripperName = Constants.Arm.Right
            self.otherToolframe = Constants.Frames.RightTool
        elif armName == Constants.Arm.Right:
            self.gripperName = Constants.Arm.Right
            self.toolframe = Constants.Frames.RightTool
            #self.calibrateGripperState = ImageDetectionClass.State.CalibrateRight
            self.otherArmName = Constants.Arm.Left
            self.otherGripperName = Constants.Arm.Left
            self.otherToolframe = Constants.Frames.LeftTool

        self.listener = tf.TransformListener.instance()

        # initialize the three main control mechanisms
        # image detection, gripper control, and arm control
        self.imageDetector = imageDetector
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.foamAllocator = ArmFoamAllocator(self.armName,foamAllocator)
        
        # translation frame
        self.transFrame = Constants.Frames.Link0
        self.otherTransFrame = Constants.Frames.Link0
        # rotation frame
        self.rotFrame = self.toolframe
        self.otherRotFrame = self.otherToolframe
        self.otherFoamAllocator = ArmFoamAllocator(self.otherArmName,foamAllocator)

        # height offset for foam
        self.objectHeightOffset = .004

        # in cm/sec, I think
        self.openLoopSpeed = .01

        self.gripperOpenCloseDuration = 5.5

        # for Trajopt planning, 2 steps/cm
        self.stepsPerMeter = 200

        # move no more than 5cm per servo
        self.maxServoDistance = .05

        # debugging outputs
        self.des_pose_pub = rospy.Publisher('desired_pose', PoseStamped)
        self.obj_pub = rospy.Publisher('object_pose', PoseStamped)
        
        self.sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['objectHeightOffset'])
        
        with self.sm:
            smach.StateMachine.add('findReceptacle', FindReceptacle(self.armName, self.imageDetector), 
                                   transitions = {'success': 'findHome',
                                                  'failure': 'findReceptacle'})
            smach.StateMachine.add('findHome', FindHome(self.armName, self.imageDetector),
                                   transitions = {'success': 'moveToReceptacle',
                                                 'failure': 'findHome'})
            smach.StateMachine.add('findObject', FindObject(self.foamAllocator, self.toolframe, self.obj_pub),
                                   transitions = {'success': 'findGripper',
                                                  'failure': 'doNothing'})
            smach.StateMachine.add('doNothing',DoNothing(self.armName, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'success': 'doNothing'})
            smach.StateMachine.add('findGripper', FindGripper(self.imageDetector, self.gripperName),
                                   transitions = {'success': 'planTrajToObject',
                                                  'failure': 'rotateGripper'})
            smach.StateMachine.add('rotateGripper', RotateGripper(self.ravenArm),
                                   transitions = {'success': 'findGripper'})
            smach.StateMachine.add('planTrajToObject', PlanTrajToObject(self.ravenArm, self.ravenPlanner, self.stepsPerMeter,
                                                                        self.transFrame, self.rotFrame, self.gripperOpenCloseDuration),
                                   transitions = {'reachedObject': 'objectServoSuccessMoveVertical',
                                                  'notReachedObject': 'moveTowardsObject',
                                                  'failure': 'moveToHome'})
            smach.StateMachine.add('moveTowardsObject', MoveTowardsObject(self.ravenArm, self.stepsPerMeter, self.maxServoDistance),
                                   transitions = {'success': 'findGripper'})
            smach.StateMachine.add('objectServoSuccessMoveVertical',MoveVertical(self.ravenArm, self.ravenPlanner, self.openLoopSpeed),
                                   transitions = {'success': 'checkPickup'})
            smach.StateMachine.add('checkPickup', CheckPickup(self.ravenArm, self.gripperOpenCloseDuration),
                                   transitions = {'success': 'pickupSuccessMoveToHome',
                                                  'failure': 'findObject'})
            smach.StateMachine.add('pickupSuccessMoveToHome', MoveToHome(self.ravenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                   transitions = {'success': 'moveToReceptacle'})
            smach.StateMachine.add('moveToReceptacle', MoveToReceptacle(self.ravenArm, self.ravenPlanner, self.openLoopSpeed, self.gripperOpenCloseDuration),
                                   transitions = {'success': 'moveToHome'})
            smach.StateMachine.add('moveToHome', MoveToHome(self.ravenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                   transitions = {'success': 'findObject'})
        
        self.otherRavenArm = RavenArm(self.otherArmName)
        
        self.other_sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['objectHeightOffset'])
        with self.other_sm:
            other_sm_type = None
            #other_sm_type = 'nothing'
            #other_sm_type = 'updown'
            if other_sm_type == 'nothing':
                smach.StateMachine.add('doNothing',DoNothing(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'success': 'doNothing'})
            elif other_sm_type == 'updown':
                smach.StateMachine.add('moveUp',MoveUp(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'success': 'moveDown'})
                smach.StateMachine.add('moveDown',MoveDown(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'success': 'moveUp'})
            else:
                smach.StateMachine.add('findReceptacle', FindReceptacle(self.otherArmName, self.imageDetector), 
                                   transitions = {'success': 'findHome',
                                                  'failure': 'findReceptacle'})
                smach.StateMachine.add('findHome', FindHome(self.otherArmName, self.imageDetector),
                                       transitions = {'success': 'moveToReceptacle',
                                                     'failure': 'findHome'})
                smach.StateMachine.add('findObject', FindObject(self.otherFoamAllocator, self.otherToolframe, self.obj_pub),
                                       transitions = {'success': 'findGripper',
                                                      'failure': 'success'})
                smach.StateMachine.add('findGripper', FindGripper(self.imageDetector, self.otherGripperName),
                                       transitions = {'success': 'planTrajToObject',
                                                      'failure': 'rotateGripper'})
                smach.StateMachine.add('rotateGripper', RotateGripper(self.otherRavenArm),
                                       transitions = {'success': 'findGripper'})
                smach.StateMachine.add('planTrajToObject', PlanTrajToObject(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter,
                                                                            self.otherTransFrame, self.otherRotFrame, self.gripperOpenCloseDuration),
                                       transitions = {'reachedObject': 'objectServoSuccessMoveVertical',
                                                      'notReachedObject': 'moveTowardsObject',
                                                      'failure': 'moveToHome'})
                smach.StateMachine.add('moveTowardsObject', MoveTowardsObject(self.otherRavenArm, self.stepsPerMeter, self.maxServoDistance),
                                       transitions = {'success': 'findGripper'})
                smach.StateMachine.add('objectServoSuccessMoveVertical',MoveVertical(self.otherRavenArm, self.ravenPlanner, self.openLoopSpeed),
                                       transitions = {'success': 'checkPickup'})
                smach.StateMachine.add('checkPickup', CheckPickup(self.otherRavenArm, self.gripperOpenCloseDuration),
                                       transitions = {'success': 'pickupSuccessMoveToHome',
                                                      'failure': 'findObject'})
                smach.StateMachine.add('pickupSuccessMoveToHome', MoveToHome(self.otherRavenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                       transitions = {'success': 'moveToReceptacle'})
                smach.StateMachine.add('moveToReceptacle', MoveToReceptacle(self.otherRavenArm, self.ravenPlanner, self.openLoopSpeed, self.gripperOpenCloseDuration),
                                       transitions = {'success': 'moveToHome'})
                smach.StateMachine.add('moveToHome', MoveToHome(self.otherRavenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                       transitions = {'success': 'findObject'})
    
    def run(self):
        self.ravenArm.start()
        self.otherRavenArm.start()
        
        sis = smach_ros.IntrospectionServer('master_server_%s' % self.armName, self.sm, '/SM_%s' % self.armName)
        sis.start()
        userData = smach.UserData()
        userData['objectHeightOffset'] = self.objectHeightOffset
        
        otherSis = smach_ros.IntrospectionServer('master_server_%s' % self.otherArmName, self.other_sm, '/SM_%s' % self.otherArmName)
        otherSis.start()
        otherUserData = smach.UserData()
        otherUserData['objectHeightOffset'] = self.objectHeightOffset
        
        try:
            #outcome = self.sm.execute(userData)
            thread1 = threading.Thread(target=self.sm.execute,args=(userData,))
            thread1.daemon = True
            thread1.start()
            
            thread2 = threading.Thread(target=self.other_sm.execute,args=(otherUserData,))
            thread2.daemon = True
            thread2.start()
        except Exception, e:
            print e
        
        rospy.spin()

        self.ravenArm.stop()
        self.otherRavenArm.stop()

def mainloop():
    """
    Gets an instance of the MasterClass
    for the left arm and executes the
    run loop
    """
    rospy.init_node('master_node',anonymous=False)
    armName = Constants.Arm.Right
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--pause',action='store_true')
    parser.add_argument('--smooth',action='store_false', dest='pause')
    args = parser.parse_args(rospy.myargv()[1:])
    
    if args.pause is not None:
        MasterClass.PAUSE_BETWEEN_STATES = args.pause
    
    imageDetector = ImageDetector()
    foamAllocator = FoamAllocator()
    ravenArm = RavenArm(armName)
    ravenPlanner = RavenPlanner(Constants.Arm.Both)
    master = MasterClass(armName, ravenArm, ravenPlanner, imageDetector, foamAllocator)
    master.run()


if __name__ == '__main__':
    mainloop()
    

#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math
from math import *

import numpy as np

import tf
import tf.transformations as tft
import tfx

import smach
import smach_ros
smach.set_shutdown_check(rospy.is_shutdown)

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion
from raven_pose_estimator.srv import ThreshRed

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.RavenCommand.RavenPlanner2 import RavenPlanner
from RavenDebridement.RavenCommand.RavenBSP import RavenPlannerBSP
from RavenDebridement.ImageProcessing.ImageDetection import ImageDetector
from RavenDebridement.ImageProcessing.FoamAllocator import FoamAllocator,\
    ArmFoamAllocator
from RavenDebridement.ImageProcessing.GripperPoseEstimator import GripperPoseEstimator

import threading

import IPython

def pause_func(myclass):
    arm = getattr(myclass,'armName',None)
    if not arm and hasattr(myclass,'ravenArm'):
        arm = myclass.ravenArm.armName
    rospy.loginfo('In {0} method on arm {1}. Press enter to continue'.format(myclass,arm))
    raw_input()
    
class DoNothing(smach.State):
    def __init__(self, ravenArm, ravenPlanner, completer=None):
        smach.State.__init__(self, outcomes = ['success','failure','complete'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.completer = completer

    def execute(self, userdata):
        rospy.sleep(.5)
        #if MasterClass.PAUSE_BETWEEN_STATES:
        #    pause_func(self)
            
        #rospy.loginfo('Doing nothing')
        
        if self.completer and self.completer.isComplete():
            return 'complete'

        endPose = self.ravenArm.getGripperPose()
        n_steps = 5
        #print 'requesting trajectory for otherarm', self.armName
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.armName, endPose, n_steps=n_steps)
        
        if poseTraj == None:
            return 'failure'

        return 'success'
    
class Completer(object):
    def __init__(self, armNames):
        self.armNames = armNames
        self._complete = dict([(arm, False) for arm in self.armNames])
    
    def isComplete(self):
        return all(self._complete.values())
    
    def complete(self, armName):
        self._complete[armName] = True
    
    def __str__(self):
        return 'complete: %s' % self._complete
    
    __repr__ = __str__

class ReceptacleToken(object):
    def __init__(self):
        self.lock = threading.Lock()
        
        self.tokenHolder = None
    
    def requestToken(self, armName):
        with self.lock:
            if self.tokenHolder is None or self.tokenHolder == armName:
                self.tokenHolder = armName
                return True
            else:
                return False
    
    def releaseToken(self, armName):
        with self.lock:
            if self.tokenHolder and self.tokenHolder != armName:
                raise RuntimeError("Arm %s is trying to release token held by %s!" % (armName, self.tokenHolder))
            self.tokenHolder = None
    
class WaitForCompletion(smach.State):
    def __init__(self, ravenArm, ravenPlanner, completer):
        smach.State.__init__(self, outcomes = ['success','failure'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.completer = completer

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Waiting for completion')
        
        while not self.completer.isComplete():
            endPose = self.ravenArm.getGripperPose()
            n_steps = 5
            print 'requesting trajectory for arm', self.armName
            poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.armName, endPose, n_steps=n_steps)
            rospy.sleep(0.5)
            
            if poseTraj is None:
                return 'failure'
        
        return 'success'
    
class AllocateFoam(smach.State):
    def __init__(self, armName, foamAllocator, completer=None):
        smach.State.__init__(self, outcomes = ['foamFound','noFoamFound'], input_keys = ['foamOffset'], output_keys = ['foamPose'])
        self.armName = armName
        self.foamAllocator = foamAllocator
        self.completer = completer
        
        self.foam_pub = rospy.Publisher('foam_allocator_%s'%armName, PoseStamped)
    
    def didNotFindFoam(self):
        rospy.loginfo('Did not find foam')
        if self.completer:
            self.completer.complete(self.foamAllocator.armName)
            print self.foamAllocator.armName, self.completer
        return 'noFoamFound'

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)

        new=False
        
        rospy.loginfo('Allocating foam %s' % self.armName)
        
        if not self.foamAllocator.hasFoam(new=False):
            rospy.loginfo('has foam returning false')
            return self.didNotFindFoam()
        
        foamPose = self.foamAllocator.allocateFoam(new=False)

        if foamPose is None:
            return self.didNotFindFoam()
        
        foamPose = tfx.pose(foamPose) + userdata.foamOffset
        
        """
        # find foam point and pose
        if not self.foamAllocator.hasFoam(new=new):
            rospy.sleep(1)
        if not self.foamAllocator.hasFoam(new=new):
            rospy.loginfo('Did not find foam')
            if self.completer:
                self.completer.complete(self.foamAllocator.armName)
                print self.foamAllocator.armName, self.completer
            return 'noFoamFound'
        # get foam w.r.t. toolframe
        foamPose = tfx.pose(self.foamAllocator.allocateFoam(new=new)) + userdata.foamOffset
        """
        self.foam_pub.publish(foamPose.msg.PoseStamped())
        
        userdata.foamPose = foamPose.msg.PoseStamped()
        
        print 'foamPose %s' % self.armName
        print foamPose

        rospy.loginfo('Found foam')
        return 'foamFound'
    
    
class PlanTrajToFoam(smach.State):
    def __init__(self, ravenArm, gripperPoseEstimator, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['success', 'failure'],
                             output_keys = ['deltaPoseTraj','gripperPose'],
                             io_keys = ['foamPose'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.gripperPoseEstimator = gripperPoseEstimator
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        
        self.endPosePub = rospy.Publisher('traj_end_pose_%s'%self.armName,PoseStamped)

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
        
        foamPose = tfx.pose(userdata.foamPose).copy()
        gripperPose = tfx.pose(self.gripperPoseEstimator.getGripperPose(self.armName)).copy()
        
        userdata.gripperPose = tfx.pose(gripperPose).msg.PoseStamped()
        
        #objectPose.orientation = gripperPose.orientation
        
        deltaPose = Util.deltaPose(gripperPose, foamPose, self.transFrame, self.rotFrame)
             
        rospy.loginfo('Planning trajectory from gripper to object')
        
        endPoseForPub = Util.endPose(gripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.endPosePub.publish(endPoseForPub.msg.PoseStamped())

        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        deltaPoseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, foamPose, startPose=gripperPose, n_steps=n_steps, approachDir=np.array([0,0,1]))


        if deltaPoseTraj == None:
            return 'failure'

        userdata.deltaPoseTraj = deltaPoseTraj
        return 'success'
    
    
class MoveTowardsFoam(smach.State):
    def __init__(self, ravenArm, maxServoDistance, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['reachedFoam', 'notReachedFoam'], input_keys = ['deltaPoseTraj','gripperPose'], io_keys = ['foamPose'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.maxServoDistance = maxServoDistance
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        
        self.midPosePub = rospy.Publisher('traj_mid_pose_%s'%self.armName,PoseStamped)

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        gripperPose = tfx.pose(userdata.gripperPose).copy()
        foamPose = tfx.pose(userdata.foamPose).copy()
            
        transBound = .008
        rotBound = float("inf")
        if Util.withinBounds(gripperPose, foamPose, transBound, rotBound, self.transFrame, self.rotFrame):
            rospy.loginfo('Reached foam piece')
            return 'reachedFoam'

        rospy.loginfo('Moving towards the foam piece')
        deltaPoseTraj = userdata.deltaPoseTraj

        # limit distance moved
        for index in xrange(len(deltaPoseTraj)):
            deltaPose = tfx.pose(deltaPoseTraj[index])
            if deltaPose.position.norm > self.maxServoDistance:
                endTrajStep = index
                break
        else:
            endTrajStep = -1
            
        self.midPosePub.publish((Util.endPose(gripperPose,deltaPoseTraj[endTrajStep]).msg.PoseStamped()))
            
        truncDeltaPoseTraj = deltaPoseTraj[:endTrajStep]
        
        rospy.loginfo('Total steps')
        rospy.loginfo(len(userdata.deltaPoseTraj))
        rospy.loginfo('endTrajStep')
        rospy.loginfo(endTrajStep)

        self.ravenArm.executeDeltaPoseTrajectory(truncDeltaPoseTraj,block=True)

        return 'notReachedFoam'
    
class PlanTrajToReceptacle(smach.State):
    def __init__(self, holdingPose, ravenArm, gripperPoseEstimator, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['success', 'failure'],
                             output_keys = ['deltaPoseTraj','gripperPose'])
        self.holdingPose = holdingPose
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.gripperPoseEstimator = gripperPoseEstimator
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        
        self.endPosePub = rospy.Publisher('traj_end_pose_%s'%self.armName,PoseStamped)

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
        
        holdingPose = tfx.pose(self.holdingPose)
        gripperPose = self.gripperPoseEstimator.getGripperPose(self.armName)
        
        userdata.gripperPose = tfx.pose(gripperPose).msg.PoseStamped()
        
        #objectPose.orientation = gripperPose.orientation
        
        deltaPose = Util.deltaPose(gripperPose, holdingPose, self.transFrame, self.rotFrame)
             
        rospy.loginfo('Planning trajectory from gripper to receptacle')
        
        endPoseForPub = Util.endPose(gripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.endPosePub.publish(endPoseForPub.msg.PoseStamped())

        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        deltaPoseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, holdingPose, startPose=gripperPose, n_steps=n_steps)


        if deltaPoseTraj == None:
            return 'failure'

        userdata.deltaPoseTraj = deltaPoseTraj
        return 'success'
    
    
class MoveTowardsReceptacle(smach.State):
    def __init__(self, ravenArm, maxServoDistance, transFrame, rotFrame, holdingPose, receptacleLock):
        smach.State.__init__(self, outcomes = ['notReachedReceptacle', 'receptacleAvailable','receptacleOccupied','foamNotInGripper'], input_keys = ['deltaPoseTraj','gripperPose'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.maxServoDistance = maxServoDistance
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        
        self.holdingPose = holdingPose
        self.receptacleLock = receptacleLock
        
        self.foamInGripper = True
        rospy.Subscriber('found_red_%s' % self.armName, Bool, self._foamTrackerCallback)
        
        
    def _foamTrackerCallback(self, msg):
        self.foamInGripper = msg.data
        

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
            
        gripperPose = tfx.pose(userdata.gripperPose)
        holdingPose = tfx.pose(self.holdingPose)
            
        transBound = .006
        rotBound = float("inf")
        if Util.withinBounds(gripperPose, holdingPose, transBound, rotBound, self.transFrame, self.rotFrame):
            if self.receptacleLock.requestToken(self.armName):
                rospy.loginfo('Near open receptacle. Moving in for drop-off.')
                return 'receptacleAvailable'
            else:
                rospy.loginfo('Other arm at receptacle. Must wait.')
                return 'receptacleOccupied'
                

        rospy.loginfo('Moving towards the receptacle')
        deltaPoseTraj = userdata.deltaPoseTraj

        # limit distance moved
        for index in xrange(len(deltaPoseTraj)):
            deltaPose = tfx.pose(deltaPoseTraj[index])
            if deltaPose.position.norm > self.maxServoDistance:
                endTrajStep = index
                break
        else:
            endTrajStep = -1
        
        truncDeltaPoseTraj = deltaPoseTraj[:endTrajStep]
        
        rospy.loginfo('Total steps')
        rospy.loginfo(len(userdata.deltaPoseTraj))
        rospy.loginfo('endTrajStep')
        rospy.loginfo(endTrajStep)

        
        self.ravenArm.executeDeltaPoseTrajectory(truncDeltaPoseTraj,block=False)
        while not self.ravenArm.isPaused() and not rospy.is_shutdown():
            if not self.foamInGripper:
                return 'foamNotInGripper'
            rospy.sleep(.5)

        return 'notReachedReceptacle'
    

class GraspFoam(smach.State):
    def __init__(self, ravenArm, gripperPoseEstimator, gripperOpenCloseDuration):
        smach.State.__init__(self, outcomes = ['graspedFoam'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.gripperPoseEstimator = gripperPoseEstimator
        self.duration = gripperOpenCloseDuration
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        self.gripperPoseEstimator.suppressImageEstimation(self.armName)
            
        #self.ravenArm.openGripper(duration=1)
        self.ravenArm.setGripper(-.2, duration=3)
        
        return 'graspedFoam'
    
class DropFoamInReceptacle(smach.State):
    def __init__(self, ravenArm, gripperPoseEstimator, gripperOpenCloseDuration, receptaclePose, receptacleLock, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['findNextFoamPiece'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.gripperPoseEstimator = gripperPoseEstimator
        self.duration = gripperOpenCloseDuration
        self.receptaclePose = receptaclePose
        self.receptacleLock = receptacleLock
        self.transFrame = transFrame
        self.rotFrame = rotFrame
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        receptaclePose = tfx.pose(self.receptaclePose).copy()
        gripperPose = self.gripperPoseEstimator.getGripperPose(self.armName)
        calcGripperPose = startPose = tfx.pose(self.ravenArm.getGripperPose())
        
        deltaPose = Util.deltaPose(gripperPose, receptaclePose, self.transFrame, self.rotFrame)
             
        rospy.loginfo('Planning trajectory from gripper to receptacle')

        endPose = Util.endPose(calcGripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.ravenArm.goToGripperPose(endPose)
        
        self.ravenArm.openGripper()
        
        self.gripperPoseEstimator.enableImageEstimation(self.armName)
        
        # go back to original position so out of the way
        self.ravenArm.goToGripperPose(startPose)
        
        self.receptacleLock.releaseToken(self.armName)
        
        return 'findNextFoamPiece'
    
class WaitForReceptacle(smach.State):
    def __init__(self, armName, receptacleLock, ravenPlanner, ravenArm):
        smach.State.__init__(self, outcomes = ['receptacleAvailable'])
        self.armName = armName
        self.receptacleLock = receptacleLock
        self.ravenPlanner = ravenPlanner
        self.ravenArm = ravenArm
        
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
        
        rospy.loginfo('Waiting for receptacle lock to be released')
        endPose = self.ravenArm.getGripperPose()
        n_steps = 5
        while not rospy.is_shutdown() and not self.receptacleLock.requestToken(self.armName):
            self.ravenPlanner.getTrajectoryFromPose(self.armName, endPose, n_steps=n_steps)
            rospy.sleep(.1)
            
        rospy.loginfo('Acquired receptacle lock')        
        
        return 'receptacleAvailable'
    
    

class Move(smach.State):
    """
    Same as DoNothing, but moves the arm
    """
    def __init__(self, ravenArm, ravenPlanner, movementVector,completer=None):
        smach.State.__init__(self, outcomes = ['success','failure','complete'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.movementVector = movementVector
        self.completer = completer

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Moving up')

        if self.completer and self.completer.isComplete():
            return 'complete'

        transBound = .006
        rotBound = float("inf")
        endPose = self.ravenArm.getGripperPose() + self.movementVector
        n_steps = 5
        print 'requesting trajectory for otherarm', self.armName
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.armName, endPose, n_steps=n_steps)
        
        if poseTraj == None:
            return 'failure'
        
        self.ravenArm.executePoseTrajectory(poseTraj,block=True)

        return 'success'
    
    
    





class MasterClass(object):
    PAUSE_BETWEEN_STATES = False
    file_lock = threading.Lock()
    output_file = open('/tmp/master_output.txt','w')
    
    @classmethod
    def write(cls,*args,**kwargs):
        with cls.file_lock:
            cls.output_file.write(*args,**kwargs)
    
    def __init__(self, armName, ravenPlanner, foamAllocator, gripperPoseEstimator, receptaclePose, closedGraspValues=dict()):
        self.armName = armName
        
        if armName == Constants.Arm.Left:
            self.gripperName = Constants.Arm.Left
            self.toolframe = Constants.Frames.LeftTool
            self.holdingPose = receptaclePose + [.03, -.01, .03]
            self.otherHoldingPose = receptaclePose + [-.03, -.01, .03]
            self.otherArmName = Constants.Arm.Right
            self.otherGripperName = Constants.Arm.Right
            self.otherToolframe = Constants.Frames.RightTool
        elif armName == Constants.Arm.Right:
            self.gripperName = Constants.Arm.Right
            self.toolframe = Constants.Frames.RightTool
            self.holdingPose = receptaclePose + [-.03, -.01, .03]
            self.otherHoldingPose = receptaclePose + [.03, -.01, .03]
            self.otherArmName = Constants.Arm.Left
            self.otherGripperName = Constants.Arm.Left
            self.otherToolframe = Constants.Frames.LeftTool
        
        other_sm_type = None
        #other_sm_type = 'nothing'
        #other_sm_type = 'updown'
        
        if other_sm_type is None:
            self.completer = Completer([self.armName,self.otherArmName])
        else:
            self.completer = Completer([self.armName])

        self.listener = tf.TransformListener.instance()

        self.ravenArm = RavenArm(self.armName, closedGraspValues.get(self.armName,0.))
        self.ravenPlanner = ravenPlanner
        self.gripperPoseEstimator = gripperPoseEstimator
        
        self.foamAllocator = ArmFoamAllocator(self.armName,foamAllocator)
        self.otherFoamAllocator = ArmFoamAllocator(self.otherArmName,foamAllocator)
        
        self.receptaclePose = receptaclePose  
        self.receptacleLock = ReceptacleToken()
        
        # translation frame
        self.transFrame = Constants.Frames.Link0
        self.otherTransFrame = Constants.Frames.Link0
        # rotation frame
        self.rotFrame = self.toolframe
        self.otherRotFrame = self.otherToolframe

        # height offset for foam
        self.foamOffset = [0.,.003,.004]

        # in cm/sec, I think
        self.openLoopSpeed = .01

        self.gripperOpenCloseDuration = 7

        # for Trajopt planning, 2 steps/cm
        self.stepsPerMeter = 600

        # move no more than 5cm per servo
        self.maxServoDistance = .025

        # debugging outputs
        self.des_pose_pub = rospy.Publisher('desired_pose', PoseStamped)
        self.obj_pub = rospy.Publisher('object_pose', PoseStamped)
        
        def arm(s):
            return '{0}_{1}'.format(s, self.armName)
        
        def otherArm(s):
            return '{0}_{1}'.format(s, self.otherArmName)
        
        self.sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['foamOffset'])  

        
        with self.sm:
            self.setup_sm(self.armName, self.ravenArm, self.foamAllocator, self.transFrame, self.rotFrame, self.holdingPose)
        
        
        self.otherRavenArm = RavenArm(self.otherArmName, closedGraspValues.get(self.otherArmName,0.))
        
        self.other_sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['foamOffset'])
        
        with self.other_sm:
            if other_sm_type == 'nothing':
                smach.StateMachine.add(otherArm('doNothing'), DoNothing(self.otherRavenArm, self.ravenPlanner, completer=self.completer),
                                       transitions = {'success': otherArm('doNothing'),
                                                      'complete' : 'success',
                                                      'failure' : 'failure'})
            elif other_sm_type == 'updown':
                smach.StateMachine.add(otherArm('moveUp'), Move(self.otherRavenArm, self.ravenPlanner, [0,0,.01], completer=self.completer),
                                       transitions = {'success' : otherArm('moveDown'),
                                                      'complete' : 'success',
                                                      'failure' : 'failure'})
                smach.StateMachine.add(otherArm('moveDown'), Move(self.otherRavenArm, self.ravenPlanner, [0,0,-.01], completer=self.completer),
                                       transitions = {'success' : otherArm('moveUp'),
                                                      'complete' : 'success',
                                                      'failure' : 'failure'})
            else:
                self.setup_sm(self.otherArmName, self.otherRavenArm, self.otherFoamAllocator, self.otherTransFrame, self.otherRotFrame, self.otherHoldingPose)
        
    def setup_sm(self, armName, ravenArm, foamArmAllocator, transFrame, rotFrame, holdingPose):
        def arm(s):
            return '{0}_{1}'.format(s, armName)
        
        smach.StateMachine.add(arm('allocateFoam'), AllocateFoam(armName, foamArmAllocator, completer=self.completer),
                               transitions = {'foamFound' : arm('planTrajToFoam'),
                                              'noFoamFound' : arm('waitForOtherArmToComplete')})
        smach.StateMachine.add(arm('waitForOtherArmToComplete'), WaitForCompletion(ravenArm, self.ravenPlanner, self.completer),
                               transitions = {'success' : 'success',
                                              'failure' : 'failure'})
        smach.StateMachine.add(arm('planTrajToFoam'), PlanTrajToFoam(ravenArm, self.gripperPoseEstimator, self.ravenPlanner,
                                                               self.stepsPerMeter, transFrame, rotFrame),
                               transitions = {'success' : arm('moveTowardsFoam'),
                                              'failure' : 'failure'})
        smach.StateMachine.add(arm('moveTowardsFoam'), MoveTowardsFoam(ravenArm, self.maxServoDistance, transFrame, rotFrame),
                               transitions = {'reachedFoam' : arm('graspFoam'),
                                              'notReachedFoam' : arm('planTrajToFoam')})
        smach.StateMachine.add(arm('graspFoam'), GraspFoam(ravenArm, self.gripperPoseEstimator, self.gripperOpenCloseDuration),
                               transitions = {'graspedFoam' : arm('planTrajToReceptacle')})
        smach.StateMachine.add(arm('planTrajToReceptacle'), PlanTrajToReceptacle(holdingPose, ravenArm, self.gripperPoseEstimator,
                                                                                 self.ravenPlanner, self.stepsPerMeter, transFrame, rotFrame),
                               transitions = {'success' : arm('moveTowardsReceptacle'),
                                              'failure' : 'failure'})
        smach.StateMachine.add(arm('moveTowardsReceptacle'), MoveTowardsReceptacle(ravenArm, self.maxServoDistance,transFrame, rotFrame,
                                                                                   holdingPose, self.receptacleLock),
                               transitions = {'notReachedReceptacle' : arm('planTrajToReceptacle'),
                                              'receptacleAvailable' : arm('dropFoamInReceptacle'),
                                              'receptacleOccupied' : arm('waitForReceptacle'),
                                              'foamNotInGripper' : arm('allocateFoam')})
        smach.StateMachine.add(arm('waitForReceptacle'), WaitForReceptacle(armName, self.receptacleLock, self.ravenPlanner, ravenArm),
                               transitions = {'receptacleAvailable' : arm('dropFoamInReceptacle')})
        smach.StateMachine.add(arm('dropFoamInReceptacle'), DropFoamInReceptacle(ravenArm, self.gripperPoseEstimator, self.gripperOpenCloseDuration,
                                                                                 self.receptaclePose, self.receptacleLock, transFrame, rotFrame),
                               transitions = {'findNextFoamPiece' : arm('allocateFoam')})
    
    def run(self):
        self.ravenArm.start()
        self.otherRavenArm.start()
        
        sis = smach_ros.IntrospectionServer('master_server_%s' % self.armName, self.sm, '/SM_%s' % self.armName)
        sis.start()
        userData = smach.UserData()
        userData['foamOffset'] = self.foamOffset
        
        otherSis = smach_ros.IntrospectionServer('master_server_%s' % self.otherArmName, self.other_sm, '/SM_%s' % self.otherArmName)
        otherSis.start()
        otherUserData = smach.UserData()
        otherUserData['foamOffset'] = self.foamOffset
        
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
        
        rospy.sleep(2)
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.sm.is_running() and not self.other_sm.is_running():
                rospy.signal_shutdown('state machines finished')
                break
            rate.sleep()

        self.ravenArm.stop()
        self.otherRavenArm.stop()

def mainloop():
    rospy.init_node('master_node',anonymous=False)
    armName = Constants.Arm.Right
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--pause',action='store_true')
    parser.add_argument('--smooth',action='store_false', dest='pause')
    parser.add_argument('--interactive',action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    
    if args.pause is not False:
        MasterClass.PAUSE_BETWEEN_STATES = args.pause
        
    import trajoptpy
    if args.interactive is not False:
        print('Interactive trajopt')
        trajoptpy.SetInteractive(True)
    
    foamAllocator = FoamAllocator()
    gripperPoseEstimator = GripperPoseEstimator(Constants.Arm.Both)
    ravenPlanner = RavenPlanner(Constants.Arm.Both)
    receptaclePose = tfx.pose([-.066, .006, -.139], tfx.tb_angles(-90,90,0),frame='0_link')
    closedGraspValues = {Constants.Arm.Left : 0., Constants.Arm.Right : -30.}
    master = MasterClass(armName, ravenPlanner, foamAllocator, gripperPoseEstimator, receptaclePose)
    master.run()


if __name__ == '__main__':
    mainloop()
    

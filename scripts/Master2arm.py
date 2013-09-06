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
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Doing nothing')
        
        if self.completer and self.completer.isComplete():
            return 'complete'

        endPose = self.ravenArm.getGripperPose()
        n_steps = 5
        print 'requesting trajectory for otherarm', self.armName
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
    
class AllocateFoam(smach.State):
    def __init__(self, armName, foamAllocator, completer=None):
        smach.State.__init__(self, outcomes = ['foamFound','noFoamFound'], input_keys = ['foamHeightOffset'], output_keys = ['foamPose'])
        self.armName = armName
        self.foamAllocator = foamAllocator
        self.completer = completer
        
        self.foam_pub = rospy.Publisher('foam_allocator_%s'%armName, PoseStamped)
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        # TEMP
        # change foamAllocator so it doesn't restrict based on allocation
        new = True

        rospy.loginfo('Searching for foam')
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
        foamPose = self.foamAllocator.allocateFoam(new=new)
        foamPose.position.z += userdata.foamHeightOffset
        self.foam_pub.publish(foamPose.msg.PoseStamped())
        
        print foamPose

        userdata.foamPose = foamPose.msg.PoseStamped()

        rospy.loginfo('Found foam')
        return 'foamFound'
    
    
class PlanTrajToFoam(smach.State):
    def __init__(self, ravenArm, gripperPoseEstimator, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['success', 'failure'],
                             output_keys = ['poseTraj','gripperPose'],
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
        gripperPose = self.gripperPoseEstimator.getGripperPose(self.armName)
        calcGripperPose = tfx.pose(self.ravenArm.getGripperPose())
        
        userdata.gripperPose = tfx.pose(gripperPose).copy()
        
        #objectPose.orientation = gripperPose.orientation
        
        deltaPose = Util.deltaPose(gripperPose, foamPose, self.transFrame, self.rotFrame)
             
        rospy.loginfo('Planning trajectory from gripper to object')
        
        endPoseForPub = Util.endPose(gripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.endPosePub.publish(endPoseForPub.msg.PoseStamped())

        endPose = Util.endPose(calcGripperPose, deltaPose, frame=Constants.Frames.Link0)
        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps, approachDir=np.array([0,.1,.9]))


        if poseTraj == None:
            return 'failure'

        userdata.poseTraj = poseTraj
        return 'success'
    
    
class MoveTowardsFoam(smach.State):
    def __init__(self, ravenArm, maxServoDistance, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['reachedFoam', 'notReachedFoam'], input_keys = ['poseTraj','gripperPose'], io_keys = ['foamPose'])
        self.ravenArm = ravenArm
        self.maxServoDistance = maxServoDistance
        self.transFrame = transFrame
        self.rotFrame = rotFrame

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
        poseTraj = userdata.poseTraj

        # limit distance moved
        currPose = self.ravenArm.getGripperPose()
        for index in range(len(poseTraj)):
            pose = tfx.pose(poseTraj[index])
            deltaPos = np.array(currPose.position.list) - np.array(pose.position.list)
            if np.linalg.norm(deltaPos) > self.maxServoDistance:
                endTrajStep = index
                break
        else:
            endTrajStep = -1
            
        truncPoseTraj = poseTraj[:endTrajStep]
        
        rospy.loginfo('Total steps')
        rospy.loginfo(len(userdata.poseTraj))
        rospy.loginfo('endTrajStep')
        rospy.loginfo(endTrajStep)

        self.ravenArm.executePoseTrajectory(truncPoseTraj,block=True)

        return 'notReachedFoam'
    
class PlanTrajToReceptacle(smach.State):
    def __init__(self, receptaclePose, ravenArm, gripperPoseEstimator, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['success', 'failure'],
                             output_keys = ['poseTraj','gripperPose'])
        self.receptaclePose = receptaclePose
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
        
        receptaclePose = tfx.pose(self.receptaclePose).copy()
        gripperPose = self.gripperPoseEstimator.getGripperPose(self.armName)
        calcGripperPose = tfx.pose(self.ravenArm.getGripperPose())
        
        userdata.gripperPose = tfx.pose(gripperPose).copy()
        
        #objectPose.orientation = gripperPose.orientation
        
        deltaPose = Util.deltaPose(gripperPose, receptaclePose, self.transFrame, self.rotFrame)
             
        rospy.loginfo('Planning trajectory from gripper to receptacle')
        
        endPoseForPub = Util.endPose(gripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.endPosePub.publish(endPoseForPub.msg.PoseStamped())

        endPose = Util.endPose(calcGripperPose, deltaPose, frame=Constants.Frames.Link0)
        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps)


        if poseTraj == None:
            return 'failure'

        userdata.poseTraj = poseTraj
        return 'success'
    
    
class MoveTowardsReceptacle(smach.State):
    def __init__(self, ravenArm, maxServoDistance, transFrame, rotFrame, receptaclePose, receptacleLock):
        smach.State.__init__(self, outcomes = ['notReachedReceptacle', 'receptacleAvailable','receptacleOccupied'], input_keys = ['poseTraj','gripperPose'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.maxServoDistance = maxServoDistance
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        
        self.receptaclePose = receptaclePose
        self.receptacleLock = receptacleLock
        self.lockDistance = 1.5 * maxServoDistance

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        gripperPose = tfx.pose(userdata.gripperPose).copy()
        receptaclePose = tfx.pose(self.receptaclePose).copy()
            
        transBound = self.lockDistance
        rotBound = float("inf")
        if Util.withinBounds(gripperPose, receptaclePose, transBound, rotBound, self.transFrame, self.rotFrame):
            if self.receptacleLock.locked_lock():
                rospy.loginfo('Other arm at receptacle. Must wait.')
                return 'receptacleOccupied'
            else:
                rospy.loginfo('Near open receptacle. Moving in for drop-off.')
                self.receptaclLock.acquire()
                return 'receptacleAvailable'

        rospy.loginfo('Moving towards the receptacle')
        poseTraj = userdata.poseTraj

        # limit distance moved
        currPose = self.ravenArm.getGripperPose()
        for index in range(len(poseTraj)):
            pose = tfx.pose(poseTraj[index])
            deltaPos = np.array(currPose.position.list) - np.array(pose.position.list)
            if np.linalg.norm(deltaPos) > self.maxServoDistance:
                endTrajStep = index
                break
        else:
            endTrajStep = -1
            
        truncPoseTraj = poseTraj[:endTrajStep]
        
        rospy.loginfo('Total steps')
        rospy.loginfo(len(userdata.poseTraj))
        rospy.loginfo('endTrajStep')
        rospy.loginfo(endTrajStep)

        self.ravenArm.executePoseTrajectory(truncPoseTraj,block=True)

        return 'notReachedReceptacle'
    

class GraspFoam(smach.State):
    def __init__(self, ravenArm, gripperOpenCloseDuration):
        smach.State.__init__(self, outcomes = ['graspedFoam'])
        self.ravenArm = ravenArm
        self.duration = gripperOpenCloseDuration
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        self.ravenArm.closeGripper(duration=self.gripperOpenCloseDuration)
        
        return 'graspedFoam'
    
class DropFoamInReceptacle(smach.State):
    def __init__(self, ravenArm, gripperPoseEstimator, gripperOpenCloseDuration, receptaclePose, receptacleLock, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['findNextFoamPiece'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.gripperPoseEstimator = gripperPoseEstimator
        self.duration = gripperOpenCloseDuration
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
        
        self.ravenArm.setGripper(0.75)
        
        # go back to original position so out of the way
        self.ravenArm.goToGripperPose(startPose)
        
        self.receptacleLock.release()
        
        return 'success'
    
class ReceptacleLock():
    def __init__(self):
        self.lock = threading.Lock()
        self.armThatHasLock = None
        
    def isLockedByOther(self, armName):
        if self.armThatHasLock is None or self.armThatHasLock == armName:
            return False
        
        return True
    
    def setArmThatHasLock(self, armName):
        self.armThatHasLock = armName
        
    def acquire(self):
        self.lock.acquire()
        
    def locked_lock(self):
        return self.lock.locked_lock()
        
    def release(self):
        self.lock.release()
        self.armThatHasLock = None
        
    
    
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
        while not rospy.is_shutdown() and self.receptacleLock.locked_lock():
            self.ravenPlanner.getTrajectoryFromPose(self.armName, endPose, n_steps=n_steps)
            rospy.sleep(.1)
            
        self.receptacleLock.acquire()
    
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
    
    def __init__(self, armName, ravenArm, ravenPlanner, imageDetector, foamAllocator, gripperPoseEstimator):
        self.armName = armName
        
        if armName == Constants.Arm.Left:
            self.gripperName = Constants.Arm.Left
            self.toolframe = Constants.Frames.LeftTool
            self.otherArmName = Constants.Arm.Right
            self.otherGripperName = Constants.Arm.Right
            self.otherToolframe = Constants.Frames.RightTool
        elif armName == Constants.Arm.Right:
            self.gripperName = Constants.Arm.Right
            self.toolframe = Constants.Frames.RightTool
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

        self.imageDetector = imageDetector
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.gripperPoseEstimator = gripperPoseEstimator
        
        self.foamAllocator = ArmFoamAllocator(self.armName,foamAllocator)
        self.otherFoamAllocator = ArmFoamAllocator(self.otherArmName,foamAllocator)
        
        self.receptaclePose = self.imageDetector.getReceptaclePose(self.armName)
        self.otherReceptaclePose = self.imageDetector.getReceptaclePose(self.otherArmName)
        
        self.receptacleLock = threading.Lock()
        
        # translation frame
        self.transFrame = Constants.Frames.Link0
        self.otherTransFrame = Constants.Frames.Link0
        # rotation frame
        self.rotFrame = self.toolframe
        self.otherRotFrame = self.otherToolframe

        # height offset for foam
        self.foamHeightOffset = .004

        # in cm/sec, I think
        self.openLoopSpeed = .01

        self.gripperOpenCloseDuration = 7

        # for Trajopt planning, 2 steps/cm
        self.stepsPerMeter = 200

        # move no more than 5cm per servo
        self.maxServoDistance = .05

        # debugging outputs
        self.des_pose_pub = rospy.Publisher('desired_pose', PoseStamped)
        self.obj_pub = rospy.Publisher('object_pose', PoseStamped)
        
        def arm(s):
            return '{0}_{1}'.format(s, self.armName)
        
        def otherArm(s):
            return '{0}_{1}'.format(s, self.otherArmName)
        
        self.sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['foamHeightOffset'])  

        
        with self.sm:
            smach.StateMachine.add(arm('allocateFoam'), AllocateFoam(self.armName, self.foamAllocator, completer=self.completer),
                                   transitions = {'foamFound' : arm('planTrajToFoam'),
                                                  'noFoamFound' : arm('waitForCompletion')})
            smach.StateMachine.add(arm('waitForCompletion'), DoNothing(self.ravenArm, self.ravenPlanner, completer=self.completer),
                                   transitions = {'success' : arm('waitForCompletion'),
                                                  'complete' : 'success',
                                                  'failure' : 'failure'})
            smach.StateMachine.add(arm('planTrajToFoam'), PlanTrajToFoam(self.ravenArm, self.gripperPoseEstimator, self.ravenPlanner,
                                                                   self.stepsPerMeter, self.transFrame, self.rotFrame),
                                   transitions = {'success' : arm('moveTowardsFoam'),
                                                  'failure' : 'failure'})
            smach.StateMachine.add(arm('moveTowardsFoam'), MoveTowardsFoam(self.ravenArm, self.maxServoDistance, self.transFrame, self.rotFrame),
                                   transitions = {'reachedFoam' : arm('graspFoam'),
                                                  'notReachedFoam' : arm('planTrajToFoam')})
            smach.StateMachine.add(arm('graspFoam'), GraspFoam(self.ravenArm, self.gripperOpenCloseDuration),
                                   transitions = {'graspedFoam' : arm('planTrajToReceptacle')})
            smach.StateMachine.add(arm('planTrajToReceptacle'), PlanTrajToReceptacle(self.receptaclePose, self.ravenArm, self.gripperPoseEstimator,
                                                                                     self.ravenPlanner, self.stepsPerMeter, self.transFrame, self.rotFrame),
                                   transitions = {'success' : arm('moveTowardsReceptacle'),
                                                  'failure' : 'failure'})
            smach.StateMachine.add(arm('moveTowardsReceptacle'), MoveTowardsReceptacle(self.ravenArm, self.maxServoDistance,self.transFrame, self.rotFrame,
                                                                                       self.receptaclePose, self.receptacleLock),
                                   transitions = {'notReachedReceptacle' : arm('planTrajToReceptacle'),
                                                  'receptacleAvailable' : arm('dropFoamInReceptacle'),
                                                  'receptacleOccupied' : arm('waitForReceptacle')})
            smach.StateMachine.add(arm('waitForReceptacle'), WaitForReceptacle(self.armName, self.receptacleLock, self.ravenPlanner, self.ravenArm),
                                   transitions = {'receptacleAvailable' : arm('dropFoamInReceptacle')})
            smach.StateMachine.add(arm('dropFoamInReceptacle'), DropFoamInReceptacle(self.ravenArm, self.gripperPoseEstimator, self.gripperOpenCloseDuration,
                                                                                     self.receptaclePose, self.receptacleLock, self.transFrame, self.rotFrame),
                                   transitions = {'findNextFoamPiece' : arm('allocateFoam')})
        
        
        self.otherRavenArm = RavenArm(self.otherArmName)
        
        self.other_sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['foamHeightOffset'])
        
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
                smach.StateMachine.add(otherArm('allocateFoam'), AllocateFoam(self.otherArmName, self.otherFoamAllocator, completer=self.completer),
                                   transitions = {'foamFound' : otherArm('planTrajToFoam'),
                                                  'noFoamFound' : otherArm('waitForCompletion')})
                smach.StateMachine.add(otherArm('waitForCompletion'), DoNothing(self.otherRavenArm, self.ravenPlanner, completer=self.completer),
                                       transitions = {'success' : otherArm('waitForCompletion'),
                                                      'complete' : 'success',
                                                      'failure' : 'failure'})
                smach.StateMachine.add(otherArm('planTrajToFoam'), PlanTrajToFoam(self.otherRavenArm, self.gripperPoseEstimator, self.ravenPlanner,
                                                                       self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'success' : otherArm('moveTowardsFoam'),
                                                      'failure' : 'failure'})
                smach.StateMachine.add(otherArm('moveTowardsFoam'), MoveTowardsFoam(self.otherRavenArm, self.maxServoDistance, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'reachedFoam' : otherArm('graspFoam'),
                                                      'notReachedFoam' : otherArm('planTrajToFoam')})
                smach.StateMachine.add(otherArm('graspFoam'), GraspFoam(self.otherRavenArm, self.gripperOpenCloseDuration),
                                       transitions = {'graspedFoam' : otherArm('planTrajToReceptacle')})
                smach.StateMachine.add(otherArm('planTrajToReceptacle'), PlanTrajToReceptacle(self.otherReceptaclePose, self.otherRavenArm, self.gripperPoseEstimator,
                                                                                         self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'success' : otherArm('moveTowardsReceptacle'),
                                                      'failure' : 'failure'})
                smach.StateMachine.add(otherArm('moveTowardsReceptacle'), MoveTowardsReceptacle(self.otherRavenArm, self.maxServoDistance,self.otherTransFrame, self.otherRotFrame,
                                                                                           self.otherReceptaclePose, self.receptacleLock),
                                       transitions = {'notReachedReceptacle' : otherArm('planTrajToReceptacle'),
                                                      'receptacleAvailable' : otherArm('dropFoamInReceptacle'),
                                                      'receptacleOccupied' : otherArm('waitForReceptacle')})
                smach.StateMachine.add(otherArm('waitForReceptacle'), WaitForReceptacle(self.otherArmName, self.receptacleLock, self.ravenPlanner, self.otherRavenArm),
                                       transitions = {'receptacleAvailable' : otherArm('dropFoamInReceptacle')})
                smach.StateMachine.add(otherArm('dropFoamInReceptacle'), DropFoamInReceptacle(self.otherRavenArm, self.gripperPoseEstimator, self.gripperOpenCloseDuration,
                                                                                         self.otherReceptaclePose, self.receptacleLock, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'findNextFoamPiece' : otherArm('allocateFoam')})
        

    
    def run(self):
        self.ravenArm.start()
        self.otherRavenArm.start()
        
        sis = smach_ros.IntrospectionServer('master_server_%s' % self.armName, self.sm, '/SM_%s' % self.armName)
        sis.start()
        userData = smach.UserData()
        userData['foamHeightOffset'] = self.foamHeightOffset
        
        otherSis = smach_ros.IntrospectionServer('master_server_%s' % self.otherArmName, self.other_sm, '/SM_%s' % self.otherArmName)
        otherSis.start()
        otherUserData = smach.UserData()
        otherUserData['foamHeightOffset'] = self.foamHeightOffset
        
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
    args = parser.parse_args(rospy.myargv()[1:])
    
    if args.pause is not None:
        MasterClass.PAUSE_BETWEEN_STATES = args.pause
    
    imageDetector = ImageDetector()
    foamAllocator = FoamAllocator()
    gripperPoseEstimator = GripperPoseEstimator(Constants.Arm.Both)
    ravenArm = RavenArm(armName)
    ravenPlanner = RavenPlanner(Constants.Arm.Both)
    master = MasterClass(armName, ravenArm, ravenPlanner, imageDetector, foamAllocator, gripperPoseEstimator)
    master.run()


if __name__ == '__main__':
    mainloop()
    

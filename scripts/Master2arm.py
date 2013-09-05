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
        smach.State.__init__(self, outcomes = ['success','failure'], input_keys = ['foamHeightOffset'], output_keys = ['goalPose'])
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

        userdata.goalPose = objectPose.msg.PoseStamped()

        rospy.loginfo('Found foam')
        return 'success'
    
    
class PlanTraj(smach.State):
    def __init__(self, ravenArm, gripperPoseEstimator, ravenPlanner, stepsPerMeter, transFrame, rotFrame):
        smach.State.__init__(self, outcomes = ['reachedGoalPose', 'notReachedGoalPose','failure'],
                             output_keys = ['poseTraj'],
                             io_keys = ['goalPose'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.gripperPoseEstimator = gripperPoseEstimator
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        
        self.endPosePub = rospy.Publisher('traj_end_pose',PoseStamped)

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
        
        goalPose = tfx.pose(userdata.goalPose).copy()
        gripperPose = self.gripperPoseEstimator.getGripperPose(self.armName)
        calcGripperPose = tfx.pose(self.ravenArm.getGripperPose())
        
        #objectPose.orientation = gripperPose.orientation
        
        deltaPose = Util.deltaPose(gripperPose, goalPose, self.transFrame, self.rotFrame)
        
        transBound = .008
        rotBound = float("inf")
        in_bounds = Util.withinBounds(gripperPose, goalPose, transBound, rotBound, self.transFrame, self.rotFrame)
        
            
        rospy.loginfo('Planning trajectory from gripper to object')
        
        if in_bounds:
            return 'reachedGoalPose'

        endPoseForPub = Util.endPose(gripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.endPosePub.publish(endPoseForPub.msg.PoseStamped())

        endPose = Util.endPose(calcGripperPose, deltaPose, frame=Constants.Frames.Link0)
        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps, approachDir=np.array([0,.1,.9]))


        if poseTraj == None:
            return 'failure'

        userdata.poseTraj = poseTraj
        return 'notReachedGoalPose'
    
    
class MoveTowards(smach.State):
    def __init__(self, ravenArm, maxServoDistance):
        smach.State.__init__(self, outcomes = ['success', 'goalOccupied'], input_keys = ['poseTraj'], io_keys = ['goalPose'])
        self.ravenArm = ravenArm
        self.maxServoDistance = maxServoDistance

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)

        rospy.loginfo('Moving towards the goal')
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

        return 'success'
    

class CloseGripper(smach.State):
    def __init__(self, ravenArm, gripperOpenCloseDuration, nextGoalPose):
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['goalPose'])
        self.ravenArm = ravenArm
        self.duration = gripperOpenCloseDuration
        self.nextGoalPose = nextGoalPose
    
    def execute(self, userdata):
        self.ravenArm.closeGripper(duration=self.gripperOpenCloseDuration)
        
        userdata.goalPose = self.nextGoalPose
        
        return 'success'
    
class OpenGripper(smach.State):
    def __init__(self, ravenArm, gripperOpenCloseDuration, receptacleLock):
        smach.State.__init__(self, outcomes = ['success'])
        self.ravenArm = ravenArm
        self.duration = gripperOpenCloseDuration
        self.receptacleLock = receptacleLock
    
    def execute(self, userdata):
        self.receptacleLock.release()
        
        self.ravenArm.setGripper(0.75)
        
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
    def __init__(self, receptacleLock, maxServoDistance, ravenArm, ravenPlanner):
        smach.State.__init__(self, outcomes = ['success'], io_keys = ['goalPose'])
        self.receptacleLock = receptacleLock
        self.lockDistance = 2*maxServoDistance
        self.isLockedByMe = False
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
        
        currentPose = tfx.pose(self.ravenArm.getGripperPose())    
        goalPose = tfx.pose(userdata.goalPose)
        
        distanceToGoal = currentPose.position.distance(goalPose.position)
        
        if distanceToGoal < self.lockDistance:
            rospy.loginfo('Within receptacle buffer')
            if not self.receptacleLock.locked_lock():
                self.receptacleLock.acquire()
                self.receptacleLock.setArmThatHasLock(self.armName)
            elif self.receptacleLock.locked_lock() and self.receptacleLock.isLockedByOther(self.armName):
                rospy.loginfo('Waiting for receptacle lock to be released')
                endPose = self.ravenArm.getGripperPose()
                n_steps = 5
                while not rospy.is_shutdown() and self.receptacleLock.locked_lock():
                    self.ravenPlanner.getTrajectoryFromPose(self.armName, endPose, n_steps=n_steps)
                    rospy.sleep(.1)
                    
                self.receptacleLock.acquire()
                self.receptacleLock.setArmThatHasLock(self.armName)
            
            rospy.loginfo('Acquired receptacle lock')
        else:
            rospy.loginfo('Not within receptacle buffer')
            
            # just in case moved in and out of buffer
            if not self.receptacleLock.isLockedByOther(self.armName):
                self.receptacleLock.release()
        
        
        return 'success'
    
    

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
        
        self.receptacleLock = ReceptacleLock()
        
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
            return s + self.armName
        
        def otherArm(s):
            return s + self.otherArmName
        
        self.sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['foamHeightOffset'])
        

        
        with self.sm:
            smach.StateMachine.add(arm('allocateFoam'), AllocateFoam(self.armName, self.foamAllocator, completer=self.completer),
                                   transitions = {'success' : arm('planTrajToFoam'),
                                                  'noFoamFound' : arm('waitForCompletion')})
            smach.StateMachine.add(arm('waitForCompletion'), DoNothing(self.ravenArm, self.ravenPlanner, completer=self.completer),
                                   transitions = {'success' : arm('waitForCompletion'),
                                                  'complete' : 'success',
                                                  'failure' : 'failure'})
            smach.StateMachine.add(arm('planTrajToFoam'), PlanTraj(self.ravenArm, self.gripperPoseEstimator, self.ravenPlanner,
                                                                   self.stepsPerMeter, self.transFrame, self.rotFrame),
                                   transitions = {'reachedGoalPose' : arm('graspFoam'),
                                                  'notReachedGoalPose': arm('moveTowardsFoam'),
                                                  'failure' : 'failure'})
            smach.StateMachine.add(arm('moveTowardsFoam'), MoveTowards(self.ravenArm, self.maxServoDistance),
                                   transitions = {'success' : arm('planTrajToFoam')})
            smach.StateMachine.add(arm('graspFoam'), CloseGripper(self.ravenArm, self.gripperOpenCloseDuration, self.receptaclePose),
                                   transitions = {'success' : arm('planTrajToReceptacle')})
            smach.StateMachine.add(arm('planTrajToReceptacle'), PlanTraj(self.ravenArm, self.gripperPoseEstimator, self.ravenPlanner,
                                                                         self.stepsPerMeter, self.transFrame, self.rotFrame),
                                   transitions = {'reachedGoalPose' : arm('releaseFoam'),
                                                  'notReachedGoalPose' : arm('moveTowardsReceptacle'),
                                                  'failure' : 'failure'})
            smach.StateMachine.add(arm('moveTowardsReceptacle'), MoveTowards(self.ravenArm, self.maxServoDistance),
                                   transitions = {'success' : arm('waitForReceptacle')})
            smach.StateMachine.add(arm('waitForReceptacle'), WaitForReceptacle(self.receptacleLock, self.maxServoDistance, self.ravenArm, self.ravenPlanner),
                                   transitions = {'success' : arm('planTrajToReceptacle')})
            smach.StateMachine.add(arm('releaseFoam'), OpenGripper(self.ravenArm, self.gripperOpenCloseDuration, self.receptacleLock),
                                   transitions = {'success' : arm('allocateFoam')})
        
        
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
                                       transitions = {'success' : otherArm('planTrajToFoam'),
                                                      'noFoamFound' : otherArm('waitForCompletion')})
                smach.StateMachine.add(otherArm('waitForCompletion'), DoNothing(self.otherRavenArm, self.ravenPlanner, completer=self.completer),
                                       transitions = {'success' : otherArm('waitForCompletion'),
                                                      'complete' : 'success',
                                                      'failure' : 'failure'})
                smach.StateMachine.add(otherArm('planTrajToFoam'), PlanTraj(self.otherRavenArm, self.gripperPoseEstimator, self.ravenPlanner,
                                                                       self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'reachedGoalPose' : otherArm('graspFoam'),
                                                      'notReachedGoalPose': otherArm('moveTowardsFoam'),
                                                      'failure' : 'failure'})
                smach.StateMachine.add(otherArm('moveTowardsFoam'), MoveTowards(self.otherRavenArm, self.maxServoDistance),
                                       transitions = {'success' : otherArm('planTrajToFoam')})
                smach.StateMachine.add(otherArm('graspFoam'), CloseGripper(self.otherRavenArm, self.gripperOpenCloseDuration, self.otherReceptaclePose),
                                       transitions = {'success' : otherArm('planTrajToReceptacle')})
                smach.StateMachine.add(otherArm('planTrajToReceptacle'), PlanTraj(self.otherRavenArm, self.gripperPoseEstimator, self.ravenPlanner,
                                                                             self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame),
                                       transitions = {'reachedGoalPose' : otherArm('releaseFoam'),
                                                      'notReachedGoalPose' : otherArm('moveTowardsReceptacle'),
                                                      'failure' : 'failure'})
                smach.StateMachine.add(otherArm('moveTowardsReceptacle'), MoveTowards(self.otherRavenArm, self.maxServoDistance),
                                       transitions = {'success' : otherArm('waitForReceptacle')})
                smach.StateMachine.add(otherArm('waitForReceptacle'), WaitForReceptacle(self.receptacleLock, self.maxServoDistance, self.otherRavenArm, self.ravenPlanner),
                                       transitions = {'success' : otherArm('planTrajToReceptacle')})
                smach.StateMachine.add(otherArm('releaseFoam'), OpenGripper(self.otherRavenArm, self.gripperOpenCloseDuration, self.receptacleLock),
                                       transitions = {'success' : otherArm('allocateFoam')})
        

    
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
    

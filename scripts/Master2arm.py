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
from RavenDebridement.RavenCommand.RavenBSP import RavenBSP
from RavenDebridement.ImageProcessing.ImageDetection import ImageDetector
from RavenDebridement.ImageProcessing.FoamAllocator import FoamAllocator,\
    ArmFoamAllocator

import threading

import IPython

def pause_func(myclass):
    arm = getattr(myclass,'armName',None)
    if not arm and hasattr(myclass,'ravenArm'):
        arm = myclass.ravenArm.armName
    rospy.loginfo('In {0} method on arm {1}. Press enter to continue'.format(myclass,arm))
    raw_input()

class DoNothing(smach.State):
    def __init__(self, ravenArm, ravenPlanner, stepsPerMeter, transFrame, rotFrame, completer=None):
        smach.State.__init__(self, outcomes = ['success','failure','complete'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        self.completer = completer

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Doing nothing')
        
        if self.completer and self.completer.isComplete():
            return 'complete'

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
    def __init__(self, ravenArm, ravenPlanner, stepsPerMeter, transFrame, rotFrame, completer=None):
        smach.State.__init__(self, outcomes = ['success','failure','complete'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        self.completer = completer

    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Moving up')

        if self.completer and self.completer.isComplete():
            return 'complete'

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
    def __init__(self, ravenArm, ravenPlanner, stepsPerMeter, transFrame, rotFrame, completer=None):
        smach.State.__init__(self, outcomes = ['success','failure','complete'])
        self.ravenArm = ravenArm
        self.ravenPlanner = ravenPlanner
        self.stepsPerMeter = stepsPerMeter
        self.transFrame = transFrame
        self.rotFrame = rotFrame
        self.completer = completer

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
    def __init__(self, foamAllocator, toolframe, obj_pub, completer=None):
        smach.State.__init__(self, outcomes = ['success','failure'], input_keys = ['objectHeightOffset'], output_keys = ['rotateBy'], io_keys=['objectPose'])
        self.foamAllocator = foamAllocator
        self.toolframe = toolframe
        self.obj_pub = obj_pub
        self.completer = completer
    
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
        if not self.foamAllocator.hasFoam(new=new):
            rospy.sleep(1)
        if not self.foamAllocator.hasFoam(new=new):
            rospy.loginfo('Did not find object')
            if self.completer:
                self.completer.complete(self.foamAllocator.armName)
                print self.foamAllocator.armName, self.completer
            return 'failure'
        # get object w.r.t. toolframe
        objectPose = self.foamAllocator.allocateFoam(new=new)
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
        objectPose = tfx.pose(userdata.objectPose)
        gripperPose = tfx.pose(userdata.gripperPose)
        calcGripperPose = tfx.pose(self.ravenArm.getGripperPose())
        
        #objectPose.orientation = gripperPose.orientation
        
        self.objectPosePub.publish(objectPose.msg.PoseStamped())
        
        deltaPose = tfx.pose(Util.deltaPose(gripperPose, objectPose, self.transFrame, self.rotFrame))
        
        transBound = .008
        rotBound = float("inf")
        in_bounds = Util.withinBounds(gripperPose, objectPose, transBound, rotBound, self.transFrame, self.rotFrame)
        
        not_str = '' if in_bounds else 'not '
        
        reached_str = 'arm %s %sat object! deltas: pos %.4f, angle %.5f, pose %s' % (
                self.ravenArm.armName,
                not_str, 
                deltaPose.position.norm,
                deltaPose.orientation.angle * 180. / pi,
                deltaPose.tostring())
        print reached_str
        MasterClass.write(reached_str + '\n')
        
        if MasterClass.PAUSE_BETWEEN_STATES:
            pause_func(self)
            
        rospy.loginfo('Planning trajectory from gripper to object')
        
        if in_bounds:
            rospy.loginfo('Closing the gripper')
            self.ravenArm.closeGripper(duration=self.gripperOpenCloseDuration)
            userdata.vertAmount = .04
            return 'reachedObject'

        endPoseForPub = Util.endPose(gripperPose, deltaPose, frame=Constants.Frames.Link0)
        self.endPosePub.publish(endPoseForPub.msg.PoseStamped())

        endPose = Util.endPose(calcGripperPose, deltaPose, frame=Constants.Frames.Link0)
        n_steps = int(self.stepsPerMeter * deltaPose.position.norm) + 1
        poseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, endPose, n_steps=n_steps, approachDir=np.array([0,.1,.9]))
        
#         print('objectPose')
#         print(objectPose)
#         print('gripperPose')
#         print(gripperPose)
#         print('calcGripperPose')
#         print(calcGripperPose)
#         print('endPoseForPub')
#         print(endPoseForPub)
#         print('endPose')
#         print(endPose)
#         print('deltaPose')
#         print(deltaPose)
        
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
        currPose = self.ravenArm.getGripperPose()
        for index in range(len(poseTraj)):
            pose = tfx.pose(poseTraj[index])
            deltaPos = np.array(currPose.position.list) - np.array(pose.position.list)
            if np.linalg.norm(deltaPos) > self.maxServoDistance:
                endTrajStep = index
                break
        else:
            endTrajStep = -1
            
        #endTrajStep = min(int(self.stepsPerMeter*self.maxServoDistance)+1, len(poseTraj))
        #poseTraj = poseTraj[:endTrajStep]
        truncPoseTraj = poseTraj[:endTrajStep]
        
        rospy.loginfo('Total steps')
        rospy.loginfo(len(userdata.poseTraj))
        rospy.loginfo('endTrajStep')
        rospy.loginfo(endTrajStep)

        self.ravenArm.executePoseTrajectory(truncPoseTraj,block=True)

        return 'success'


class MoveVertical(smach.State):
    """
    Move vertical in open-loop
    """
    def __init__(self, ravenArm, ravenPlanner, openLoopSpeed):
        smach.State.__init__(self, outcomes = ['success'], io_keys=['vertAmount'])
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
    def __init__(self, ravenArm, imageDetector, gripperOpenCloseDuration):
        smach.State.__init__(self, outcomes = ['success','failure'], input_keys=['vertAmount'])
        self.armName = ravenArm.name
        self.ravenArm = ravenArm
        self.imageDetector = imageDetector
        self.gripperOpenCloseDuration = gripperOpenCloseDuration
    
    def execute(self, userdata):
        if MasterClass.PAUSE_BETWEEN_STATES:
           pause_func(self)

        rospy.loginfo('Check if red foam piece successfully picked up')
        
        gripperPose = self.imageDetector.getGripperPose(self.armName)
        convGripperPose = tfx.pose(Util.convertToFrame(gripperPose, Constants.Frames.Link0))
        lastGripperPoint = convGripperPose.position
        vertErrorMargin=.005
        lowerBoundGripperPoint = lastGripperPoint + [0,0,userdata.vertAmount-vertErrorMargin]

        serviceName = '{0}_{1}'.format(Constants.Services.isFoamGrasped, self.armName)

        # last tape pose position + vertAmount

        try:
            rospy.wait_for_service(serviceName, timeout=5)
            foamGraspedService = rospy.ServiceProxy(serviceName, ThreshRed)
            isFoamGrasped = foamGraspedService(lowerBoundGripperPoint.msg.PointStamped()).output
            
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
        #self.ravenArm.openGripper(duration=self.gripperOpenCloseDuration)
        self.ravenArm.setGripper(0.75)
        
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

class MasterClass(object):
    PAUSE_BETWEEN_STATES = False
    file_lock = threading.Lock()
    output_file = open('/tmp/master_output.txt','w')
    
    @classmethod
    def write(cls,*args,**kwargs):
        with cls.file_lock:
            cls.output_file.write(*args,**kwargs)
    
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
        
        other_sm_type = None
        #other_sm_type = 'nothing'
        #other_sm_type = 'updown'
        
        if other_sm_type is None:
            self.completer = Completer([self.armName,self.otherArmName])
        else:
            self.completer = Completer([self.armName])

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
        
        self.sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['objectHeightOffset'])
        
        with self.sm:
            smach.StateMachine.add(arm('findReceptacle'), FindReceptacle(self.armName, self.imageDetector), 
                                   transitions = {'success': arm('findHome'),
                                                  'failure': arm('findReceptacle')})
            smach.StateMachine.add(arm('findHome'), FindHome(self.armName, self.imageDetector),
                                   transitions = {'success': arm('moveToReceptacle'),
                                                 'failure': arm('findHome')})
            smach.StateMachine.add(arm('findObject'), FindObject(self.foamAllocator, self.toolframe, self.obj_pub, completer=self.completer),
                                   transitions = {'success': arm('findGripper'),
                                                  'failure': arm('doNothing')})
            smach.StateMachine.add(arm('doNothing'),DoNothing(self.ravenArm, self.ravenPlanner, self.stepsPerMeter, self.transFrame, self.rotFrame, completer=self.completer),
                                       transitions = {'success': arm('doNothing'),
                                                      'complete': 'success'})
            smach.StateMachine.add(arm('findGripper'), FindGripper(self.imageDetector, self.gripperName),
                                   transitions = {'success': arm('planTrajToObject'),
                                                  'failure': arm('rotateGripper')})
            smach.StateMachine.add(arm('rotateGripper'), RotateGripper(self.ravenArm),
                                   transitions = {'success': arm('findGripper')})
            smach.StateMachine.add(arm('planTrajToObject'), PlanTrajToObject(self.ravenArm, self.ravenPlanner, self.stepsPerMeter,
                                                                        self.transFrame, self.rotFrame, self.gripperOpenCloseDuration),
                                   transitions = {'reachedObject': arm('objectServoSuccessMoveVertical'),
                                                  'notReachedObject': arm('moveTowardsObject'),
                                                  'failure': arm('moveToHome')})
            smach.StateMachine.add(arm('moveTowardsObject'), MoveTowardsObject(self.ravenArm, self.stepsPerMeter, self.maxServoDistance),
                                   transitions = {'success': arm('findGripper')})
            smach.StateMachine.add(arm('objectServoSuccessMoveVertical'),MoveVertical(self.ravenArm, self.ravenPlanner, self.openLoopSpeed),
                                   transitions = {'success': arm('checkPickup')})
            smach.StateMachine.add(arm('checkPickup'), CheckPickup(self.ravenArm, self.imageDetector, self.gripperOpenCloseDuration),
                                   transitions = {'success': arm('pickupSuccessMoveToHome'),
                                                  'failure': arm('findObject')})
            smach.StateMachine.add(arm('pickupSuccessMoveToHome'), MoveToHome(self.ravenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                   transitions = {'success': arm('moveToReceptacle')})
            smach.StateMachine.add(arm('moveToReceptacle'), MoveToReceptacle(self.ravenArm, self.ravenPlanner, self.openLoopSpeed, self.gripperOpenCloseDuration),
                                   transitions = {'success': arm('moveToHome')})
            smach.StateMachine.add(arm('moveToHome'), MoveToHome(self.ravenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                   transitions = {'success': arm('findObject')})
        
        self.otherRavenArm = RavenArm(self.otherArmName)
        
        self.other_sm = smach.StateMachine(outcomes=['success','failure'],input_keys=['objectHeightOffset'])
        with self.other_sm:
            if other_sm_type == 'nothing':
                smach.StateMachine.add(otherArm('doNothing'),DoNothing(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame, completer=self.completer),
                                       transitions = {'success': otherArm('doNothing'),
                                                      'complete': 'success'})
            elif other_sm_type == 'updown':
                smach.StateMachine.add(otherArm('moveUp'),MoveUp(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame, completer=self.completer),
                                       transitions = {'success': otherArm('moveDown'),
                                                      'complete': 'success'})
                smach.StateMachine.add(otherArm('moveDown'),MoveDown(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame, completer=self.completer),
                                       transitions = {'success': otherArm('moveUp'),
                                                      'complete': 'success'})
            else:
                smach.StateMachine.add(otherArm('findReceptacle'), FindReceptacle(self.otherArmName, self.imageDetector), 
                                   transitions = {'success': otherArm('findHome'),
                                                  'failure': otherArm('findReceptacle')})
                smach.StateMachine.add(otherArm('findHome'), FindHome(self.otherArmName, self.imageDetector),
                                       transitions = {'success': otherArm('moveToReceptacle'),
                                                     'failure': otherArm('findHome')})
                smach.StateMachine.add(otherArm('findObject'), FindObject(self.otherFoamAllocator, self.otherToolframe, self.obj_pub, completer=self.completer),
                                       transitions = {'success': otherArm('findGripper'),
                                                      'failure': otherArm('doNothing')})
                smach.StateMachine.add(otherArm('doNothing'),DoNothing(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter, self.otherTransFrame, self.otherRotFrame, completer=self.completer),
                                       transitions = {'success': otherArm('doNothing'),
                                                      'complete': 'success'})
                smach.StateMachine.add(otherArm('findGripper'), FindGripper(self.imageDetector, self.otherGripperName),
                                       transitions = {'success': otherArm('planTrajToObject'),
                                                      'failure': otherArm('rotateGripper')})
                smach.StateMachine.add(otherArm('rotateGripper'), RotateGripper(self.otherRavenArm),
                                       transitions = {'success': otherArm('findGripper')})
                smach.StateMachine.add(otherArm('planTrajToObject'), PlanTrajToObject(self.otherRavenArm, self.ravenPlanner, self.stepsPerMeter,
                                                                            self.otherTransFrame, self.otherRotFrame, self.gripperOpenCloseDuration),
                                       transitions = {'reachedObject': otherArm('objectServoSuccessMoveVertical'),
                                                      'notReachedObject': otherArm('moveTowardsObject'),
                                                      'failure': otherArm('moveToHome')})
                smach.StateMachine.add(otherArm('moveTowardsObject'), MoveTowardsObject(self.otherRavenArm, self.stepsPerMeter, self.maxServoDistance),
                                       transitions = {'success': otherArm('findGripper')})
                smach.StateMachine.add(otherArm('objectServoSuccessMoveVertical'),MoveVertical(self.otherRavenArm, self.ravenPlanner, self.openLoopSpeed),
                                       transitions = {'success': otherArm('checkPickup')})
                smach.StateMachine.add(otherArm('checkPickup'), CheckPickup(self.otherRavenArm, self.imageDetector, self.gripperOpenCloseDuration),
                                       transitions = {'success': otherArm('pickupSuccessMoveToHome'),
                                                      'failure': otherArm('findObject')})
                smach.StateMachine.add(otherArm('pickupSuccessMoveToHome'), MoveToHome(self.otherRavenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                       transitions = {'success': otherArm('moveToReceptacle')})
                smach.StateMachine.add(otherArm('moveToReceptacle'), MoveToReceptacle(self.otherRavenArm, self.ravenPlanner, self.openLoopSpeed, self.gripperOpenCloseDuration),
                                       transitions = {'success': otherArm('moveToHome')})
                smach.StateMachine.add(otherArm('moveToHome'), MoveToHome(self.otherRavenArm, self.ravenPlanner, self.imageDetector, self.openLoopSpeed),
                                       transitions = {'success': otherArm('findObject')})
    
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
    

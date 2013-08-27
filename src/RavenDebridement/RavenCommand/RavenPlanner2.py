#!/usr/bin/env python


import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from math import *
import os

from raven_2_msgs.msg import * 
from geometry_msgs.msg import *

from RavenDebridement.srv import InvKinSrv


import openravepy as rave
import trajoptpy
import json
import numpy as np

import threading

import tfx

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants

import IPython

def jointRequest(n_steps, endJointPositions):
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            "start_fixed" : True
            },
        "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]}
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [100],
                        "continuous" : True,
                        "dist_pen" : [0.01]
                        }
                    },
               {
                "type" : "collision",
                "params" : {
                    "coeffs" : [100],
                    "continuous" : False,
                    "dist_pen" : [0.01]
                    }
                },
                ],
        "constraints" : [
            {
                "type" : "joint", # joint-space target
                "params" : {"vals" : endJointPositions } # length of vals = # dofs of manip
                }
                
            ],
        "init_info" : {
            "type" : "straight_line",
            "endpoint" : endJointPositions
            }
        }

    return request

def jointTrajRequest(n_steps, jointTraj):
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            "start_fixed" : True
            },
        "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]}
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [100],
                        "continuous" : True,
                        "dist_pen" : [0.001]
                        }
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [100000],
                        "continuous" : False,
                        "dist_pen" : [0.01]
                        }
                    },
                ],
        "constraints" : [
            {
                "type" : "joint", # joint-space target
                "params" : {"vals" : jointTraj[-1,:].tolist() } # length of vals = # dofs of manip
                }
                
            ],
        "init_info" : {
            "type" : "given_traj",
            "endpoint" : jointTraj.tolist()
            }
        }

    return request

class RavenPlanner:
    rosJointTypes = [Constants.JOINT_TYPE_SHOULDER,
                     Constants.JOINT_TYPE_ELBOW,
                     Constants.JOINT_TYPE_INSERTION,
                     Constants.JOINT_TYPE_ROTATION,
                     Constants.JOINT_TYPE_PITCH,
                     Constants.JOINT_TYPE_YAW]

    raveJointNamesPrefixes = ["shoulder",
                              "elbow",
                              "insertion",
                              "tool_roll",
                              "wrist_joint",
                              "grasper_yaw"]
    
    defaultJointPositions = [.512, 1.6, -.2, .116, .088, 0]
    defaultJoints = dict([(jointType,jointPos) for jointType, jointPos in zip(rosJointTypes,defaultJointPositions)])

    def __init__(self, armNames, thread=True):
        if isinstance(armNames,basestring):
            armNames = [armNames]
        self.armNames = sorted(armNames)
        
        self.refFrame = MyConstants.Frames.Link0

        self.env = rave.Environment()

        rospy.loginfo('Before loading model')
        ravenFile = os.path.join(roslib.packages.get_pkg_subdir('RavenDebridement','models'),'myRaven.xml')
        #ravenFile = '/home/gkahn/ros_workspace/RavenDebridement/models/myRaven.xml'
        self.env.Load(ravenFile)
        rospy.loginfo('After loading model')

        #trajoptpy.SetInteractive(True)

        self.robot = self.env.GetRobots()[0]
        
        self.invKinArm = dict()
        self.toolFrame = dict()
        self.manipName = dict()
        self.manip = dict()
        self.manipJoints = dict()
        self.raveJointNames = dict()
        self.raveJointTypes = dict()
        self.raveJointTypesToRos = dict()
        self.rosJointTypesToRave = dict()
        self.raveGrasperJointNames = dict()
        self.raveGrasperJointTypes = dict()
        
        self.trajRequest = dict()
        
        self.trajEndJoints = dict()
        self.trajEndGrasp = dict()
        
        self.trajStartJoints = dict()
        self.trajStartGrasp = dict()
        
        self.trajReqType = None
        self.trajSteps = dict()
        
        self.jointTraj = dict() # for debugging
        self.poseTraj = dict()
        
        activeDOFs = []
        for armName in self.armNames:
            self._init_arm(armName)
            activeDOFs += self.raveJointTypes[armName]
            
        self.robot.SetActiveDOFs(activeDOFs)
        
        self.currentState = None
        rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)
        
        self.start_pose_pubs = dict((armName, rospy.Publisher('planner_%s_start' % armName,PoseStamped)) for armName in self.armNames)
        self.end_pose_pubs = dict((armName, rospy.Publisher('planner_%s_end' % armName,PoseStamped)) for armName in self.armNames)
        
        self.lock = threading.RLock()
        if thread:
            self.thread = threading.Thread(target=self.optimizeLoop)
            self.thread.setDaemon(True)
            self.thread.start()

    def _init_arm(self, armName):
        if armName == MyConstants.Arm.Left:
            self.invKinArm[armName] = Constants.ARM_TYPE_GOLD
            self.toolFrame[armName] = MyConstants.OpenraveLinks.LeftTool
            self.manipName[armName] = 'left_arm'
        else:
            self.invKinArm[armName] = Constants.ARM_TYPE_GREEN
            self.toolFrame[armName] = MyConstants.OpenraveLinks.RightTool
            self.manipName[armName] = 'right_arm'
        self.robot.SetActiveManipulator(self.manipName[armName])
        self.manip[armName] = self.robot.GetActiveManipulator()
        self.manipJoints[armName] = self.robot.GetJoints(self.manip[armName].GetArmJoints())


        self.raveJointNames[armName] = ['{0}_{1}'.format(name, armName[0].upper()) for name in self.raveJointNamesPrefixes]

        self.raveJointTypes[armName] = [self.robot.GetJointIndex(name) for name in self.raveJointNames[armName]]
        self.raveJointTypesToRos[armName] = dict((rave,ros) for rave,ros in zip(self.raveJointTypes[armName], self.rosJointTypes))
        self.rosJointTypesToRave[armName] = dict((ros,rave) for ros,rave in zip(self.rosJointTypes, self.raveJointTypes[armName]))
        
        self.raveGrasperJointNames[armName] = ['grasper_joint_1_{0}'.format(armName[0].upper()), 'grasper_joint_2_{0}'.format(armName[0].upper())]
        self.raveGrasperJointTypes[armName] = [self.robot.GetJointIndex(name) for name in self.raveGrasperJointNames[armName]]
        
        self.updateOpenraveJoints(armName, dict(self.defaultJoints), grasp=0)

        self.trajRequest[armName] = False


    def _ravenStateCallback(self, msg):
        self.currentState = msg
    
    def getCurrentPose(self, armName=None):
        if not self.currentState:
            return None
        currentPose = {}
        for arm in self.currentState.arms:
            armPose = tfx.pose(arm.tool.pose,header=self.currentState.header)
            if armName is None:
                currentPose[arm.name] = armPose
            elif arm.name == armName:
                currentPose = armPose
                break
        return currentPose
                    
    def getCurrentGrasp(self, armName = None):
        if not self.currentState:
            return None
        currentGrasp = {}
        for arm in self.currentState.arms:
            armGrasp = dict((joint.type, joint.position) for joint in arm.joints)[Constants.JOINT_TYPE_GRASP]
            if armName is None:
                currentGrasp[arm.name] = armGrasp
            elif arm.name == armName:
                currentGrasp = armGrasp
                break
        print currentGrasp
        return currentGrasp
                    
    def getCurrentJoints(self, armName=None):
        if not self.currentState:
            return None
        currentJoints = {}
        for arm in self.currentState.arms:
            armJoints = dict((joint.type, joint.position) for joint in arm.joints)
            if armName is None:
                currentJoints[arm.name] = armJoints
            elif arm.name == armName:
                currentJoints = armJoints
                break
        return currentJoints
    
    def waitForState(self):
        if not self.currentState:
            print 'waiting for ravenstate'
            while self.currentState is None and not rospy.is_shutdown():
                rospy.sleep(.05)
            print 'got it!'
    
    def getJointsFromPose(self, armName, pose, grasp, quiet=False):
        """
        Calls IK server and returns a dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Needs to return finger1 and finger2
        """

        pose = Util.convertToFrame(tfx.pose(pose), self.refFrame)
        
        try:
            rospy.loginfo('Waiting for IK server...')
            rospy.wait_for_service('inv_kin_server',timeout=5)
            inv_kin_service = rospy.ServiceProxy('inv_kin_server', InvKinSrv)
            rospy.loginfo('Calling the IK server')
            resp = inv_kin_service(self.invKinArm[armName], grasp, pose.msg.Pose())
            rospy.loginfo('IK success!')
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.loginfo("IK server failure: %s"%e)
            if quiet:
                return
            else:
                raise e

        return dict((joint.type, joint.position) for joint in resp.joints)

    def updateOpenraveJoints(self, armName, joints1=None, grasp=None):
        """
        Updates the openrave raven model using rosJoints

        rosJoints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Updates based on: shoulder, elbow, insertion,
                          rotation, pitch, yaw, grasp (self.currentGrasp)
        """
        rosJoints = joints1
        
        if rosJoints is None:
            rosJoints = self.getCurrentJoints(armName)
            if rosJoints is None:
                return
            if grasp is None:
                grasp = self.getCurrentGrasp(armName)
        if grasp is None:
            grasp = 0.

        raveJointTypes = []
        jointPositions = []
        for rosJointType, jointPos in rosJoints.items():
            if self.rosJointTypesToRave[armName].has_key(rosJointType):
                raveJointType = self.rosJointTypesToRave[armName][rosJointType]
                raveJointTypes.append(raveJointType)
                
                # since not a hard limit, must do this
                if rosJointType == Constants.JOINT_TYPE_ROTATION:
                    lim = self.robot.GetJointFromDOFIndex(raveJointType).GetLimits()
                    limLower, limUpper = lim[0][0], lim[1][0]
                    jointPos = Util.setWithinLimits(jointPos, limLower, limUpper, 2*pi)
                
                jointPositions.append(jointPos)
                

        # for opening the gripper
        raveJointTypes += self.raveGrasperJointTypes[armName]
        
        if armName == MyConstants.Arm.Left:
            jointPositions += [grasp/2, grasp/2]
        else:
            jointPositions += [grasp/2, -grasp/2]


        self.robot.SetJointValues(jointPositions, raveJointTypes)

    def jointTrajToDicts(self, armName, jointTrajArray, **kwargs):
        """
        Converts a numpy array trajectory
        to a list of joint dictionaries

        dicts contain ros joints1:
        shoulder, elbow, insertion,
        rotation, pitch, finger1, finger2
        """
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp'))
        endGrasp = kwargs.get('startGrasp',kwargs.get('grasp'))
        if startGrasp is None:
            startGrasp = self.getCurrentGrasp(armName)
        if endGrasp is None:
            endGrasp = startGrasp
        
        grasps = np.linspace(startGrasp, endGrasp, len(jointTrajArray))
        jointTrajDicts = []
        for trajIndex in range(len(jointTrajArray)):
            grasp = grasps[trajIndex]
            
            waypointJointPositions = list(jointTrajArray[trajIndex])
            
            # since yaw is pseudo-joint, convert to finger1 and finger2
            yaw = waypointJointPositions[-1]
            finger1 = yaw - grasp/2
            finger2 = -(yaw + grasp/2)
            if armName == MyConstants.Arm.Left:
                finger1 = -finger1
                finger2 = -finger2
            # try keeping yaw
            waypointJointPositions = waypointJointPositions[:] + [finger1, finger2]
            rosJointTypesPlusFingers = self.rosJointTypes[:] + [Constants.JOINT_TYPE_GRASP_FINGER1, Constants.JOINT_TYPE_GRASP_FINGER2]
            
            waypointDict = dict(zip(rosJointTypesPlusFingers, waypointJointPositions))
            jointTrajDicts.append(waypointDict)

        return jointTrajDicts
    
    def jointDictsToPoses(self, armName, jointTrajDicts):
        """
        Converts a list of joint trajectory dicts
        to a list of poses using openrave FK
        """
        currentDOFValues = self.robot.GetDOFValues()
        
        poseList = []
        for jointDict in jointTrajDicts:
            self.updateOpenraveJoints(armName, jointDict)
            
            poseMatInRef = np.eye(4)
            refLinkName = self.toolFrame[armName]
            targLinkName = MyConstants.Frames.Link0
            
            # ref -> world
            refFromWorld = self.robot.GetLink(refLinkName).GetTransform()
    
            # target -> world
            targFromWorld = self.robot.GetLink(targLinkName).GetTransform()
    
            # target -> ref
            targFromRef = np.dot(np.linalg.inv(targFromWorld), refFromWorld)
    
            poseMatInTarg = np.dot(targFromRef, poseMatInRef)
            pose = tfx.pose(poseMatInTarg, frame=targLinkName)
            
            poseList.append(pose)
            
        self.robot.SetDOFValues(currentDOFValues)
            
        return poseList
    
    def optimize1(self, n_steps, trajStartJoints, trajEndJoints):
        startJointPositions = []
        endJointPositions = []
        for armName in self.armNames:
            endJoints = trajEndJoints[armName]
            if endJoints is None:
                print trajEndJoints, self.trajRequest
            endJoints = dict([(jointType,jointPos) for jointType, jointPos in endJoints.items() if jointType in self.rosJointTypes])
            
            startJoints = trajStartJoints[armName]
            startJoints = dict([(jointType,jointPos) for jointType, jointPos in startJoints.items() if jointType in self.rosJointTypes])
            
            print 'start joints %s: %s' % (armName, [startJoints[k] for k in sorted(startJoints.keys())])
            print 'end joints %s: %s' % (armName, [endJoints[k] for k in sorted(endJoints.keys())])
            
            for raveJointType in self.manip[armName].GetArmIndices():
                rosJointType = self.raveJointTypesToRos[armName][raveJointType]
                endJointPositions.append(endJoints[rosJointType])
                startJointPositions.append(startJoints[rosJointType])
            
            self.updateOpenraveJoints(armName, trajStartJoints[armName])
                
        request = jointRequest(n_steps, endJointPositions)
    
        #IPython.embed()
    
        # convert dictionary into json-formatted string
        s = json.dumps(request)
        #print s
        #print self.robot.GetActiveDOFValues()
        #return
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.env)
        # do optimization
        result = trajoptpy.OptimizeProblem(prob)
        
        
    
        # check trajectory safety
        from trajoptpy.check_traj import traj_is_safe
        prob.SetRobotActiveDOFs()
        if not traj_is_safe(result.GetTraj(), self.robot):
            rospy.loginfo('Trajopt trajectory is not safe. Trajopt failed!')
            for armName in self.armNames:
                self.poseTraj[armName] = None
        else:
            startIndex = 0
            for armName in self.armNames:
                endIndex = startIndex + len(self.manipJoints[armName])
                
                graspKwargs = {}
                if self.trajStartGrasp[armName] is not None:
                    graspKwargs['startGrasp'] = self.trajStartGrasp[armName]
                if self.trajEndGrasp[armName] is not None:
                    graspKwargs['endGrasp'] = self.trajEndGrasp[armName]
                
                armJointTrajArray = result.GetTraj()[:,startIndex:endIndex]
                jointTrajDicts = self.jointTrajToDicts(armName, armJointTrajArray, **graspKwargs)
                self.jointTraj[armName] = jointTrajDicts # for debugging
                poseTraj = self.jointDictsToPoses(armName, jointTrajDicts)
                self.poseTraj[armName] = poseTraj
                
                startIndex = endIndex
    
    def optimize2(self, n_steps):
        return self.optimize1(n_steps, self.trajStartJoints, self.trajEndJoints)
    
    def optimize3(self):
        n_steps = max(self.trajSteps.values())
        return self.optimize2(n_steps)
    
    def optimizeLoop(self, once=False):
        while not rospy.is_shutdown():
            rospy.sleep(.05)
            
            # block until all traj submissions received
            while False in [self.trajRequest.get(armName,False) for armName in self.armNames]:
                if rospy.is_shutdown():
                    return
                rospy.sleep(0.05)
            
            print "it's go time"
            self.optimize3()
            
            with self.lock:
                for armName in self.armNames:
                    self.trajRequest[armName] = False
            
            if once:
                break
    
    def setStartAndEndJoints(self, armName, startJoints, endJoints, **kwargs):
        self.trajStartGrasp[armName] = kwargs.get('startGrasp',kwargs.get('grasp',None))
        self.trajEndGrasp[armName] = kwargs.get('endGrasp',kwargs.get('grasp',None))
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def setStartJointsAndEndPose(self, armName, startJoints, endPose,**kwargs):
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp',0))
        endGrasp = kwargs.get('endGrasp',kwargs.get('grasp',0))
        endJoints = self.getJointsFromPose(armName, endPose, grasp=endGrasp)
        
        self.trajStartGrasp[armName] = startGrasp
        self.trajEndGrasp[armName] = endGrasp
        endPose = Util.convertToFrame(tfx.pose(endPose), MyConstants.Frames.Link0)
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def setStartPoseAndEndJoints(self, armName, startPose, endJoints, **kwargs):
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp',0))
        endGrasp = kwargs.get('endGrasp',kwargs.get('grasp',0))
        startJoints = self.getJointsFromPose(armName, startPose, grasp=startGrasp)
        
        self.trajStartGrasp[armName] = startGrasp
        self.trajEndGrasp[armName] = endGrasp
        startPose = Util.convertToFrame(tfx.pose(startPose), MyConstants.Frames.Link0)
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def setStartAndEndPose(self, armName, startPose, endPose, **kwargs):
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp',0))
        endGrasp = kwargs.get('endGrasp',kwargs.get('grasp',0))
        startJoints = self.getJointsFromPose(armName, startPose, grasp=startGrasp)
        endJoints = self.getJointsFromPose(armName, endPose, grasp=endGrasp)
        
        self.trajStartGrasp[armName] = startGrasp
        self.trajEndGrasp[armName] = endGrasp
        startPose = Util.convertToFrame(tfx.pose(startPose), MyConstants.Frames.Link0)
        endPose = Util.convertToFrame(tfx.pose(endPose), MyConstants.Frames.Link0)
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def getTrajectoryJointsToPose(self, armName, endPose, startJoints=None, n_steps=50, debug=False, **kwargs):
        self.setStartJointsAndEndPose(armName, startJoints, endPose, **kwargs)
        self.trajSteps[armName] = n_steps
        with self.lock:
            self.trajRequest[armName] = True
        if self.trajEndJoints[armName] is None:
            print armName, endPose, startJoints, kwargs
            raise Exception()
    
    def getTrajectoryPoseToPose(self, armName, startPose, endPose, n_steps=50, **kwargs):
        self.setStartAndEndPose(armName, startPose, endPose, **kwargs)
        self.trajSteps[armName] = n_steps
        with self.lock:
            self.trajRequest[armName] = True
        if self.trajEndJoints[armName] is None:
            print armName, startPose, endPose, kwargs
            raise Exception()
    
    def getTrajectoryFromPose(self, armName, endPose, endGrasp = None, n_steps=50, block=True):
        self.waitForState()
        joints1 = self.getCurrentJoints(armName)
        startPose = self.getCurrentPose(armName)
        startGrasp = self.getCurrentGrasp(armName)
        if endGrasp is None:
            endGrasp = startGrasp
        self.setStartAndEndPose(armName, startPose, endPose, startGrasp=startGrasp, endGrasp=endGrasp)
        self.trajSteps[armName] = n_steps
        with self.lock:
            self.trajRequest[armName] = True
        if self.trajEndJoints[armName] is None:
            print armName, startPose, startGrasp, endPose, endGrasp
            raise Exception()
        
        self.start_pose_pubs[armName].publish(startPose.msg.PoseStamped())
        self.end_pose_pubs[armName].publish(endPose.msg.PoseStamped())
        
        if block:
            print 'waiting for traj'
            while self.trajRequest[armName] and not rospy.is_shutdown():
                rospy.sleep(0.05)
                
            return self.poseTraj[armName]
        
    getPoseTrajectory = getTrajectoryFromPose
    
    def trajReady(self):
        return not any(self.trajRequest.values())
    
    def waitForTrajReady(self):
        while not self.trajReady() and not rospy.is_shutdown():
            rospy.sleep(.05)

def testSwitchPlaces(show=True):
    #trajoptpy.SetInteractive(True)
    
    rospy.init_node('testSwitchPlaces',anonymous=True)
    rp = RavenPlanner([MyConstants.Arm.Left,MyConstants.Arm.Right], thread=True)
    
    #rightCurrPose = tfx.pose([-0.068,-0.061,-0.129],tfx.tb_angles(-139.6,88.5,111.8),frame=MyConstants.Frames.Link0)
    #leftCurrPose = tfx.pose([-.072,-.015,-.153],tfx.tb_angles(43.9,78.6,100.9),frame=MyConstants.Frames.Link0)
    
    leftCurrPose = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[MyConstants.Arm.Left]),MyConstants.Frames.Link0)
    rightCurrPose = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[MyConstants.Arm.Right]),MyConstants.Frames.Link0)

    #rp.getTrajectoryPoseToPose(MyConstants.Arm.Left, leftCurrPose, rightCurrPose, n_steps=50)
    #rp.getTrajectoryPoseToPose(MyConstants.Arm.Right, rightCurrPose, leftCurrPose, n_steps=50)
    
    #rp.getPoseTrajectory(MyConstants.Arm.Left, rightCurrPose, n_steps=50)
    #rp.getPoseTrajectory(MyConstants.Arm.Right, leftCurrPose, n_steps=50)
    rp.getPoseTrajectory(MyConstants.Arm.Left, leftCurrPose+[0.01,0,0], n_steps=50)
    rp.getPoseTrajectory(MyConstants.Arm.Right, rightCurrPose-[0.01,0,0], n_steps=50)
    
    #IPython.embed()
    
    print 'waiting'
    rp.waitForTrajReady()
    print 'woooooooo'
    
    if rospy.is_shutdown():
        return
    
    #IPython.embed()
    
    if not show:
        return
    rp.env.SetViewer('qtcoin')
    
    leftPoseTraj = rp.poseTraj[MyConstants.Arm.Left]
    rightPoseTraj = rp.poseTraj[MyConstants.Arm.Right]
    
    for left, right in zip(leftPoseTraj,rightPoseTraj):
        if rospy.is_shutdown():
            break
        rp.updateOpenraveJoints('L', rp.getJointsFromPose('L', left, rp.getCurrentGrasp('L')), grasp=rp.getCurrentGrasp('L'))
        rp.updateOpenraveJoints('R', rp.getJointsFromPose('R', right, rp.getCurrentGrasp('R')), grasp=rp.getCurrentGrasp('R'))
        rospy.loginfo('Press enter to go to next step')
        raw_input()
        
    return
    
    leftTraj = rp.jointTraj[MyConstants.Arm.Left]
    rightTraj = rp.jointTraj[MyConstants.Arm.Right]
    
    for left, right in zip(leftTraj,rightTraj):
        if rospy.is_shutdown():
            break
        rp.updateOpenraveJoints('L', left)
        rp.updateOpenraveJoints('R', right)
        rospy.loginfo('Press enter to go to next step')
        raw_input()
    
    #IPython.embed()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--show',action='store_true',default=False)
    
    args = parser.parse_args()
    testSwitchPlaces(**vars(args))
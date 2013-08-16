#!/usr/bin/env python


import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from math import *
import copy
import sys, os

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *

from RavenDebridement.srv import InvKinSrv


import openravepy as rave
import trajoptpy
import json
import numpy as np

import threading

import tf
import tfx

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants

import code
import IPython

class Request():
    """
    Class for the trajopt json requests
    """
    class Type:
        Joints = 0
        Pose = 1

    @staticmethod
    def pose(n_steps, endJointPositions,  toolFrames, endPoses):
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
                #{
                #    "type" : "collision",
                #    "params" : {
                #        "coeffs" : [100000],
                #        "continuous" : False,
                #        "dist_pen" : [0.003]
                #        }
                #    },
                ],
            "constraints" : [
                ],
            "init_info" : {
                "type" : "straight_line",
                "endpoint" : endJointPositions
                }
            }

        for toolFrame, endPose in zip(toolFrames, endPoses):
            pose = tfx.pose(endPose)
        
            desPos = pose.position.list
            desQuat = pose.orientation
            wxyzQuat = [desQuat.w, desQuat.x, desQuat.y, desQuat.z]
            
            # cost or constraint?
            request["constraints"].append({
                        "type": "pose",
                        "name" : "target_pose",
                        "params" : {"xyz" : desPos,
                                    "wxyz" : wxyzQuat,
                                    "link" : toolFrame,
                                    "rot_coeffs" : [5,5,5],
                                    "pos_coeffs" : [100,100,100]
                                    }
                        })
            
            
            for timestep in range(int(n_steps)):
                request["costs"].append({
                        "type" : "pose",
                        "params" : {"xyz"  : [0,0,0],
                                    "wxyz" : wxyzQuat,
                                    "link" : toolFrame,
                                    "timestep" : timestep,
                                    "rot_coeffs" : [1,1,1],
                                    "pos_coeffs" : [0,0,0]
                                    }
                        })
            
            
        return request

    @staticmethod
    def joints(n_steps, manipName, endJointPositions):
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

class RavenPlanner():

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
    
    defaultJointPositions = [.512, 1.6, -5, .116, .088, 0]
    defaultJoints = dict([(jointType,jointPos) for jointType, jointPos in zip(rosJointTypes,defaultJointPositions)])

    def __init__(self, armNames=[MyConstants.Arm.Left]):
        """
        openrave planner for the raven
        """
        rospy.loginfo('Initializing openrave planner for {0} raven arm'.format(armNames))

        armNames.sort()
        self.armNames = armNames
        

        self.currentState = None

        self.refFrame = MyConstants.Frames.Link0

        

        self.env = rave.Environment()

        rospy.loginfo('Before loading model')
        ravenFile = os.path.dirname(__file__) + '/../../../models/myRaven.xml'
        #ravenFile = '/home/gkahn/ros_workspace/RavenDebridement/models/myRaven.xml'
        self.env.Load(ravenFile)
        rospy.loginfo('After loading model')

        #trajoptpy.SetInteractive(True)

        self.robot = self.env.GetRobots()[0]
        
        self.currentGrasp = dict()
        self.currentJoints = dict()
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
        self.trajEndPose = dict()
        self.trajEndJoints = dict()
        self.trajStartJoints = dict()
        self.trajReqType = None
        self.trajSteps = dict()
        self.jointTraj = dict() # for debugging
        self.poseTraj = dict()
        
        activeDOFs = []
        for armName in self.armNames:
            self._init_arm(armName)
            activeDOFs += self.raveJointTypes[armName]
            
        self.robot.SetActiveDOFs(activeDOFs)
        
        rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)
        
        while self.currentState == None and not rospy.is_shutdown():
            rospy.sleep(.05)
            
        self.thread = threading.Thread(target=self.getTrajectoryFromPoseThread)
        self.thread.setDaemon(True)
        self.thread.start()
        

        
        rospy.loginfo('Exiting init')

    def _init_arm(self, armName):
        self.currentGrasp[armName] = 0

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
        
        self.currentJoints[armName] = dict(self.defaultJoints)
        self.updateOpenraveJoints(armName, self.currentJoints[armName])

        self.trajRequest[armName] = False


    def _ravenStateCallback(self, msg):
        self.currentState = msg
        for arm in msg.arms:
            if arm.name in self.armNames:
                self.currentJoints[arm.name] = dict((joint.type, joint.position) for joint in arm.joints)
                    
                if self.currentJoints[arm.name].has_key(Constants.JOINT_TYPE_GRASP):
                    self.currentGrasp[arm.name] = self.currentJoints[arm.name][Constants.JOINT_TYPE_GRASP]
                
    ######################
    # Openrave methods   #
    ######################

    def updateOpenraveJoints(self, armName, rosJoints=None):
        """
        Updates the openrave raven model using rosJoints

        rosJoints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Updates based on: shoulder, elbow, insertion,
                          rotation, pitch, yaw, grasp (self.currentGrasp)
        """
        if rosJoints == None:
            rosJoints = self.currentJoints[armName]
            if rosJoints == None:
                return

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
        currentGrasp = self.currentGrasp[armName]
        if armName == MyConstants.Arm.Left:
            jointPositions += [currentGrasp/2, currentGrasp/2]
        else:
            jointPositions += [currentGrasp/2, -currentGrasp/2]

        self.robot.SetJointValues(jointPositions, raveJointTypes)

        


    #####################################
    # Conversion/Miscellaneous methods  #
    #####################################

    def transformRelativePoseForIk(self, armName, poseMatrix, refLinkName, targLinkName):
        """
        Adapted from PR2.py

        Returns transformed poseMatrix that can be used in openrave IK
        """
        # world <- ref
        refLink = self.robot.GetLink(refLinkName)
        worldFromRefLink = refLink.GetTransform()

        # world <- targ
        targLink = self.robot.GetLink(targLinkName)
        worldFromTargLink = targLink.GetTransform()

        # targ <- EE
        worldFromEE = self.manip[armName].GetEndEffectorTransform()
        targLinkFromEE = np.dot(np.linalg.inv(worldFromTargLink), worldFromEE)
        
        # world <- EE
        refLinkFromTargLink = poseMatrix
        newWorldFromEE = np.dot(np.dot(worldFromRefLink, refLinkFromTargLink), targLinkFromEE)

        return newWorldFromEE

    def jointTrajToDicts(self, armName, jointTrajArray):
        """
        Converts a numpy array trajectory
        to a list of joint dictionaries

        dicts contain ros joints:
        shoulder, elbow, insertion,
        rotation, pitch, finger1, finger2
        """
        grasp = self.currentGrasp[armName]
        jointTrajDicts = []
        for trajIndex in range(len(jointTrajArray)):
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
        

    #################################
    # IK and Trajectory planning    #
    #################################

    def getJointsFromPose(self, armName, endPose):
        """
        Calls IK server and returns a dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Needs to return finger1 and finger2
        """

        endPose = Util.convertToFrame(tfx.pose(endPose), self.refFrame)

        try:
            rospy.loginfo('Waiting for IK server...')
            rospy.wait_for_service('inv_kin_server',timeout=5)
            inv_kin_service = rospy.ServiceProxy('inv_kin_server', InvKinSrv)
            rospy.loginfo('Calling the IK server')
            resp = inv_kin_service(self.invKinArm[armName], self.currentGrasp[armName], endPose.msg.Pose())
            rospy.loginfo('IK success!')
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.loginfo("IK server failure: %s"%e)
            return

        return dict((joint.type, joint.position) for joint in resp.joints)
    
    def getTrajectoryFromPose(self, armName, endPose, startJoints=None, reqType=Request.Type.Pose, n_steps=50, debug=False):
        endPose = Util.convertToFrame(tfx.pose(endPose), MyConstants.Frames.Link0)
        self.trajEndPose[armName] = endPose
        
        self.trajEndJoints[armName] = self.getJointsFromPose(armName, endPose)
        if self.trajEndJoints[armName] == None:
            return None
        
        if startJoints == None:
            self.trajStartJoints[armName] = self.currentJoints[armName]
        else:
            self.trajStartJoints[armName] = startJoints
            
        self.trajSteps[armName] = n_steps
        self.trajReqType = reqType
            
        self.trajRequest[armName] = True
        
        if debug:
            return
        
        while self.trajRequest[armName] and not rospy.is_shutdown():
            rospy.sleep(.05)
            
        return self.poseTraj[armName]

    def getTrajectoryFromPoseThread(self):
        """
        Thread that uses trajopt to compute trajectory
        
        Thread waits for all values in self.trajRequest
        to be true

        Sets the values of all the keys in self.poseTraj
        """
        while not rospy.is_shutdown():
            rospy.sleep(.05)
            
            # block until all traj submissions received
            while False in self.trajRequest.values():
                if rospy.is_shutdown():
                    return
                rospy.sleep(.05)
            
            n_steps = 0
            toolFrames = []
            endPoses = []
            endJointPositions = []
            for armName in self.armNames:
                n_steps += float(self.trajSteps[armName])/len(self.armNames)
    
                toolFrames.append(self.toolFrame[armName])
                
                endPose = self.trajEndPose[armName]
                worldFromEE = tfx.pose(self.transformRelativePoseForIk(armName, endPose.matrix, endPose.frame, self.toolFrame[armName]))
                endPoses.append(worldFromEE)
                
                endJoints = self.trajEndJoints[armName]
                # only keep shoulder, elbow, insertion, rotation, pitch, yaw
                endJoints = dict([(jointType,jointPos) for jointType, jointPos in endJoints.items() if jointType in self.rosJointTypes])
                
                for raveJointType in self.manip[armName].GetArmIndices():
                    rosJointType = self.raveJointTypesToRos[armName][raveJointType]
                    endJointPositions.append(endJoints[rosJointType])
                    
                self.updateOpenraveJoints(armName, self.trajStartJoints[armName])
                
    
            if self.trajReqType == Request.Type.Joints:
                request = Request.joints(n_steps, self.manip.GetName(), endJointPositions)
            else:
                request = Request.pose(n_steps, endJointPositions, toolFrames, endPoses)
                                       
    
            # convert dictionary into json-formatted string
            s = json.dumps(request)
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
                    
                    armJointTrajArray = result.GetTraj()[:,startIndex:endIndex]
                    jointTrajDicts = self.jointTrajToDicts(armName, armJointTrajArray)
                    self.jointTraj[armName] = jointTrajDicts # for debugging
                    poseTraj = self.jointDictsToPoses(armName, jointTrajDicts)
                    self.poseTraj[armName] = poseTraj
                    
                    startIndex = endIndex
                    
            for armName in self.armNames:
                self.trajRequest[armName] = False






def testGeneral(armNames=[MyConstants.Arm.Left, MyConstants.Arm.Right]):
    rospy.init_node('test_RavenPlanner',anonymous=True)
    rp = RavenPlanner(armNames)
    rospy.sleep(2)
    
    
    rp.updateOpenraveJoints('R')
    rp.updateOpenraveJoints('L')
    
    currRight = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame['R']), MyConstants.Frames.Link0)
    currLeft = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame['L']), MyConstants.Frames.Link0)
    
    raveRightMat = Util.openraveTransformFromTo(rp.robot, np.eye(4), rp.toolFrame['R'], MyConstants.Frames.Link0)
    raveRight = tfx.pose(raveRightMat,frame=MyConstants.Frames.Link0)
    
    raveLeftMat = Util.openraveTransformFromTo(rp.robot, np.eye(4), rp.toolFrame['L'], MyConstants.Frames.Link0)
    raveLeft = tfx.pose(raveLeftMat,frame=MyConstants.Frames.Link0)
    
    print('currRight')
    print(currRight)
    print('raveRight')
    print(raveRight)
    
    print('currLeft')
    print(currLeft)
    print('raveLeft')
    print(raveLeft)
    
    
def testSwitchPlaces(armNames=[MyConstants.Arm.Left,MyConstants.Arm.Right]):
    rospy.init_node('testSwitchPlaces',anonymous=True)
    rp = RavenPlanner(armNames)
    rospy.sleep(2)
    
    for index in range(len(armNames)):
        armName = armNames[index]
        otherArmName = armNames[(index+1)%len(armNames)]
        desPose = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[otherArmName]),MyConstants.Frames.Link0)
        
        rp.getTrajectoryFromPose(armName, desPose, debug=True)
        
    while rp.trajRequest[armNames[0]] and rp.trajRequest[armNames[1]] and not rospy.is_shutdown():
        rospy.sleep(.05)
        
    leftTraj = rp.jointTraj[MyConstants.Arm.Left]
    rightTraj = rp.jointTraj[MyConstants.Arm.Right]
    
    rp.env.SetViewer('qtcoin')
    
    for left, right in zip(leftTraj,rightTraj):
        rospy.loginfo('Press enter to go to next step')
        raw_input()
        rp.updateOpenraveJoints('L', left)
        rp.updateOpenraveJoints('R', right)
    
    IPython.embed()

    
        
    
def testGetTrajectoryFromPose(armNames=[MyConstants.Arm.Left]):
    rospy.init_node('test_RavenPlanner',anonymous=True)
    rp = RavenPlanner(armNames)
    rospy.sleep(2)
    
    desPose = tfx.pose([0,0,0], frame=rp.toolFrame[armNames[0]])
    tf_tool_to_link0 = tfx.lookupTransform(MyConstants.Frames.Link0, desPose.frame, wait=5)
    desPose = tf_tool_to_link0 * desPose
    
    desPose.position.x += .03
    
    poseTraj = rp.getTrajectoryFromPose(armNames[0], desPose)
    
    IPython.embed()




def testJointDictsToPoses(arm=MyConstants.Arm.Left):
    rospy.init_node('test_IK',anonymous=True)
    rp = RavenPlanner(arm)
    rospy.sleep(2)
    
    currentJoints = rp.currentJoints
    currPose = rp.jointDictsToPoses([currentJoints])[0]
    IPython.embed()

if __name__ == '__main__':
    #testGeneral()
    testSwitchPlaces()
    #testGetTrajectoryFromPose()
    #testJointDictsToPoses()



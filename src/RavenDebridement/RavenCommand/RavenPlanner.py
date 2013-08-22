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
                "n_steps" : int(n_steps),
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
            
            """
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
            """
            
        return request

    @staticmethod
    def joints(n_steps, endJointPositions):
        request = {
            "basic_info" : {
                "n_steps" : int(n_steps),
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
        self._currentGrasp = dict()
        self._currentJoints = dict()
        
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
            
        self.thread = threading.Thread(target=self.getTrajectoryFromPoseThread)
        self.thread.setDaemon(True)
        #self.thread.start()
        

        
        rospy.loginfo('Exiting init')

    @property
    def currentGrasp(self):
        if self._currentGrasp:
            return self._currentGrasp
        return self._currentGrasp
    @currentGrasp.setter
    def currentGrasp(self, value):
        self._currentGrasp = value

    @property
    def currentJoints(self):
        self.waitForState()
        return self._currentJoints
    @currentGrasp.setter
    def currentJoints(self, value):
        self._currentJoints = value
    
    def waitForState(self):
        if not self.currentState:
            print 'waiting for ravenstate'
            while self.currentState is None and not rospy.is_shutdown():
                rospy.sleep(.05)
            print 'got it!'

    def _init_arm(self, armName):
        self._currentGrasp[armName] = 0

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
        
        self.updateOpenraveJoints(armName, dict(self.defaultJoints))

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

    def updateOpenraveJoints(self, armName, rosJoints=None, grasp=None):
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
        currentGrasp = grasp if grasp is not None else self.currentGrasp[armName]
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

    def getJointsFromPose(self, armName, endPose, grasp=None):
        """
        Calls IK server and returns a dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Needs to return finger1 and finger2
        """

        endPose = Util.convertToFrame(tfx.pose(endPose), self.refFrame)
        
        grasp = grasp if grasp is not None else self.currentGrasp[armName]

        try:
            rospy.loginfo('Waiting for IK server...')
            rospy.wait_for_service('inv_kin_server',timeout=5)
            inv_kin_service = rospy.ServiceProxy('inv_kin_server', InvKinSrv)
            rospy.loginfo('Calling the IK server')
            resp = inv_kin_service(self.invKinArm[armName], grasp, endPose.msg.Pose())
            rospy.loginfo('IK success!')
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.loginfo("IK server failure: %s"%e)
            return

        return dict((joint.type, joint.position) for joint in resp.joints)
    
    def getTrajectoryFromPose(self, armName, endPose, endGrasp=None, startJoints=None, reqType=Request.Type.Joints, n_steps=50, debug=False):
        endPose = Util.convertToFrame(tfx.pose(endPose), MyConstants.Frames.Link0)
        self.trajEndPose[armName] = endPose
        
        self.trajEndJoints[armName] = self.getJointsFromPose(armName, endPose, grasp=endGrasp)
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

    def getTrajectoryFromPoseThread(self, once=False):
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
            
            n_steps = int(n_steps)
    
            if self.trajReqType == Request.Type.Joints:
                request = Request.joints(n_steps, endJointPositions)
            else:
                request = Request.pose(n_steps, endJointPositions, toolFrames, endPoses)

            #IPython.embed()
                
            # convert dictionary into json-formatted string
            s = json.dumps(request)
            
            #print s
            #print self.robot.GetActiveDOFValues()
            #raise Exception()
            # create object that stores optimization problem
            prob = trajoptpy.ConstructProblem(s, self.env)
            #raise Exception()
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
            
            #print self.jointTraj
            #raise Exception()
            
            for armName in self.armNames:
                self.trajRequest[armName] = False
            
            if once:
                break






def testSwitchPlaces(armNames=[MyConstants.Arm.Left,MyConstants.Arm.Right],fixedPose=False):
    rospy.init_node('testSwitchPlaces',anonymous=True)
    rp = RavenPlanner(armNames)
    rospy.sleep(2)
    
    leftStartJoints = None
    rightStartJoints = None
    
    if fixedPose:
        rightCurrPose = tfx.pose([-0.068,-0.061,-0.129],tfx.tb_angles(-139.6,88.5,111.8),frame=MyConstants.Frames.Link0)
        
        leftCurrPose = tfx.pose([-.072,-.015,-.153],tfx.tb_angles(43.9,78.6,100.9),frame=MyConstants.Frames.Link0)
    
        leftStartJoints = rp.getJointsFromPose(MyConstants.Arm.Left, leftCurrPose, grasp=0)
        rightStartJoints = rp.getJointsFromPose(MyConstants.Arm.Right, rightCurrPose, grasp=0)
    else:
        leftCurrPose = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[MyConstants.Arm.Left]),MyConstants.Frames.Link0)
        rightCurrPose = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[MyConstants.Arm.Right]),MyConstants.Frames.Link0)
        
    
    #trajoptpy.SetInteractive(True)
    
    if fixedPose:
        rp.getTrajectoryFromPose(MyConstants.Arm.Left, rightCurrPose, endGrasp=0, startJoints=leftStartJoints, debug=True)
        rp.getTrajectoryFromPose(MyConstants.Arm.Right, leftCurrPose, endGrasp=0, startJoints=rightStartJoints, debug=True)
    else:
        rp.getTrajectoryFromPose(MyConstants.Arm.Left, rightCurrPose, leftStartJoints, debug=True)
        rp.getTrajectoryFromPose(MyConstants.Arm.Right, leftCurrPose, rightStartJoints, debug=True)
    
    """
    for index in range(len(armNames)):
        armName = armNames[index]
        otherArmName = armNames[(index+1)%len(armNames)]
        desPose = Util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[otherArmName]),MyConstants.Frames.Link0)
        
        rp.getTrajectoryFromPose(armName, desPose, debug=True)
    """
    
    rp.getTrajectoryFromPoseThread(once=True)
#     while rp.trajRequest[armNames[0]] and rp.trajRequest[armNames[1]] and not rospy.is_shutdown():
#         rospy.sleep(.05)
        
    leftTraj = rp.jointTraj[MyConstants.Arm.Left]
    rightTraj = rp.jointTraj[MyConstants.Arm.Right]
    
    rp.env.SetViewer('qtcoin')
    
    for left, right in zip(leftTraj,rightTraj):
        rospy.loginfo('Press enter to go to next step')
        raw_input()
        rp.updateOpenraveJoints('L', left)
        rp.updateOpenraveJoints('R', right)
    
    #IPython.embed()

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--fixed-pose',action='store_true',default=False)
    
    args = parser.parse_args()
    
    testSwitchPlaces(fixedPose=args.fixed_pose)

"""
Pose:         (-0.063, -0.062, -0.126)
              [yaw:16.0, pitch:55.1, roll:-93.0]
Pose command: (-0.063, -0.062, -0.126)
              [yaw:16.0, pitch:55.1, roll:-93.0]
Grasp:         0.879
Grasp command: 0.879

------------------------------JOINTS------------------------------
            SHOULDE  ELBOW  INSERTI ROTATIO  PITCH  FINGER1 FINGER2   YAW    GRASP 
state     :   READY   READY   READY   READY   READY   READY   READY   READY   READY
jpos      :   0.817   2.055 -14.692   3.197   0.613  -0.617  -0.262  -0.177   0.879
jvel      :   0.000   0.000  -0.000   0.000   0.000   0.000   0.000  -0.000   0.000
jcmd_type :     NON     NON     NON     NON     NON     NON     NON     NON     NON
jcmd      :     0.0     0.0     0.0     0.0     0.0     0.0     0.0     0.0     0.0
jpos_d    :   0.823   2.047 -14.692   3.022   0.613  -0.617  -0.262  -0.177   0.879
jvel_d    :  -0.001  -0.001  -0.006   0.000   0.000   0.000   0.000  -0.000   0.000

------------------------------MOTORS------------------------------
            SHOULDE  ELBOW  INSERTI ROTATIO  PITCH  FINGER1 FINGER2
mpos      :   103.2   245.0 -26890.0   -74.7   -86.9   -97.3   -94.2
mvel      :     0.0     0.0    -0.8    -0.0    -0.0    -0.0    -0.0
torque    :  -0.060  -0.057   0.008   0.000   0.000   0.000   0.000
mpos_d    :   104.0   244.2 -26893.4   -75.7   -86.9   -97.3   -94.2
mvel_d    :     0.0     0.0     0.0     0.0     0.0     0.0     0.0

****Arm L [GOLD]****

Pose:         (-0.071, -0.000, -0.153)
              [yaw:55.3, pitch:74.7, roll:113.3]
Pose command: (-0.071, -0.000, -0.153)
              [yaw:55.3, pitch:74.7, roll:113.3]
Grasp:         1.045
Grasp command: 1.045

------------------------------JOINTS------------------------------
            SHOULDE  ELBOW  INSERTI ROTATIO  PITCH  FINGER1 FINGER2   YAW    GRASP 
state     :   READY   READY   READY   READY   READY   READY   READY   READY   READY
jpos      :   0.255   1.969 -14.914   6.877  -0.148  -0.124   1.169   0.647   1.045
jvel      :  -0.000  -0.000   0.000   0.000   0.000   0.000   0.000   0.000   0.000
jcmd_type :     NON     NON     NON     NON     NON     NON     NON     NON     NON
jcmd      :     0.0     0.0     0.0     0.0     0.0     0.0     0.0     0.0     0.0
jpos_d    :   0.258   1.972 -14.914   6.703  -0.148  -0.124   1.169   0.647   1.045
jvel_d    :   0.000  -0.001   0.004   0.000   0.000   0.000   0.000   0.000   0.000

------------------------------MOTORS------------------------------
            SHOULDE  ELBOW  INSERTI ROTATIO  PITCH  FINGER1 FINGER2
mpos      :    32.2   226.1 -28479.3   -60.5   -98.4   -98.4   -86.9
mvel      :    -0.0    -0.0     0.5    -0.0    -0.0    -0.0    -0.0
torque    :   0.058  -0.070   0.010   0.000   0.000   0.000   0.000
mpos_d    :    32.6   226.4 -28471.1   -61.4   -98.4   -98.3   -86.9
mvel_d    :     0.0     0.0     0.0     0.0     0.0     0.0     0.0

"""

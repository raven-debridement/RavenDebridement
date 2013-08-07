#!/usr/bin/env python


import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from math import *
import copy
import sys, os

import openravepy as rave

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *

from RavenDebridement.srv import InvKinSrv


import openravepy as rave
import trajoptpy
import json
#import trajoptpy.kin_utils as ku
import numpy as np


import tf
import tfx

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants

import code

class Request():
    """
    Class for the trajopt json requests
    """
    
    def __init__(self, n_steps=20):
        self.n_steps = n_steps
        

    def straightLine(self, manipName, toolFrame, pose, endJointPositions):
        pose = tfx.pose(pose)
        
        desPos = pose.position.list
        desQuat = pose.orientation
        wxyzQuat = [desQuat.w, desQuat.x, desQuat.y, desQuat.z]

        request = {
            "basic_info" : {
                "n_steps" : self.n_steps,
                "manip" : manipName,
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

    """
    Joint order:
    shoulder
    elbow
    insertion
    tool_roll
    wrist_joint
    grasper_joint_1
    grasper_joint_2
    """
    rosJointTypes = [Constants.JOINT_TYPE_SHOULDER,
                     Constants.JOINT_TYPE_ELBOW,
                     Constants.JOINT_TYPE_INSERTION,
                     Constants.JOINT_TYPE_ROTATION,
                     Constants.JOINT_TYPE_PITCH,
                     Constants.JOINT_TYPE_YAW]

    raveJointNames = ["shoulder",
                      "elbow",
                      "insertion",
                      "tool_roll",
                      "wrist_joint",
                      "grasper_yaw"]

    def __init__(self, armName=MyConstants.Arm.Left):
        """
        openrave planner for the raven
        """
        rospy.loginfo('Initializing openrave planner for ' + armName + ' raven arm')

        self.armName = armName
        

        self.currentState = None
        self.currentGrasp = 0
        
        self.jointPositions = [0 for _ in range(len(self.rosJointTypes))]
        self.jointTypes = list(self.rosJointTypes)

        self.refFrame = MyConstants.Frames.Link0

        rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)


        self.env = rave.Environment()

        # TEMP
        #self.env.SetViewer('qtcoin')

        ravenFile = os.path.dirname(__file__) + '/../../../models/myRaven.xml'
        self.env.Load(ravenFile)


        # TEMP
        #trajoptpy.SetInteractive(True)


        self.robot = self.env.GetRobots()[0]
        if self.armName == MyConstants.Arm.Left:
            self.invKinArm = Constants.ARM_TYPE_GOLD
            self.toolFrame = MyConstants.OpenraveLinks.LeftToolBase
            self.robot.SetActiveManipulator('left_arm')
        else:
            self.invKinArm = Constants.ARM_TYPE_GREEN
            self.toolFrame = MyConstants.OpenraveLinks.RightToolBase
            self.robot.SetActiveManipulator('right_arm')
        self.manip = self.robot.GetActiveManipulator()
        self.manipJoints = self.robot.GetJoints(self.manip.GetArmJoints())

        #rospy.loginfo('Loading model')
        #ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=rave.IkParameterization.Type.Transform6D)
        #if not ikmodel.load():
        #    ikmodel.autogenerate()
        #rospy.loginfo('Model loaded')


        self.raveJointNames = ['{0}_{1}'.format(name, self.armName[0].upper()) for name in self.raveJointNames]

        self.raveJointTypes = [self.robot.GetJointIndex(name) for name in self.raveJointNames]
        self.raveJointTypesToRos = dict((rave,ros) for rave,ros in zip(self.raveJointTypes, self.rosJointTypes))
        self.rosJointTypesToRave = dict((ros,rave) for ros,rave in zip(self.rosJointTypes, self.raveJointTypes))
        
        self.raveGrasperJointNames = ['grasper_joint_1_{0}'.format(self.armName[0].upper()), 'grasper_joint_2_{0}'.format(self.armName[0].upper())]
        self.raveGrasperJointTypes = [self.robot.GetJointIndex(name) for name in self.raveGrasperJointNames]

        

    def _ravenStateCallback(self, msg):
        self.currentState = msg
        for arm in msg.arms:
            if arm.name == self.armName:
                self.currentJoints = dict((joint.type, joint.position) for joint in arm.joints)

                if self.currentJoints.has_key(Constants.JOINT_TYPE_GRASP):
                    self.currentGrasp = self.currentJoints[Constants.JOINT_TYPE_GRASP]
                
    ######################
    # Openrave methods   #
    ######################

    def updateOpenraveJoints(self, rosJoints=None):
        """
        Updates the openrave raven model using rosJoints

        rosJoints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Updates based on: shoulder, elbow, insertion,
                          rotation, pitch, yaw, grasp (self.currentGrasp)
        """
        if rosJoints == None:
            rosJoints = self.currentJoints
            if rosJoints == None:
                return

        raveJointTypes = []
        jointPositions = []
        for rosJointType, jointPos in rosJoints.items():
            if self.rosJointTypesToRave.has_key(rosJointType):
                raveJointTypes.append(self.rosJointTypesToRave[rosJointType])
                jointPositions.append(jointPos)

        # for opening the gripper
        # NOTE: this is for the right arm, may be diff for left arm
        raveJointTypes += self.raveGrasperJointTypes
        #currentGrasp = self.currentGrasp if self.armName == MyConstants.Arm.Left else -self.currentGrasp
        jointPositions += [self.currentGrasp/2, -self.currentGrasp/2]

        self.robot.SetJointValues(jointPositions, raveJointTypes)

        


    #####################################
    # Conversion/Miscellaneous methods  #
    #####################################

    def transformRelativePoseForIk(poseMatrix, refLinkName, targLinkName):
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
        worldFromEE = self.manip.GetEndEffectorTransform()
        targLinkFromEE = np.dot(np.linalg.inv(worldFromTargLink), worldFromEE)

        # world <- EE
        refLinkFromTargLink = poseMatrix
        newWorldFromEE = np.dot(np.dot(worldFromRefLink, refLinkFromTargLink), targLinkFromEE)

        return newWorldFromEE

    def trajoptTrajToDicts(self, trajoptTraj):
        """
        Converts a trajopt numpy array trajectory
        to a list of joint dictionaries

        dicts contain ros joints:
        shoulder, elbow, insertion,
        rotation, pitch, finger1, finger2
        """
        grasp = self.currentGrasp
        jointTraj = []
        for trajIndex in range(len(trajoptTraj)):
            waypointJointPositions = list(trajoptTraj[trajIndex])
            
            # since yaw is pseudo-joint, convert to finger1 and finger2
            yaw = waypointJointPositions[-1]
            finger1 = yaw - grasp/2
            finger2 = -(yaw + grasp/2)
            waypointJointPositions = waypointJointPositions[:-1] + [finger1, finger2]
            rosJointTypesPlusFingers = self.rosJointTypes[:-1] + [Constants.JOINT_TYPE_GRASP_FINGER1, Constants.JOINT_TYPE_GRASP_FINGER2]
            
            waypointDict = dict(zip(rosJointTypesPlusFingers, waypointJointPositions))
            jointTraj.append(waypointDict)

        return jointTraj

    #################################
    # IK and Trajectory planning    #
    #################################

    def getJointsFromPose(self, endPose):
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
            resp = inv_kin_service(self.invKinArm, self.currentGrasp, endPose.msg.Pose())
            rospy.loginfo('IK success!')
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.loginfo("IK server failure: %s"%e)
            return

        return dict((joint.type, joint.position) for joint in resp.joints)

    def getTrajectoryFromPose(self, startJoints, endPose, reqFunc=Request(40).straightLine):
        """
        Use trajopt to compute trajectory

        Returns a list of joint dictionaries

        """
        self.updateOpenraveJoints(startJoints)

        endJoints = self.getJointsFromPose(endPose)

        if endJoints == None:
            rospy.loginfo('IK failed!')
            return

        # only keep shoulder, elbow, insertion, rotation, pitch, yaw
        endJoints = dict([(jointType,jointPos) for jointType, jointPos in endJoints.items() if jointType in self.rosJointTypes])

        transEndPose = Util.convertToFrame(endPose, self.toolFrame)

        
        #code.interact(local=locals())

        endJointPositions = []
        for raveJointType in self.manip.GetArmIndices():
            rosJointType = self.raveJointTypesToRos[raveJointType]
            endJointPositions.append(endJoints[rosJointType])

        # manipName, toolFrame, pose, endJointPositions
        request = reqFunc(self.manip.GetName(), self.toolFrame, transEndPose, endJointPositions) 

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
            rospy.loginfo('Still returning trajectory')
            #return

        return self.trajoptTrajToDicts(result.GetTraj())


def testIK():
    rospy.init_node('test_IK',anonymous=True)
    rp = RavenPlanner(MyConstants.Arm.Right)
    rospy.sleep(2)

    angle = tfx.tb_angles(-3.2,88.5,-1.8)#tfx.tb_angles(-70.4,85.6,-70.7)
    endPose = tfx.pose([-.136, -.017, -.074], angle,frame=MyConstants.Frames.Link0)
    rospy.loginfo('Getting joint positions')
    rp.currentGrasp = 1.2
    joints = rp.getJointsFromPose(endPose)
   
    for jointType, jointPos in joints.items():
        print("jointType = {0}, jointPos = {1}".format(jointType,((180.0/pi)*jointPos)))

    rp.updateOpenraveJoints(joints)
    rp.env.SetViewer('qtcoin')

    code.interact(local=locals())

    rospy.spin()

def testRP():
    rospy.init_node('test_request',anonymous=True)
    rp = RavenPlanner(MyConstants.Arm.Right)



if __name__ == '__main__':
    #testIK()
    testRP()



"""
        rospy.loginfo('Links')
        for link in self.robot.GetLinks():
            print(link.GetName())

        rospy.loginfo('Joints')
        for joint in self.robot.GetJoints():
            print(joint.GetName())

        rospy.loginfo('Independent links')
        indLinks = self.manip.GetIndependentLinks()
        for link in indLinks:
            print(link.GetName())

        rospy.loginfo('Dependent links')
        for link in self.robot.GetLinks():
            if link not in indLinks:
                print(link.GetName())
"""

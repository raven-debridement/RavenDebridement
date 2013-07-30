#!/usr/bin/env python

"""
Adapted from Ben Kehoe's trajectory_player
"""

import roslib
roslib.load_manifest('master-control')
import rospy
from math import *
import copy
import sys, os

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *

import openravepy as rave
import numpy as np


import tf
import tfx

import thread
import threading

import Util
import Constants as MyConstants

import code

class Stage(object):
    def __init__(self,name,duration,cb):
        self.name = name
        self.duration = rospy.Duration(duration)
        self.cb = cb
		
    @staticmethod
    def stageBreaks(stages):
        stageBreaks = [rospy.Duration(0)]
        for stage in stages:
            stageBreaks.append(stage.duration + stageBreaks[-1])
        return stageBreaks

class RavenController():
    def __init__(self, arm, tfListener=None):
        self.arm = arm

        self.tfListener = tfListener
        if self.tfListener is None:
            self.tfListener = tfx.TransformListener.instance()
        rospy.loginfo('waiting for transform')
        
        self.tfListener.waitForTransform('/0_link','/tool_'+self.arm,rospy.Time(0),rospy.Duration(5))

        self.stopRunning = threading.Event()

        # ADDED, initializes the rest
        self.reset()
		
			
        self.pubCmd = rospy.Publisher('raven_command', RavenCommand)
		
        self.stateSub = rospy.Subscriber('raven_state',raven_2_msgs.msg.RavenState,self._stateCallback)
	
        self.header = Header()
        self.header.frame_id = '/0_link'
        
        self.thread = None

        #################
        # PAUSE COMMAND #
        #################
        header = Header()
        header.frame_id = MyConstants.Frames.Link0
        header.stamp = rospy.Time.now()
    
        # create the tool pose
        toolCmd = ToolCommand()
        toolCmd.pose = tfx.pose([0,0,0]).msg.Pose()
        toolCmd.pose_option = ToolCommand.POSE_RELATIVE
        toolCmd.grasp_option = ToolCommand.GRASP_OFF
    
        
        # create the arm command
        armCmd = ArmCommand()
        armCmd.tool_command = toolCmd
        armCmd.active = True
        
        # create the raven command
        ravenPauseCmd = RavenCommand()
        ravenPauseCmd.arm_names.append(self.arm)
        ravenPauseCmd.arms.append(armCmd)
        ravenPauseCmd.pedal_down = True
        ravenPauseCmd.header = header
        ravenPauseCmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE

        self.ravenPauseCmd = ravenPauseCmd
        
    
    def _stateCallback(self, msg):
        self.currentState = msg
        for arm in msg.arms:
            if arm.name == self.arm:
                self.currentPose = tfx.pose(arm.tool.pose, header=msg.header)
                self.currentGrasp = arm.tool.grasp 


    ###############################################
    # start, stop, and resetting of the raven arm #
    ###############################################

    def reset(self):
        """
        Resets player, allows for reuse.
        Never needs to be called by user, is automatically
        called when start is called.
        """
        self.stages = []
        self.startTime = None

        self.defaultSpeed = .01

		
        self.currentState = None
        self.currentPose = None
        self.currentGrasp = None
		
        # ADDED
        self.stopRunning.clear()


    def stop(self):
        """
        Sets flag to stop executing trajectory

        Returns False if times out, otherwise True
        """
        self.stopRunning.set()

        rate = rospy.Rate(50)
        timeout = Util.Timeout(999999)
        timeout.start()
        while not timeout.hasTimedOut():
            if not self.thread.isAlive():
                return True

            rate.sleep()

        return False


    def clearStages(self):
        """
        If playing, this effectively pauses the raven
        """
        self.stages = []

    def start(self):
        """
        Intended use is to call play once at the beginning
        and then add stages to move it
        """
        if self.thread == None or not self.thread.isAlive():
            self.reset()
            #self.thread = thread.start_new_thread(self.run, tuple())
            self.thread = threading.Thread(target=self.run)
            self.thread.setDaemon(True)
            self.thread.start()

        return True

    def run(self):
        rate = rospy.Rate(50)
		
        cmd = None
		
        while self.currentState is None and not rospy.is_shutdown():
            rate.sleep()
		
        if self.currentState.runlevel == 0:
            rospy.loginfo("Raven in E-STOP, waiting")
            while self.currentState.runlevel == 0 and not rospy.is_shutdown():
                rate.sleep()
		
        self.startTime = rospy.Time.now()
        numStages = 0
		
        success = None

        cmd = self.ravenPauseCmd

        # ADDED
        self.stopRunning.clear()
		
        lastStageIndex = -1
        while not rospy.is_shutdown():
			
            if self.currentState.runlevel == 0:
                rospy.logerr('Raven in E-STOP, exiting')
                success = False
                break

            # ADDED
            if self.stopRunning.isSet():
                self.stopRunning.clear()
                success = True
                break

            stages = self.stages

            # when a stage appears, set startTime
            if numStages == 0 and len(stages) > 0:
                self.startTime = rospy.Time.now()

            numStages = len(stages)
            stageBreaks = Stage.stageBreaks(stages)
            
            now = rospy.Time.now()

            if numStages > 0:
                durFromStart = now - self.startTime
                stageIndex = 0
                for idx,stageBreak in enumerate(stageBreaks):
                    if stageBreak > durFromStart:
                        stageIndex = idx-1
                        break
                else:
                    self.clearStages()
                    continue
                stageIndex = min(stageIndex,lastStageIndex + 1)
            
                lastStageIndex = stageIndex
            
                stage = stages[stageIndex]
            
                if stage.duration.is_zero():
                    t = 1
                else:
                    t = (durFromStart - stageBreaks[stageIndex]).to_sec() / stage.duration.to_sec()
            
            
                cmd = RavenCommand()
                cmd.pedal_down = True
            
                stage.cb(cmd,t)
            
            else:
                # no stages
                cmd = self.ravenPauseCmd

            self.header.stamp = now
            cmd.header = self.header
			
            self.pubCmd.publish(cmd)
			
            rate.sleep()
            
        return success


    ############################
    # Commanding the raven arm #
    ############################

    def addStage(self, name, duration, cb):
        self.stages.append(Stage(name,duration,cb))

    def goToPose(self, end, start=None, duration=None, speed=None):
        if start == None:
            start = self.currentPose

        start = tfx.pose(start)
        end = tfx.pose(end)
        
        if duration is None:
            if speed is None:
                speed = self.defaultSpeed
            duration = end.position.distance(start.position) / speed

        def fn(cmd, t):
            pose = start.interpolate(end, t)
            
            toolPose = pose.msg.Pose()

            RavenController.addArmPoseCmd(cmd, self.arm, toolPose)

        self.addStage('goToPose', duration, fn)
        
    ###############################
    # setting the gripper grasp   #
    ###############################

    def openGripper(self,duration=2):
        def fn(cmd,t):
            RavenController.addArmGraspCmd(cmd, self.arm, grasp=1, graspOption=ToolCommand.GRASP_INCREMENT_SIGN)
        self.addStage('Open gripper',duration,fn)
	
    def closeGripper(self,duration=2):
        def fn(cmd,t):
            RavenController.addArmGraspCmd(cmd, self.arm, grasp=-1, graspOption=ToolCommand.GRASP_INCREMENT_SIGN)
        self.addStage('Close gripper',duration,fn)

    def setGripper(self,value,duration=2):
        def fn(cmd,t):
            RavenController.addArmGraspCmd(cmd, self.arm, grasp=value, graspOption=ToolCommand.GRASP_SET_NORMALIZED)
        self.addStage('Set gripper',duration,fn)


    ###############################################
    # static methods for adding to a RavenCommand #
    ###############################################

    @staticmethod
    def addArmCmd(cmd,armName,toolPose=None,poseOption=ToolCommand.POSE_OFF,grasp=0,graspOption=ToolCommand.GRASP_OFF):
        cmd.arm_names.append(armName)

        arm_cmd = ArmCommand()
        arm_cmd.active = True

        tool_cmd = ToolCommand()
        tool_cmd.pose_option = poseOption
        if toolPose is not None:
            tool_cmd.pose = toolPose
            
        tool_cmd.grasp_option = graspOption
        tool_cmd.grasp = grasp

        arm_cmd.tool_command = tool_cmd

        cmd.arms.append(arm_cmd)

    @staticmethod
    def addArmPoseCmd(cmd,armName,toolPose,poseOption=ToolCommand.POSE_ABSOLUTE):
        return RavenController.addArmCmd(cmd, armName, toolPose=toolPose, poseOption=poseOption, graspOption=ToolCommand.GRASP_OFF)

    @staticmethod
    def addArmGraspCmd(cmd,armName,grasp,graspOption=ToolCommand.GRASP_INCREMENT_SIGN):
        return RavenController.addArmCmd(cmd, armName, grasp=grasp, graspOption=graspOption, poseOption=ToolCommand.POSE_OFF)




    def add_go_to_pose_using_joints(self, name, startJoints, endPose, arm=None, duration=None, speed=None):
        """
        Calculate the the joints for endPose using openrave IK

        Then, return callback function for moving with linear angular velocity
        from startJoints to endJoints
        """
        env = rave.Environment()
        ravenFile = os.path.dirname(__file__) + '/../models/raven2.zae'
        env.Load(ravenFile)
        robot = env.GetRobots()[0]

        code.interact(local=locals())




def test_startstop():
    rospy.init_node('raven_arm',anonymous=True)
    leftArm = RavenController('L')
    rospy.sleep(2)

    rospy.loginfo('Press enter to start')
    raw_input()

    leftArm.start()

    rospy.loginfo('Press enter to stop')
    raw_input()

    leftArm.stop()

    rospy.loginfo('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    test_startstop()

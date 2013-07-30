#!/usr/bin/env python

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
    def stage_breaks(stages):
        stage_breaks = [rospy.Duration(0)]
        for stage in stages:
            stage_breaks.append(stage.duration + stage_breaks[-1])
        return stage_breaks

class RavenArm():
    def __init__(self, arm, tf_listener=None):
        self.arm = arm

        self.tf_listener = tf_listener
        if self.tf_listener is None:
            self.tf_listener = tfx.TransformListener.instance()
        rospy.loginfo('waiting for transform')
        
        self.tf_listener.waitForTransform('/0_link','/tool_'+self.arm,rospy.Time(0),rospy.Duration(5))

        self.stopRunning = threading.Event()

        # ADDED, initializes the rest
        self.reset()
		
			
        self.pub_cmd = rospy.Publisher('raven_command', RavenCommand)
		
        self.state_sub = rospy.Subscriber('raven_state',raven_2_msgs.msg.RavenState,self._state_callback)
	
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
        
    
    def _state_callback(self, msg):
        self.current_state = msg
        for arm in msg.arms:
            if arm.name == self.arm:
                self.current_pose = tfx.pose(arm.tool.pose, header=msg.header)
                self.current_grasp = arm.tool.grasp 


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
        self.start_time = None

        self.default_speed = .01

		
        self.current_state = None
        self.current_poses = {}
        self.current_grasps = {}
		
        # ADDED
        self.stopRunning.clear()


    def stop(self):
        """
        Sets flag to stop executing trajectory

        Returns False if times out, otherwise True
        """
        self.stopRunning.set()

        rate = rospy.Rate(50)
        timeout = Util.TimeoutClass(999999)
        timeout.start()
        while not timeout.hasTimedOut():
            if not self.thread.isAlive():
                return True

            rate.sleep()

        return False


    def clear_stages(self):
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
		
        while self.current_state is None and not rospy.is_shutdown():
            rate.sleep()
		
        if self.current_state.runlevel == 0:
            rospy.loginfo("Raven in E-STOP, waiting")
            while self.current_state.runlevel == 0 and not rospy.is_shutdown():
                rate.sleep()
		
        self.start_time = rospy.Time.now()
        numStages = 0
		
        success = None

        cmd = self.ravenPauseCmd

        # ADDED
        self.stopRunning.clear()
		
        last_stage_ind = -1
        while not rospy.is_shutdown():
			
            if self.current_state.runlevel == 0:
                rospy.logerr('Raven in E-STOP, exiting')
                success = False
                break

            # ADDED
            if self.stopRunning.isSet():
                self.stopRunning.clear()
                success = True
                break

            stages = self.stages

            # when a stage appears, set start_time
            if numStages == 0 and len(stages) > 0:
                self.start_time = rospy.Time.now()

            numStages = len(stages)
            stage_breaks = Stage.stage_breaks(stages)
            
            now = rospy.Time.now()

            if numStages > 0:
                dur_from_start = now - self.start_time
                stage_ind = 0
                for idx,stage_break in enumerate(stage_breaks):
                    if stage_break > dur_from_start:
                        stage_ind = idx-1
                        break
                else:
                    self.clear_stages()
                    continue
                stage_ind = min(stage_ind,last_stage_ind + 1)
            
                last_stage_ind = stage_ind
            
                stage = stages[stage_ind]
            
                if stage.duration.is_zero():
                    t = 1
                else:
                    t = (dur_from_start - stage_breaks[stage_ind]).to_sec() / stage.duration.to_sec()
            
            
                cmd = RavenCommand()
                cmd.pedal_down = True
            
                stage.cb(cmd,t)
            
            else:
                # no stages
                cmd = self.ravenPauseCmd

            self.header.stamp = now
            cmd.header = self.header
			
            self.pub_cmd.publish(cmd)
			
            rate.sleep()
            
        return success


    ############################
    # Commanding the raven arm #
    ############################

    def add_stage(self, name, duration, cb):
        self.stages.append(Stage(name,duration,cb))

    def go_to_pose(self, end, start=None, duration=None, speed=None):
        if start == None:
            start = self.current_pose

        start = tfx.pose(start)
        end = tfx.pose(end)
        
        if duration is None:
            if speed is None:
                speed = self.default_speed
            duration = end.position.distance(start.position) / speed

        def fn(cmd, t):
            pose = start.interpolate(end, t)
            
            tool_pose = pose.msg.Pose()

            RavenPose.add_arm_pose_cmd(cmd, self.arm, tool_pose)

        self.add_stage('go_to_pose', duration, fn)
        


    ###############################################
    # static methods for adding to a RavenCommand #
    ###############################################

    @staticmethod
    def add_arm_cmd(cmd,arm_name,tool_pose=None,pose_option=ToolCommand.POSE_OFF,grasp=0,grasp_option=ToolCommand.GRASP_OFF):
        cmd.arm_names.append(arm_name)

        arm_cmd = ArmCommand()
        arm_cmd.active = True

        tool_cmd = ToolCommand()
        tool_cmd.pose_option = pose_option
        if tool_pose is not None:
            tool_cmd.pose = tool_pose
            
        tool_cmd.grasp_option = grasp_option
        tool_cmd.grasp = grasp

        arm_cmd.tool_command = tool_cmd

        cmd.arms.append(arm_cmd)

    @staticmethod
    def add_arm_pose_cmd(cmd,arm_name,tool_pose,pose_option=ToolCommand.POSE_ABSOLUTE):
        return RavenArm.add_arm_cmd(cmd, arm_name, tool_pose=tool_pose, pose_option=pose_option, grasp_option=ToolCommand.GRASP_OFF)

    @staticmethod
    def add_arm_grasp_cmd(cmd,arm_name,grasp,grasp_option=ToolCommand.GRASP_INCREMENT_SIGN):
        return RavenArm.add_arm_cmd(cmd, arm_name, grasp=grasp, grasp_option=grasp_option, pose_option=ToolCommand.POSE_OFF)




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
    leftArm = RavenArm('L')
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

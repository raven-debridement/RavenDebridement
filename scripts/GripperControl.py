#!/usr/bin/env python

import roslib; roslib.load_manifest('master-control')
import numpy as np
import os
import random

import rospy

import tf
import tf.transformations as tft

from geometry_msgs.msg import *
from std_msgs.msg import Header

import code

import tfx
from raven_2_msgs.msg import *
from raven_2_trajectory.trajectory_player import TrajectoryPlayer, Stage

import Constants

class GripperControlClass:
    """
    Class for controlling the end effectors of the Raven
    """

    def __init__ (self, armName=Constants.Arms.Left, listener=None):
        self.armName = armName
        

        if self.arms = Constants.Arms.Left:
            self.toolframe = Constants.Frames.LeftTool
            self.tooltopic = Constants.ToolTopic.Left
        else:
            self.toolframe = Constants.Frames.RightTool
            self.tooltopic = Constants.ToolTopic.Right


        if listener == None:
            listener = tf.TransformListener()
        self.listener = listener

        self.pub = rospy.Publisher(self.tooltopic, ToolCommandStamped)
        #self.player = TrajectoryPlayer(tf_listener=self.listener, arms=self.armName)
 
    def goToGripperPose(self, toolPose, toolPoseOption=ToolCommand.POSE_ABSOLUTE):
        """
        For now, toolPose is assumed to be in the
        tool frames point of view
        """
        # create the header
        header = Header()
        header.frame_id = self.toolframe
        header.stamp = rospy.Time.now()

        # create the tool pose
        toolCmd = ToolCommand()
        toolCmd.pose = toolPose
        toolCmd.pose_option = toolPoseOption
        toolCmd.grasp_option = ToolCommand.GRASP_OFF
        
        # create tool pose stamped
        toolCmdStamped = ToolCommandStamped()
        toolCmdStamped.header = header
        toolCmdStamped.command = toolCmd


        self.pub.Publish(toolCmdStamped)
        """
        # create the arm command
        armCmd = ArmCommand()
        armCmd.tool_command = toolCmd
        armCmd.active = True

        # create the raven command
        ravenCmd = RavenCommand()
        ravenCmd.arm_names.append(self.armName)
        ravenCmd.arms.append(armCmd)
        ravenCmd.pedal_down = True
        """
        
    def closeGripper(self):
        # create the header
        header = Header()
        header.frame_id = self.toolframe
        header.stamp = rospy.Time.now()

        # create the tool pose
        toolCmd = ToolCommand()
        toolCmd.pose_option = ToolCommand.POSE_OFF
        toolCmd.grasp_option = ToolCommand.GRASP_INCREMENT_SIGN
        toolCmd.grasp = -1

        # create tool pose stamped
        toolCmdStamped = ToolCommandStamped()
        toolCmdStamped.header = header
        toolCmdStamped.command = toolCmd

        self.pub.Publish(toolCmdStamped)
        
        

def test():
    rospy.init_node('gripper_control',anonymous=True)
    gripperControl = GripperControlClass(Constants.Arm.Right, tf.TransformListener())
    gripperControl.closeGripper()
    rospy.spin()
        
if __name__ == '__main__':    
    test()

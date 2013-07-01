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

class ToolControlClass:
    """
    Class for controlling the end effectors of the Raven
    """

    def __init__ (self, arms=Constants.Arms.Left, listener=None):
        self.arms = arms
        
        if self.arms = Constants.Arms.Left:
            self.toolframe = Constants.Frames.LeftTool
        else:
            self.toolframe = Constants.Frames.RightTool


        if listener == None:
            listener = tf.TransformListener()
        self.listener = listener

 
    def goToToolPose(self, toolPoseStamped, toolPoseOption=ToolCommand.POSE_ABSOLUTE):
        return

        
        
        
def test():
    return
        
if __name__ == '__main__':    
    test()

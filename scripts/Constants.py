#!/usr/bin/env python

"""
Contains general constants that
are frequently used
"""

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy

class StereoClick:
    StereoName = 'stereo_points_3d'

class AR:
    Stereo = 'stereo_pose'

class Arm:
    Left = 'L'
    Right = 'R'

class Frames:
    LeftTool = '/tool_L'
    RightTool = '/tool_R'
    Base = '/0_link'

class RavenTopics:
    #publish ToolCommandStamped directly
    LeftTool = '/raven_command/tool/L'
    RightTool = '/raven_command/tool/R'

    RavenState = '/raven_state'
    RavenCommand = '/raven_command'


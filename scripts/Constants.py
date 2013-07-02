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

class Arm:
    Left = 'L'
    Right = 'R'

class Frames:
    LeftTool = '/tool_L'
    RightTool = '/tool_R'
    Base = '/0_link'

# publish ToolCommandStamped directly
class ToolTopic:
    Left = '/raven_command/tool/L'
    Right = '/raven_command/tool/R'

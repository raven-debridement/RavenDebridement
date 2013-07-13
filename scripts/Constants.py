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
    class Frames:
        Grasper1 = '/grasper1_tip'
        Grasper2 = '/grasper2_tip'
        Cube1 = '/cube_1'
        Cube2 = '/cube_2'
        Cube3 = '/cube_3'
        Cube4 = '/cube_4'
        Object = '/object'

class Arm:
    Left = 'L'
    Right = 'R'

class Frames:
    LeftTool = '/tool_L'
    RightTool = '/tool_R'
    World = '/world'
    LeftBase = '/base_link_L'
    RightBase = '/base_link_R'
    Link0 = '/0_link'

class RavenTopics:
    #publish ToolCommandStamped directly
    LeftTool = '/raven_command/tool/L'
    RightTool = '/raven_command/tool/R'

    RavenState = '/raven_state'
    RavenCommand = '/raven_command'


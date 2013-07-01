#!/usr/bin/env python

# Authors: Ben Kehoe, Anna Lee

import roslib
roslib.load_manifest('raven_2_trajectory')

import rospy
from raven_2_msgs.msg import *
from geometry_msgs.msg import *

from math import *
import optparse

RATE = 100

help_msg = """
Control The Raven!
---------------------------
Moving around:
   w
   s

q/e : increase/decrease speed by 10%

CTRL-C to quit
"""

KEYS = {'help': 'H',
        '+pos':     'W', '-pos':     'S',
        '+speed':     'Q', '-speed':     'E'}

class _Getch:
        """Gets a single character from standard input.  Does not echo to the screen."""
        def __init__(self):
                try:
                        self.impl = _GetchWindows()
                except ImportError:
                        self.impl = _GetchUnix()

        def __call__(self): return self.impl()


class _GetchUnix:
        def __init__(self):
                import tty, sys

        def __call__(self):
                import sys, tty, termios
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                        tty.setraw(sys.stdin.fileno())
                        ch = sys.stdin.read(1)
                finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch


class _GetchWindows:
        def __init__(self):
                import msvcrt

        def __call__(self):
                import msvcrt
                return msvcrt.getch()

class PoseCommander:
    """
    arm_name is either 'R' or 'L'
    """
    def __init__(self,arm_name,joint,speed=0.1):
        self.state = None
        self.first_state = None
        self.first_joint_value = {}
        self.arm_names = ['R','L']#[arm_name]
        self.selected_arm = arm_name

        self.joint = joint
        self.speed = speed

        self.start_time = None

        self.pub = rospy.Publisher('raven_command', RavenCommand)

    def setPosition(self, position):
        now = rospy.Time.now()
        if not self.start_time:
            self.start_time = now

        dur_from_start = (now - self.start_time).to_sec()

        cmd = RavenCommand()
        cmd.header.stamp = now

        cmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE
        cmd.pedal_down = True

        cmd.arm_names.append(self.selected_arm)
        arm_cmd = ArmCommand()
        arm_cmd.active = True
        arm_cmd.tool_command.pose_option = ToolCommand.POSE_POS_REL_ORI_OFF
        arm_cmd.tool_command.grasp_option = ToolCommand.GRASP_OFF
        pose = Pose()
        pose.position.x = position
        arm_cmd.tool_command.pose = pose
        cmd.arms.append(arm_cmd)

        self.pub.publish(cmd)

    def printState(self):
        print "currently:\tposition %s\tspeed %s" % (0, self.speed)

    def sendCmd(self):
        try:
            getch = _Getch()
            cmd = getch().upper()
            if ord(cmd) == 3:
                rospy.signal_shutdown('user terminated')
            else:
                if cmd == KEYS['help']:
                    print help_msg
                elif cmd == KEYS['+pos']:
                    self.position = self.speed / RATE
                    self.setPosition(self.position)
                elif cmd == KEYS['-pos']:
                    self.position = -self.speed / RATE
                    self.setPosition(self.position)
                elif cmd == KEYS['+speed']:
                    self.speed = self.speed * 1.1
                    self.printState()
                elif cmd == KEYS['-speed']:
                    self.speed = self.speed * 0.9
                    self.printState()
        except Exception, e:
            print e

if __name__ == '__main__':
    rospy.init_node('raven_key_teleop',anonymous=True)

    parser = optparse.OptionParser()
    (options,args) = parser.parse_args(rospy.myargv())

    arm_name = 'R'
    if len(args) > 1:
        arm_name = args[1]

    speed = None
    if len(args) > 2:
        speed = float(args[2])

    commander = PoseCommander(arm_name, speed)

    rate = rospy.Rate(RATE)
    status = 0
    while not rospy.is_shutdown():
        if status == 0:
            print help_msg
        if status % RATE == 0:
            commander.printState()
        status = (status + 1) % (RATE)
    
        commander.sendCmd()

        rate.sleep()
    

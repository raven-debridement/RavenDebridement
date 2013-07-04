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

# rename so no conflict with raven_2_msgs.msg.Constants
import Constants as MyConstants
import Util


class GripperControlClass:
    """
    Class for controlling the end effectors of the Raven
    """

    def __init__ (self, armName, listener=None):
        self.armName = armName
        

        if self.armName == MyConstants.Arm.Left:
            self.toolframe = MyConstants.Frames.LeftTool
            self.tooltopic = MyConstants.RavenTopics.LeftTool
            self.baseframe = MyConstants.Frames.LeftBase
        else:
            self.toolframe = MyConstants.Frames.RightTool
            self.tooltopic = MyConstants.RavenTopics.RightTool
            self.baseframe = MyConstants.Frames.RightBase

        if listener == None:
            listener = tf.TransformListener()
        self.listener = listener

        #self.pub = rospy.Publisher(self.tooltopic, ToolCommandStamped)
        #self.raven_pub = rospy.Publisher(MyConstants.RavenTopics.RavenCommand, RavenCommand)
        #self.player = TrajectoryPlayer(arms=self.armName)

        rospy.sleep(1)
 
    def goToGripperPose(self, startPose, endPose):
        """
        Given a startPose, move to endPose
        Both startPose and endPose are geometry_msgs.msg.Pose

        startPose is the current pose of the gripper WITH RESPECT TO MyConstants.Frames.Link0

        toolPose is the current pose of the gripper WITH RESPECT TO MyConstants.Frames.Link0
        """
        startPose = tfx.pose(startPose)
        endPose = tfx.pose(endPose)

        player = TrajectoryPlayer(arms=self.armName)

        #player.add_goto_first_pose(endPose,duration=10)
        player.add_pose_to_pose('goToGripperPose',startPose,endPose)

        return player.play()
        

    def goToGripperPoseDelta(self, startPose, deltaPose):
        """
        Given a startPose, move by deltaPose
        Both startPose and deltaPose are geometry_msgs.msg.Pose

        startPose is the pose of the gripper WITH RESPECT TO MyConstants.Frames.Link0

        deltaPose is the difference between the current pose and the final pose
        """

        player = TrajectoryPlayer(arms=self.armName)


        #endPose = tfx.pose(startPose.position + deltaPose.position, startPose.orientation.quaternion + deltaPose.orientation.quaternion)
        #endPose = tfx.pose(startPose.position + deltaPose.position, startPose.orientation.matrix*deltaPose.orientation.matrix)

        #endAngles = tfx.tb_angles(startPose.tb_angles.yaw_deg + deltaPose.tb_angles.yaw_deg, startPose.tb_angles.pitch_deg + deltaPose.tb_angles.pitch_deg, startPose.tb_angles.roll_deg + deltaPose.tb_angles.roll_deg)
        #endPose = tfx.pose(startPose.position + deltaPose.position, endAngles)
        # work on orientation combination

        endPose = Util.addPoses(startPose, deltaPose)

        # convert to tfx format
        startPose = tfx.pose(startPose)
        endPose = tfx.pose(endPose)

        #code.interact(local=locals())

        print(startPose)
        print(deltaPose)
        print(endPose)

        player.add_pose_to_pose('goToGripperPose',startPose,endPose,duration=10)

        return player.play()
        

    def closeGripper(self):
        player = TrajectoryPlayer(arms=self.armName)
        player.add_close_gripper(duration=4)
        return player.play()
                
    def openGripper(self):
        player = TrajectoryPlayer(arms=self.armName)
        player.add_open_gripper(duration=4)
        return player.play()

    def getGripperPose(self,frame=MyConstants.Frames.World):
        """
        Returns gripper pose w.r.t. frame

        geometry_msgs.msg.Pose
        """
        self.listener.waitForTransform(frame, self.toolframe, rospy.Time.now(), rospy.Duration(5))
        commonTime = self.listener.getLatestCommonTime(frame, self.toolframe)
        pos, quat = self.listener.lookupTransform(frame, self.toolframe, commonTime)

        return tfx.pose(pos,quat).msg.Pose()
        
#w.r.t. Link0
down = tfx.tb_angles(0,66,0)
rightArmDot = tfx.pose(tfx.point(-.060, -.039, -.155), down)
leftArmDot = tfx.pose(tfx.point(-.036, -.027, -.163), down)

def test_closeGripper():
    rospy.sleep(4)
    rospy.init_node('gripper_control',anonymous=True)
    gripperControl = GripperControlClass(MyConstants.Arm.Right, tf.TransformListener())

    rospy.loginfo('Closing the gripper')
    gripperControl.closeGripper()
    rospy.loginfo('Gripper Closed')


def test_moveGripper():
    rospy.sleep(4)
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(MyConstants.Arm.Left, tf.TransformListener())
    
    currPose = tfx.pose(gripperControl.getGripperPose(frame=MyConstants.Frames.Link0))
    
    
    desPose = leftArmDot
    #desPose.position.x -= .04
    
    """
    common = listener.getLatestCommonTime(MyConstants.Frames.World, MyConstants.Frames.Link0)
    posestamped = PoseStamped()
    posestamped.header.stamp = common
    posestamped.header.frame_id = MyConstants.Frames.World
    posestamped.pose = currPose

    posestamped = listener.transformPose(MyConstants.Frames.Link0, posestamped)

    pose = posestamped.pose
    """

    currPose = currPose.msg.Pose()
    desPose = desPose.msg.Pose()
    
    gripperControl.goToGripperPose(currPose,desPose)
    




def test_moveGripperDelta():
    rospy.sleep(4)
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(MyConstants.Arm.Left, tf.TransformListener())

    """
    currPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.World))

    desPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.World))
    desPose.position.y -= .014
    #desPose.position.x += .03
    #desPose.position.z += .01
    """

    
    currPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0))
    
    # w.r.t. Link0
    desPose = leftArmDot
    

    deltaPose = tfx.pose(Util.subPoses(desPose, currPose))
    

    #deltaOrientation = tfx.tb_angles(desPose.tb_angles.yaw_deg - currPose.tb_angles.yaw_deg, desPose.tb_angles.pitch_deg - currPose.tb_angles.pitch_deg, desPose.tb_angles.roll_deg - currPose.tb_angles.roll_deg)
    #deltaPose = tfx.pose(desPose.position - currPose.position, deltaOrientation)
    
    #d = desPose.orientation.quaternion - currPose.orientation.quaternion
    #deltaQuat = d - np.dot(d, currPose.orientation.quaternion)*currPose.orientation.quaternion
    #deltaPose = tfx.pose(desPose.position - currPose.position, deltaQuat)

    #code.interact(local=locals())

    #deltaPose = tfx.pose(desPose.position - currPose.position, desPose.orientation.quaternion - currPose.orientation.quaternion)
    
    

    #print(desPose)
    #print(currPose)
    #print(deltaPose)
    #print(desPose.orientation.quaternion)
    #print(currPose.orientation.quaternion)
    #print(deltaPose.orientation.quaternion)
    
    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(MyConstants.Frames.Link0), deltaPose.msg.Pose())





def test_gripperPose():
    rospy.init_node('gripper_control',anonymous=True)
    gripperControl = GripperControlClass(MyConstants.Arm.Left, tf.TransformListener())
    """
    point = tfx.point(-.060, -.039, -.150)
    orientation = tfx.tb_angles(0,90,0)
    
    pose = tfx.pose(point, orientation)
    print(pose.msg.Pose())
    """

    print(tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0)))

def test_down():
    down = tfx.tb_angles(0,90,0)
    print(down.quaternion)
    code.interact(local=locals())

if __name__ == '__main__':
    #test_closeGripper()
    #test_moveGripper()
    test_moveGripperDelta()
    #test_gripperPose()
    #test_down()








"""
        # original attempt at moving the arm
        # create the header
        header = Header()
        #header.frame_id = self.toolframe
        header.frame_id = MyConstants.Frames.Base
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
        
        
        #self.pub.publish(toolCmdStamped)
        
        # create the joint command
        jointCmd = JointCommand()
        jointCmd.command_type = JointCommand.COMMAND_TYPE_VELOCITY
        jointCmd.value = 0

        # create the arm command
        armCmd = ArmCommand()
        armCmd.tool_command = toolCmd
        armCmd.active = True
        armCmd.joint_types.append(Constants.JOINT_TYPE_INSERTION)
        armCmd.joint_commands.append(jointCmd)

        # create the raven command
        ravenCmd = RavenCommand()
        ravenCmd.arm_names.append(self.armName)
        ravenCmd.arms.append(armCmd)
        ravenCmd.pedal_down = True
        ravenCmd.header = header
        ravenCmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE

        self.raven_pub.publish(ravenCmd)
        
        return True
"""


"""
    def setGripper(self, value):
        # create the header
        header = Header()
        header.frame_id = self.toolframe
        header.stamp = rospy.Time.now()

        # create the tool pose
        toolCmd = ToolCommand()
        toolCmd.pose_option = ToolCommand.POSE_OFF
        toolCmd.grasp_option = ToolCommand.GRASP_INCREMENT_SIGN
        toolCmd.grasp = value

        # create tool pose stamped
        toolCmdStamped = ToolCommandStamped()
        toolCmdStamped.header = header
        toolCmdStamped.command = toolCmd

        #rospy.loginfo('publishing')
        #self.pub.publish(toolCmdStamped)

        # create the arm command
        armCmd = ArmCommand()
        armCmd.tool_command = toolCmd
        armCmd.active = True

        # create the raven command
        ravenCmd = RavenCommand()
        ravenCmd.arm_names.append(self.armName)
        ravenCmd.arms.append(armCmd)
        ravenCmd.pedal_down = True
        header.frame_id = MyConstants.Frames.Link0
        ravenCmd.header = header
        
        self.raven_pub.publish(ravenCmd)
        
        return True
"""



 
"""
    def goToGripperPoseRelative(self, toolPose, arm=None):

        player = TrajectoryPlayer(arms=self.armName)
        
        def fn(cmd,t):
            player.add_arm_pose_cmd(cmd,player._check(arm), toolPose, pose_option=ToolCommand.POSE_RELATIVE)

        player.add_stage('pose relative', 5, fn)

        # dangerous!!!!!
        #player.play()

        return True
"""



"""
def test_moveGripperRelative():
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(MyConstants.Arm.Right, tf.TransformListener())


    currPose = gripperControl.getGripperPose()
    
    desPose = gripperControl.getGripperPose()
    #desPose.position.z -= .05

    deltaPose = tfx.pose(desPose.position - currPose.position, desPose.orientation.quaternion - currPose.orientation.quaternion)

    
    gripperControl.goToGripperPoseRelative(deltaPose)
""" 

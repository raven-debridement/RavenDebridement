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

import thread

# rename so no conflict with raven_2_msgs.msg.Constants
import Constants as MyConstants
import Util
from MyTrajectoryPlayer import MyTrajectoryPlayer
from ARImageDetection import ARImageDetectionClass


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
        
        self.player = MyTrajectoryPlayer(arms=self.armName)

        self.jointStates = list()
        rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self.ravenStateCallback)

        rospy.sleep(1)
 
    def goToGripperPose(self, startPose, endPose, duration=None):
        """
        Given a startPose, move to endPose
        Both startPose and endPose are geometry_msgs.msg.Pose

        startPose is the current pose of the gripper WITH RESPECT TO MyConstants.Frames.Link0
        """
        startPose = tfx.pose(startPose)
        endPose = tfx.pose(endPose)

        # TEMP, until fix orientation issue
        endPose = tfx.pose(endPose.position, tfx.tb_angles(0,90,0))

        self.player.clear_stages()
        self.player.add_pose_to_pose('goToGripperPose',startPose,endPose,duration=duration)
        

    def goToGripperPoseDelta(self, startPose, deltaPose, duration=None):
        """
        Given a startPose, move by deltaPose
        Both startPose and deltaPose are geometry_msgs.msg.Pose

        startPose is the pose of the gripper WITH RESPECT TO MyConstants.Frames.Link0

        deltaPose is the difference between the current pose and the final pose
        """

        # convert to tfx format
        startPose = tfx.pose(startPose)
        deltaPose = tfx.pose(deltaPose)


        endPosition = startPose.position + deltaPose.position
    
        endQuatMat = startPose.orientation.matrix * deltaPose.orientation.matrix

        endPose = tfx.pose(endPosition, endQuatMat)
        
        # TEMP, until fix orientation issue
        endPose = tfx.pose(endPosition, tfx.tb_angles(0,90,0))
        """
        desAngle = tfx.tb_angles(0,90,0).msg
        actualAngle = endPose.orientation.msg.Quaternion()
        between = Util.angleBetweenQuaternions(desAngle, actualAngle)
        print('between 0,90,0')
        print(between)
        return
        """
        

        self.player.clear_stages()
        self.player.add_pose_to_pose('goToGripperPose',startPose,endPose,duration=duration)

    def start(self):
        # start running, no blocking
        return self.player.play(False)

    def pause(self):
        # pauses by clearing stages
        self.player.clear_stages()
        
    def stop(self):
        return self.player.stop_playing()

    def closeGripper(self,duration=5):
        self.player.clear_stages()
        self.player.add_close_gripper(duration=duration)
                
    def openGripper(self,duration=5):
        self.player.clear_stages()
        self.player.add_open_gripper(duration=duration)

    def setGripper(self, value, duration=5):
        self.player.clear_stages()
        self.player.add_set_gripper(value,duration=duration)

    def getGripperPose(self,frame=MyConstants.Frames.World):
        """
        Returns gripper pose w.r.t. frame

        geometry_msgs.msg.Pose
        """
        self.listener.waitForTransform(frame, self.toolframe, rospy.Time.now(), rospy.Duration(5))
        commonTime = self.listener.getLatestCommonTime(frame, self.toolframe)
        pos, quat = self.listener.lookupTransform(frame, self.toolframe, commonTime)

        return tfx.pose(pos,quat).msg.Pose()

    def ravenStateCallback(self, msg):
        self.jointStates =  msg.arms[0].joints

    def getJointStates(self):
        return self.jointStates
        







#w.r.t. Link0
#down = tfx.tb_angles(0,66,0)
down = tfx.tb_angles(0,90,0)
rightArmDot = tfx.pose(tfx.point(-.117, -.020, -.123), down)
leftArmDot = tfx.pose(tfx.point(.012, -.034, -.119), down)
leftArmDotOther = tfx.pose(tfx.point(-.034, -.030, -.159), tfx.tb_angles(-9,56.4,-3.4))
# tfx.tb_angles(0,40,0)
leftArmFourthDot = tfx.pose(tfx.point(-.084,.003,-.157), down)

armDot = leftArmDot
arm = MyConstants.Arm.Left

def test_opencloseGripper(close=True,duration=2):
    rospy.init_node('gripper_control',anonymous=True)
    rospy.sleep(2)
    gripperControl = GripperControlClass(arm, tf.TransformListener())
    rospy.sleep(2)

    gripperControl.start()

    rospy.loginfo('Setting the gripper')
    if close:
        gripperControl.closeGripper(duration=duration)
    else:
        gripperControl.openGripper(duration=duration)
    
    raw_input()

def test_moveGripper():
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(arm, tf.TransformListener())
    rospy.sleep(4)

    gripperControl.start()

    currPose = tfx.pose(gripperControl.getGripperPose(frame=MyConstants.Frames.Link0))
    
    
    desPose = armDot
    #desPose.position.x -= .04

    #rospy.loginfo('desPose')
    #rospy.loginfo(desPose)
    #rospy.loginfo('currPose')
    #rospy.loginfo(currPose)
    
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

    while not rospy.is_shutdown():
        rospy.sleep(1)
    




def test_moveGripperDelta():
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(arm, listener)
    rospy.sleep(4)

    currPose = imageDetector.getGripperPose(arm)
    desPose = imageDetector.getObjectPose()

    
    currPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0))
    
    # w.r.t. Link0
    desPose = armDot
    


    deltaPose = Util.deltaPose(currPose, desPose)
    deltaPose.position.z += .03

    #gripperControl.start()
    
    rospy.loginfo('Press enter')
    raw_input()

    transBound = .01
    rotBound = 25
    
    rate = rospy.Rate(1)

    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(MyConstants.Frames.Link0), deltaPose)

    while not Util.withinBounds(currPose, desPose, transBound, rotBound) and not rospy.is_shutdown():
        rospy.loginfo('loop')
        currPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0))    
        rate.sleep()

def test_moveGripperDeltaAR():
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(arm, listener)
    imageDetector = ARImageDetectionClass()
    rospy.sleep(4)
    
    """
    currPose = tfx.pose([0,0,0])#,frame='/stereo_53')
    desPose = tfx.pose([0,0,0])#,frame='/stereo_53')
    
    tf_des_to_53 = tfx.lookupTransform('/stereo_33','/stereo_53')
    desPose = tf_des_to_53 * desPose
    #desPose.frame = '/stereo_53'
    desPose = tfx.pose(desPose.position.tolist(),desPose.orientation.tolist())
    """

    currPose = imageDetector.getGripperPose(arm)
    desPose = imageDetector.getObjectPose()

    listener.waitForTransform('/stereo_33','/stereo_53',rospy.Time.now(), rospy.Duration(5))
    currPose = listener.transformPose('/stereo_53',currPose)
    desPose = listener.transformPose('/stereo_53',desPose)
    
    
    deltaPose = Util.deltaPose(currPose, desPose)
    deltaPose.position.z += .03
    deltaPose = tfx.pose(deltaPose.position,[0,0,0,1]).msg.Pose()

    code.interact(local=locals())
    #return
    

    #gripperControl.start()
    
    rospy.loginfo('Press enter')
    raw_input()

    transBound = .01
    rotBound = 25
    
    rate = rospy.Rate(1)

    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(MyConstants.Frames.Link0), deltaPose)

    while not Util.withinBounds(currPose, desPose, transBound, rotBound) and not rospy.is_shutdown():
        rospy.loginfo('loop')
        currPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0))    
        rate.sleep()


def test_servo():
    rospy.init_node('gripper_control',anonymous=True)
    rospy.sleep(2)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(arm, tf.TransformListener())
    rospy.sleep(2)

    gripperControl.start()

    rospy.loginfo('Press enter')
    raw_input()

    currPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0))
    
    # w.r.t. Link0
    desPose = armDot

    
    transBound = .01
    rotBound = 25
    
    rate = rospy.Rate(1)

    while not Util.withinBounds(currPose, desPose, transBound, rotBound) and not rospy.is_shutdown():
        rospy.loginfo('LOOP!!!!!!!!!')
        
        gripperControl.pause()

        currPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0))
    
        deltaPose = Util.deltaPose(currPose, desPose)
    
        gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(MyConstants.Frames.Link0), deltaPose)
        
        rate.sleep()
	


        

def test_gripperPose():
    rospy.init_node('gripper_control',anonymous=True)
    gripperControl = GripperControlClass(arm, tf.TransformListener())
    """
    point = tfx.point(-.060, -.039, -.150)
    orientation = tfx.tb_angles(0,90,0)
    
    pose = tfx.pose(point, orientation)
    print(pose.msg.Pose())
    """

    print(tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0)))

def test_down():
    down = tfx.tb_angles(-0.0,90.0,-0.0)
    print(down.quaternion)
    point = tfx.point(1,2,3)
    pose = tfx.pose(point)
    pose.orientation = down
    print(pose)
    code.interact(local=locals())

def test_commandRaven():
    rospy.init_node('command_raven',anonymous=True)
    armName = arm
    gripperControl = GripperControlClass(armName, tf.TransformListener())
    raven_pub = rospy.Publisher(MyConstants.RavenTopics.RavenCommand, RavenCommand)
    rospy.sleep(4)

    desPose = armDot
    
    while not rospy.is_shutdown():

        currPose = gripperControl.getGripperPose(MyConstants.Frames.Link0)

        deltaPose = Util.deltaPose(currPose, desPose)


 
        # original attempt at moving the arm
        # create the header
        header = Header()
        #header.frame_id = self.toolframe
        header.frame_id = MyConstants.Frames.Link0
        header.stamp = rospy.Time.now()
    
        # create the tool pose
        toolCmd = ToolCommand()
        toolCmd.pose = deltaPose #tfx.pose([0,0,0]).msg.Pose()
        toolCmd.pose_option = ToolCommand.POSE_RELATIVE
        toolCmd.grasp_option = ToolCommand.GRASP_OFF
    
        
        # create tool pose stamped
        toolCmdStamped = ToolCommandStamped()
        toolCmdStamped.header = header
        toolCmdStamped.command = toolCmd
        
            
        # create the joint command
        #jointCmd = JointCommand()
        #jointCmd.command_type = JointCommand.COMMAND_TYPE_VELOCITY
        #jointCmd.value = 0

        # create the arm command
        armCmd = ArmCommand()
        armCmd.tool_command = toolCmd
        armCmd.active = True
        #armCmd.joint_types.append(Constants.JOINT_TYPE_INSERTION)
        #armCmd.joint_commands.append(jointCmd)

        # create the raven command
        ravenCmd = RavenCommand()
        ravenCmd.arm_names.append(armName)
        ravenCmd.arms.append(armCmd)
        ravenCmd.pedal_down = True
        ravenCmd.header = header
        ravenCmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE

        rospy.loginfo('Published ravenCmd')
        raven_pub.publish(ravenCmd)

        rospy.sleep(.05)

def test_commandServo():
    rospy.init_node('command_servo',anonymous=True)
    armName = arm
    gripperControl = GripperControlClass(armName, tf.TransformListener())
    raven_pub = rospy.Publisher('/raven_command/tool/L', ToolCommandStamped)
    rospy.sleep(4)

    desPose = armDot

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():

        currPose = gripperControl.getGripperPose(MyConstants.Frames.Link0)
        deltaPose = Util.deltaPose(currPose, desPose)


        currPose = tfx.pose(currPose)
        deltaPose = tfx.pose(deltaPose)

        endPosition = currPose.position + deltaPose.position
        endQuatMat = currPose.orientation.matrix * deltaPose.orientation.matrix

        endPose = tfx.pose(endPosition, endQuatMat).msg.Pose()

        

        # original attempt at moving the arm
        # create the header
        header = Header()
        header.frame_id = MyConstants.Frames.Link0
        header.stamp = rospy.Time.now()
    
        # create the tool pose
        toolCmd = ToolCommand()
        toolCmd.pose = endPose
        toolCmd.pose_option = ToolCommand.POSE_ABSOLUTE
        toolCmd.grasp_option = ToolCommand.GRASP_OFF
    
        
        # create tool pose stamped
        toolCmdStamped = ToolCommandStamped()
        toolCmdStamped.header = header
        toolCmdStamped.command = toolCmd

        raven_pub.publish(toolCmdStamped)
        

        rate.sleep()

def test_commandJoints():
    rospy.init_node('command_raven',anonymous=True)
    armName = arm
    gripperControl = GripperControlClass(armName, tf.TransformListener())
    raven_pub = rospy.Publisher(MyConstants.RavenTopics.RavenCommand, RavenCommand)
    rospy.sleep(4)

    currJointStates = gripperControl.getJointStates()
    jointCommands = list()    
    
    #randomJointMovements = [random.uniform(-.02,.02) for _ in currJointStates]
    randomJointMovements = [.35 for _ in currJointStates]

    #code.interact(local=locals())

    while not rospy.is_shutdown():
        # original attempt at moving the arm
        # create the header
        header = Header()
        #header.frame_id = self.toolframe
        header.frame_id = MyConstants.Frames.Link0
        header.stamp = rospy.Time.now()
            
        # create the arm command
        armCmd = ArmCommand()
        armCmd.active = True

        for index in range(len(currJointStates)):
            if currJointStates[index].type == Constants.JOINT_TYPE_PITCH:
                jointCmd = JointCommand()
                jointCmd.command_type = JointCommand.COMMAND_TYPE_POSITION
                jointCmd.value = 0.0
                #jointCmd.value = currJointStates[index].position + randomJointMovements[index]

                armCmd.joint_types.append(currJointStates[index].type)
                armCmd.joint_commands.append(jointCmd)

        # create the raven command
        ravenCmd = RavenCommand()
        ravenCmd.arm_names.append(armName)
        ravenCmd.arms.append(armCmd)
        ravenCmd.pedal_down = True
        ravenCmd.header = header
        ravenCmd.controller = Constants.CONTROLLER_JOINT_POSITION

        #code.interact(local=locals())

        rospy.loginfo('Published ravenCmd')
        #print(armCmd)
        raven_pub.publish(ravenCmd)

        rospy.sleep(.05)
    


def test_jointPositions():
    rospy.init_node('command_raven',anonymous=True)
    armName = MyConstants.Arm.Left
    gripperControl = GripperControlClass(armName, tf.TransformListener())
    rospy.sleep(4)

    while not rospy.is_shutdown():
        rospy.loginfo(gripperControl.getJointStates())
        rospy.sleep(.5)

def test_angleBetween():
    rospy.init_node('command_raven',anonymous=True)
    armName = MyConstants.Arm.Left
    gripperControl = GripperControlClass(armName, tf.TransformListener())
    rospy.sleep(4)

    quat0 = armDot.msg.Pose().orientation
    quat1 = gripperControl.getGripperPose(frame=MyConstants.Frames.Link0).orientation

    between = Util.angleBetweenQuaternions(quat0,quat1)

    code.interact(local=locals())
    
    print(between)

if __name__ == '__main__':
    test_opencloseGripper(close=True,duration=5)
    #test_moveGripper()
    #test_moveGripperDelta()
    #test_moveGripperDeltaAR()
    #test_gripperPose()
    #test_down()
    #test_commandRaven()
    #test_rotation()
    #test_jointPositions()
    #test_commandJoints()
    #test_angleBetween()
    #test_servo()
    #test_commandServo()



"""
        print('moveGripper')
        print(endPose)
        print(startPose)
        print(tfx.pose(deltaPose))
        print(endPose.orientation.quaternion)
        print(startPose.orientation.quaternion)
        print(tfx.pose(deltaPose).orientation.quaternion)
"""


"""
    #endPose = tfx.pose(startPose.position + deltaPose.position, startPose.orientation.quaternion + deltaPose.orientation.quaternion)
    #endPose = tfx.pose(startPose.position + deltaPose.position, startPose.orientation.matrix*deltaPose.orientation.matrix)

    #endAngles = tfx.tb_angles(startPose.tb_angles.yaw_deg + deltaPose.tb_angles.yaw_deg, startPose.tb_angles.pitch_deg + deltaPose.tb_angles.pitch_deg, startPose.tb_angles.roll_deg + deltaPose.tb_angles.roll_deg)
    #endPose = tfx.pose(startPose.position + deltaPose.position, endAngles)
    # work on orientation combination

    #endPose = Util.addPoses(startPose, deltaPose)

    # convert to tfx format
    #startPose = tfx.pose(startPose)
    #endPose = tfx.pose(endPose)
"""



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

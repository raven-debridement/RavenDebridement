#!/usr/bin/env python

import roslib; roslib.load_manifest('RavenDebridement')
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
        
        self.player = MyTrajectoryPlayer(arms=self.armName)

        self.jointStates = list()
        rospy.Subscriber(MyConstants.RavenTopics.RavenState, RavenState, self.ravenStateCallback)

        rospy.sleep(1)
 
    def goToGripperPose(self, startPose, endPose, duration=None, speed=None, ignoreOrientation=False):
        """
        Given a startPose, move to endPose
        Both startPose and endPose are geometry_msgs.msg.Pose

        startPose is the current pose of the gripper WITH RESPECT TO MyConstants.Frames.Link0
        """
        startPose = tfx.pose(startPose)
        endPose = tfx.pose(endPose)

        # TEMP, until fix orientation issue
        #endPose = tfx.pose(endPose.position, tfx.tb_angles(0,90,0))
        #endPose = tfx.pose(endPosition, startPose.orientation)

        if ignoreOrientation:
            endPose.orientation = startPose.orientation

        self.player.clear_stages()
        self.player.add_pose_to_pose('goToGripperPose', startPose, endPose, duration=duration, speed=speed)
        

    def goToGripperPoseDelta(self, startPose, deltaPose, duration=None, speed=None, ignoreOrientation=False):
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
        #endQuatMat = deltaPose.orientation.matrix * startPose.orientation.matrix

        startQuat = tfx.tb_angles(startPose.orientation.matrix).quaternion
        endQuat = tfx.tb_angles(endQuatMat).quaternion

        if np.dot(startQuat, endQuat) < 0:
            rospy.loginfo('QUAT IS NEGATIVE')
            endQuat = list(endQuat)
            endQuat[-1] = -endQuat[-1]        

        endPose = tfx.pose(endPosition, endQuat)
        #endPose = tfx.pose(endPosition, endQuatMat)

        if ignoreOrientation:
            endPose.orientation = startPose.orientation
     
        # TEMP, until fix orientation issue
        #endPose = tfx.pose(endPosition, tfx.tb_angles(-90,90,0))
        #endPose = tfx.pose(endPosition, startPose.orientation)

        self.player.clear_stages()
        self.player.add_pose_to_pose('goToGripperPose',startPose,endPose,duration=duration, speed=speed)

            
    def goToGripperPoseUsingJoints(self, endPose, duration=None, speed=None):
        self.player.clear_stages()
        self.player.add_go_to_pose_using_joints('goToGripperPoseUsingJoints', self.jointStates, endPose, duration=duration, speed=speed)

    def start(self):
        # start running, no blocking
        return self.player.play(False)

    def pause(self):
        # pauses by clearing stages
        self.player.clear_stages()

    def isPaused(self):
        """
        True if no stages
        """
        return len(self.player.stages) == 0
        
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
        #self.listener.waitForTransform(frame, self.toolframe, rospy.Time.now(), rospy.Duration(5))
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
    gripperControl.start()
    rospy.sleep(2)
    rospy.loginfo('Setting the gripper')
    if close:
       gripperControl.closeGripper(duration=duration)
    else:
        gripperControl.openGripper(duration=duration)
    
    raw_input()

def test_moveToHome():
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(arm, listener)
    imageDetector = ARImageDetectionClass()
    rospy.sleep(4)
    
    desPose = imageDetector.getHomePose()
    
    gripperControl.start()
    
    rospy.loginfo('Press enter')
    raw_input()

    duration = 6
    gripperControl.goToGripperPose(gripperControl.getGripperPose(MyConstants.Frames.Link0), desPose, duration=duration)
    rospy.sleep(duration)

    raw_input()


def test_moveGripper():
    rospy.init_node('gc',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(arm, tf.TransformListener())
    rospy.sleep(4)

    gripperControl.start()

    currPose = tfx.pose(gripperControl.getGripperPose(frame=MyConstants.Frames.Link0))
    
    
    #desPose = armDot
    desPose = tfx.pose(currPose.position, tfx.tb_angles(-90,90,0))

    currPose = currPose.msg.Pose()
    desPose = desPose.msg.Pose()
    
    gripperControl.goToGripperPose(currPose,desPose,duration=4)

    while not rospy.is_shutdown():
        rospy.loginfo('spin')
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
    

    currPose = imageDetector.getGripperPose(arm)
    desPose = imageDetector.getObjectPose()

    
    
    deltaPose = Util.deltaPose(currPose, desPose, MyConstants.Frames.Link0, MyConstants.Frames.LeftTool)
    deltaPose.position.z += .03

    #code.interact(local=locals())
    #return
    

    #gripperControl.start()
    
    
    rospy.loginfo('Press enter')
    raw_input()

    
    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(MyConstants.Frames.Link0), deltaPose)

    rospy.loginfo('exit')
    raw_input()

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

def test_rotation():
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    tf_br = tf.TransformBroadcaster()
    gripperControl = GripperControlClass(arm, listener)
    imageDetector = ARImageDetectionClass()

    if arm == MyConstants.Arm.Left:
        frame = MyConstants.Frames.Link0
        toolframe = MyConstants.Frames.LeftTool
    else:
        frame = MyConstants.Frames.Link0
        toolframe = MyConstants.Frames.RightTool

    rospy.sleep(3)

    while not (imageDetector.hasFoundGripper(arm) and imageDetector.hasFoundObject()) and not rospy.is_shutdown():
        print imageDetector.hasFoundGripper(arm), imageDetector.hasFoundObject()
        print 'searching'
        rospy.sleep(1)

    print imageDetector.hasFoundGripper(arm), imageDetector.hasFoundObject()

    currPose = imageDetector.getGripperPose(arm)
    desPose = imageDetector.getObjectPose(toolframe)

    code.interact(local=locals())
    """
    desPose = tfx.pose([0,0,0], imageDetector.normal).msg.PoseStamped()

    
    while not listener.canTransform('tool_L', '0_link', rospy.Time()) and not rospy.is_shutdown():
        rospy.sleep(1)

    time = listener.getLatestCommonTime('tool_L', '0_link')

    print currPose.header.frame_id
    currPose.header.stamp = time
    desPose.header.stamp = time
    desPose.header.frame_id = '0_link'
    currPose = listener.transformPose('tool_L', currPose)
    desPose = listener.transformPose('tool_L', desPose) 
    """

    deltaPose = Util.deltaPose(currPose, desPose)
    deltaPose = tfx.pose([0,0,0],deltaPose.orientation).msg.PoseStamped()
    deltaPoseTb = tfx.tb_angles(deltaPose.pose.orientation)
    #deltaPose = tfx.pose([0,0,0],deltaPose.orientation).msg.Pose()
    print 'deltaPoseTb', deltaPoseTb
    
    """
    gripper_tb = tfx.tb_angles(currPose.pose.orientation)
    obj_tb = tfx.tb_angles(desPose.pose.orientation)
    print 'actual', gripper_tb
    print obj_tb
    deltaPoseTb = tfx.pose(Util.deltaPose(currPose, desPose)).orientation
    print 'deltaPose', deltaPoseTb
    endQuatMat = gripper_tb.matrix * deltaPoseTb
    print 'desOri', tfx.tb_angles(endQuatMat)

    time = listener.getLatestCommonTime('tool_L', '0_link')
    deltaPose.header.frame_id = '0_link'
    deltaPose.header.stamp = time
    deltaPose = listener.transformPose('tool_L', deltaPose)
    #deltaPose = tfx.pose([0,0,0], deltaPose.orientation).msg.Pose()
    deltaPoseTb = tfx.tb_angles(deltaPose.pose.orientation)
    print 'deltaPoseTb', deltaPoseTb
    currPose = tfx.pose(currPose)
    endQuatMat = currPose.orientation.matrix * deltaPoseTb.matrix
    print 'desOri', tfx.tb_angles(endQuatMat)
    """

    yaw = tfx.pose([0,0,0], tfx.tb_angles(deltaPoseTb.yaw_deg,0,0))
    pitch = tfx.pose([0,0,0], tfx.tb_angles(0,deltaPoseTb.pitch_deg,0))
    roll = tfx.pose([0,0,0], tfx.tb_angles(0,0,deltaPoseTb.roll_deg))

    rate = rospy.Rate(1)

    print "press enter"
    raw_input()

    """
    startPose = tfx.pose(gripperControl.getGripperPose(MyConstants.Frames.Link0))
    print 'start', startPose
    endQuatMat = startPose.orientation.matrix * tfx.tb_angles(deltaPose.orientation).matrix
    print tfx.tb_angles(endQuatMat)
    """

    
    gripperControl.start()
    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(frame), deltaPose, duration=4)
    
    """
    #gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(MyConstants.Frames.Link0), deltaPose, duration=4)
    print 'yaw', yaw
    raw_input()
    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(frame), yaw, duration=4)

    print 'pitch', pitch
    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(frame), pitch, duration=4)

    print 'roll', roll
    gripperControl.goToGripperPoseDelta(gripperControl.getGripperPose(frame), roll, duration=4)
    """

    while not rospy.is_shutdown():
        rospy.loginfo('loop')
        currPose = tfx.pose(gripperControl.getGripperPose(frame))
        rate.sleep()

def test_goToGripperPoseUsingJoints():
    rospy.init_node('gripper_control',anonymous=True)
    listener = tf.TransformListener()
    gripperControl = GripperControlClass(arm, listener)
    rospy.sleep(2)

    gripperControl.goToGripperPoseUsingJoints(tfx.pose([0,0,0]).msg.Pose())

if __name__ == '__main__':
    #test_opencloseGripper(close=False,duration=2.5)
    test_moveToHome()
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
    #test_goToGripperPoseUsingJoints()

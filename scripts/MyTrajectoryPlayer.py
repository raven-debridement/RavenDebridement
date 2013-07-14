#!/usr/bin/env python

import roslib
roslib.load_manifest('master-control')
import rospy
from geometry_msgs.msg import *
from math import *
from std_msgs.msg import Header
import copy
import sys, os.path


from raven_2_msgs.msg import *
from raven_2_trajectory.trajectory_player import TrajectoryPlayer, Stage


import tf
import tfx

import thread
import threading

import Util
import Constants as MyConstants

class MyTrajectoryPlayer(TrajectoryPlayer):
    def __init__(self,tf_listener=None,arms=['R']):
        #rospy.loginfo('Greg version!')
        self.arms = arms

        self.tf_listener = tf_listener
        if self.tf_listener is None:
            self.tf_listener = tfx.TransformListener.instance()
        rospy.loginfo('waiting for transform')
        for arm in arms:
            self.tf_listener.waitForTransform('/0_link','/tool_'+arm,rospy.Time(0),rospy.Duration(5))

        # ADDED, initializes the rest
        self.reset()
		
			
        self.pub_cmd = rospy.Publisher('raven_command', RavenCommand)
		
        self.state_sub = rospy.Subscriber('raven_state',raven_2_msgs.msg.RavenState,self._state_callback)
	
        self.header = Header()
        self.header.frame_id = '/0_link'
        
        self.timeout = Util.TimeoutClass(4)

        self.stageLock = threading.Lock()

        self.thread = None

        self.isThreadPlaying = False

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
        ravenPauseCmd.arm_names.append(self.arms)
        ravenPauseCmd.arms.append(armCmd)
        ravenPauseCmd.pedal_down = True
        ravenPauseCmd.header = header
        ravenPauseCmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE

        self.ravenPauseCmd = ravenPauseCmd
        
    def reset(self):
        """
        Added

        Resets player, allows for reuse
        """
        self.stages = []
        self.init_poses = {}
        self.start_time = None

        self.default_speed = .01

        for arm in self.arms:
            self.init_poses[arm] = tfx.pose(tfx.lookupTransform('/0_link','/tool_'+arm))
		
        self.current_state = None
        self.current_poses = {}
        self.current_grasps = {}
		
        # ADDED
        self.continuePlaying = True

    def stop_playing(self):
        """
        Sets flag to stop executing trajectory

        Returns False if times out, otherwise True
        """
        self.continuePlaying = False

        rate = rospy.Rate(50)
        self.timeout.start()
        while not self.timeout.hasTimedOut():
            if self.continuePlaying == True:
                return True

            rate.sleep()

        return False


    def clear_stages(self):
        """
        If playing, this effectively pauses the raven
        """
        self.stages = []

    def play(self,block,dry_run=False):
        """
        Intended use is to call play once at the beginning
        and then add stages to move it
        """
        if not self.isThreadPlaying:
            self.isThreadPlaying = True
            if block:
                return self.playThread(dry_run)
            else:
                thread.start_new_thread(self.playThread, (dry_run,))
                return True
            
        return True

    def playThread(self,dry_run=False):
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
        self.continuePlaying = True
		
        last_stage_ind = -1
        while not rospy.is_shutdown():
            if not dry_run and last_stage_ind != -1:
                sys.stdout.write("\r\x1b[K")
                sys.stdout.flush()
			
            if self.current_state.runlevel == 0:
                rospy.logerr('Raven in E-STOP, exiting')
                success = False
                break

            # ADDED
            if not self.continuePlaying:
                self.continuePlaying = True
                success = True
                break

            #self.stageLock.acquire()
            stages = self.stages

            # when a stage appears, set start_time
            if numStages == 0 and len(stages) > 0:
                self.start_time = rospy.Time.now()

            numStages = len(stages)
            stage_breaks = Stage.stage_breaks(stages)
            #self.stageLock.release()
            
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
            
                stage_changed = stage_ind != last_stage_ind
                last_stage_ind = stage_ind
            
                stage = self.stages[stage_ind]
            
                if stage.duration.is_zero():
                    t = 1
                else:
                    t = (dur_from_start - stage_breaks[stage_ind]).to_sec() / stage.duration.to_sec()
                
                if stage_changed:
                        rospy.loginfo("Stage #%i/%i [%4.1fs] %s",stage_ind+1,len(self.stages),stage.duration.to_sec(),stage.name)
                else:
                    sys.stdout.write("%.3f" % t)
                    if dry_run:
                        sys.stdout.write('\n')
                    sys.stdout.flush()
                
                if stage.is_pause:
                    stage.cb()
                    continue
            
                #self.header.stamp = now
            
                cmd = RavenCommand()
                #cmd.header = self.header
                cmd.pedal_down = True
            
                stage.cb(cmd,t)
            
                if stage_changed:
                    pass #print '\n\n' + str(cmd) + '\n\n'
            else:
                # no stages
                cmd = self.ravenPauseCmd

            self.header.stamp = now
            cmd.header = self.header
			
            if dry_run:
                print cmd
            else:
                self.pub_cmd.publish(cmd)
			
            rate.sleep()
            
        self.isThreadPlaying = False
        return success

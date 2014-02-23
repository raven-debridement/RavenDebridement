#!/usr/bin/env python

import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import IPython
from ipdb import launch_ipdb_on_exception

import functools, argparse
import cPickle as pickle
from collections import defaultdict

from raven_2_msgs.msg import RavenState
from geometry_msgs.msg import PoseStamped

from raven_2_utils import raven_constants

import operator

import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg
import pylab as pl

import tfx

ESTIMATED_POSE_TOPIC = '/estimated_gripper_pose'
PHASESPACE_POSE_TOPIC = '/phasespace_gripper_pose'

def get_robot_pose_ind(ts, robot_ts):
    return min(range(len(robot_ts)), key=lambda i: abs(robot_ts[i] - ts))

# Tracked pose topic is where we are publishing all of our latest poses

class DataRecorder(object):
    def __init__(self, arms=['L','R'], robot_pose_topic=raven_constants.RavenTopics.RavenState, tracked_pose_topic=PHASESPACE_POSE_TOPIC, estimated_pose_topic=ESTIMATED_POSE_TOPIC):
        self.arms = arms
        self.camera_to_robot_tf = None
        self.camera_poses = defaultdict(list)
        self.robot_poses = defaultdict(list)
        self.robot_joints = defaultdict(list)
        self.estimated_poses = defaultdict(list)
        
        self.is_recording = True
        self.converted = False
 
        self.camera_frame = None
        self.robot_frame = None
        
        self.camera_msgs = 0
        self.robot_msgs = 0
 
        rospy.Subscriber(robot_pose_topic, RavenState, self._raven_state_cb)
 
        for arm in self.arms:
            rospy.Subscriber(tracked_pose_topic+'_'+arm, PoseStamped, functools.partial(self._tracked_pose_cb,arm))
            rospy.Subscriber(estimated_pose_topic+'_'+arm, PoseStamped, functools.partial(self._estimated_pose_cb,arm)) # what is this

        
    def _tracked_pose_cb(self, arm, msg):
        if not self.is_recording:
            return
        if self.camera_frame is None:
            self.camera_frame = msg.header.frame_id
        
        if self.camera_to_robot_tf is None and self.camera_frame is not None and self.robot_frame is not None:
            self.camera_to_robot_tf = tfx.lookupTransform(self.robot_frame, self.camera_frame).array
        elif self.camera_to_robot_tf is not None:
            # store poses, put them in the correct frame
            self.camera_msgs = self.camera_msgs+1
            pose = tfx.pose(msg)
            self.camera_poses[arm].append((pose.stamp.seconds,self.camera_to_robot_tf.dot(pose.array)))
        
    def _estimated_pose_cb(self, arm, msg):
        if not self.is_recording:
            return
        
        pose = tfx.pose(msg)
        self.estimated_poses[arm].append((pose.stamp.seconds,pose.array))
        
    def _raven_state_cb(self, msg):
        if not self.is_recording:
            return
        
        if self.robot_frame is None:
            self.robot_frame = msg.header.frame_id
        
        with launch_ipdb_on_exception():
            for arm in self.arms:
                self.robot_msgs = self.robot_msgs+1
                arm_msg = [arm_ for arm_ in msg.arms if arm_.name == arm][0]
                pose = tfx.pose(arm_msg.tool.pose,header=msg.header)
                self.robot_poses[arm].append((pose.stamp.seconds,pose.array))
                joints = tuple(j.position for j in arm_msg.joints)
                self.robot_joints[arm].append((pose.stamp.seconds,joints))
    
    def write(self, fp, synchronized=True):
        print 'Writing to file...'
        
        if isinstance(fp,basestring):
            fp = open(fp,'w')
        
        d = {}
        for arm in self.arms:
            if len(self.camera_poses[arm]) == 0 or len(self.robot_poses[arm]) == 0:
                return False
        
        if synchronized:
            for key in [raven_constants.RavenDataKeys.CAM_ROBOT_TF_KEY, raven_constants.RavenDataKeys.ROBOT_JOINT_KEY,'estimated_poses', raven_constants.RavenDataKeys.CONVERTED_KEY]:
                d[key] = getattr(self,key)
            d[raven_constants.RavenDataKeys.CAM_TS_KEY] = {}
            d[raven_constants.RavenDataKeys.CAM_POSE_KEY] = {}
            d[raven_constants.RavenDataKeys.ROBOT_TS_KEY] = {}
            d[raven_constants.RavenDataKeys.ROBOT_POSE_KEY] = {}
            
            
            for arm in self.arms:
                camera_ts = [pose[0] for pose in self.camera_poses[arm]]
                camera_poses = [pose[1] for pose in self.camera_poses[arm]]
                robot_ts = [pose[0] for pose in self.robot_poses[arm]]
                robot_poses = [pose[1] for pose in self.robot_poses[arm]]
                
                # match first timestamps
                if camera_ts[0] > robot_ts[0]:
                    while len(robot_ts) > 0 and camera_ts[0] > robot_ts[0]:
                        robot_ts.pop(0)
                        robot_poses.pop(0)
                else:
                    while len(camera_ts) > 0 and camera_ts[0] < robot_ts[0]:
                        camera_ts.pop(0)
                        camera_poses.pop(0)
                        
                for arm in self.arms:
                    if len(self.camera_poses[arm]) == 0 or len(self.robot_poses[arm]) == 0:
                        return False
                
                # get corresponding robot indices
                robot_inds = []
                for ts in camera_ts:
                    robot_inds.append(get_robot_pose_ind(ts, robot_ts))
        
                robot_ts_save = []
                robot_poses_save = []
                for ind in robot_inds:
                    robot_ts_save.append(robot_ts[ind])
                    robot_poses_save.append(robot_poses[ind])
                
                d[raven_constants.RavenDataKeys.CAM_TS_KEY][arm] = camera_ts
                d[raven_constants.RavenDataKeys.CAM_POSE_KEY][arm] = camera_poses
                d[raven_constants.RavenDataKeys.ROBOT_TS_KEY][arm] = robot_ts_save
                d[raven_constants.RavenDataKeys.ROBOT_POSE_KEY][arm] = robot_poses_save
        else:
            for key in [raven_constants.RavenDataKeys.CAM_ROBOT_TF_KEY, raven_constants.RavenDataKeys.CAM_POSE_KEY, raven_constants.RavenDataKeys.ROBOT_POSE_KEY, raven_constants.RavenDataKeys.ROBOT_JOINT_KEY,'estimated_poses', raven_constants.RavenDataKeys.CONVERTED_KEY]:
                d[key] = getattr(self,key)
        
        pickle.dump(d, fp)
        return True
        
    def stop_recording(self, sort=False):
        # sort lists - for some reason they do not necessarily come in the correct order
        if sort:
            for arm in self.arms:
                self.camera_poses[arm].sort(key = operator.itemgetter(0))
                self.robot_poses[arm].sort(key = operator.itemgetter(0))
        self.is_recording = False
        
    def start_recording(self):
        self.is_recording = True
        
    def plot(self, arm):
        f, axarr = pl.subplots(3, sharex=True)
        f.set_size_inches(10, 8, forward=True)

        camera_ts = [pose[0] for pose in self.camera_poses[arm]]
        camera_poses = [pose[1] for pose in self.camera_poses[arm]]
        robot_ts = [pose[0] for pose in self.robot_poses[arm]]
        robot_poses = [pose[1] for pose in self.robot_poses[arm]]

        # get corresponding robot indices
        robot_inds = []
        for ts in camera_ts:
            robot_inds.append(get_robot_pose_ind(ts, robot_ts))

        robot_ts_plot = []
        robot_poses_plot = []
        for ind in robot_inds:
            robot_ts_plot.append(robot_ts[ind])
            robot_poses_plot.append(robot_poses[ind])

        camera_trans = np.array([pose[:3,3] for pose in camera_poses])
        robot_trans = np.array([pose[:3,3] for pose in robot_poses_plot])

        title = 'Errors in the raw tracked position estimate for %s arm' %(arm)
        axarr[0].set_title(title, size='large')
        for i in range(3):
            axarr[i].plot(camera_ts, camera_trans[:,i], 'b', label='camera')
            axarr[i].plot(robot_ts_plot, robot_trans[:,i], 'g', label='robot')
            i2coord = {0:'x', 1:'y', 2:'z'}
            axarr[i].set_ylabel(i2coord[i], fontsize=12)
        #pl.xlim(10, 160)
        pl.xlabel('time (s)', fontsize=12)
        axarr[2].legend(prop={'size':12}, loc='lower left', bbox_to_anchor=(0.75, 0.8)).draggable()
        pl.show()

def main():
    
    rospy.init_node('data_recorder',anonymous=True)
    parser = argparse.ArgumentParser()
    
    parser.add_argument('arm',nargs='?',default=None)
    parser.add_argument('--filename',nargs='?',default='data.pkl')
    parser.add_argument('--pause',action='store_true')
    parser.add_argument('--plot',nargs='?',default=False)
    parser.add_argument('--sort',nargs='?',default=True)
    parser.add_argument('--sync',nargs='?',default=True)
    
    args = parser.parse_args()
    
    arms = [args.arm]
    del args.arm
    
    if not arms[0]:
        arms = ['L', 'R']
        
    recorder = DataRecorder(arms=arms)
    
    if args.pause:
        recorder.stop_recording()
        while not rospy.is_shutdown():
            rospy.loginfo('Press enter to record')
            raw_input()
            recorder.start_recording()
            rospy.loginfo('Press enter to stop recording')
            raw_input()
            recorder.stop_recording()
    
    print 'Gathering data...'
    
    rospy.spin()
    recorder.stop_recording(sort=args.sort)
    if args.plot:
        recorder.plot(arms[0])
    recorder.write(args.filename, synchronized=args.sync)

if __name__ == '__main__':
    main()

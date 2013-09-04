#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import tf
import tf.transformations as tft
import tfx

from raven_2_msgs.msg import *

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants

import IPython

from collections import defaultdict
from functools import partial

class GripperPoseEstimator():
    """
    Used to estimate gripper pose by image processing
    """

    def __init__(self, arms = ['L','R']):
        self.arms = arms
        
        self.truthPose = defaultdict()
        self.calcPose = defaultdict()
        self.calcPoseAtTruth = defaultdict()
        self.estimatedPose = defaultdict(lambda: (None,None))
        
        self.adjustment = dict((arm,tfx.identity_tf()) for arm in self.arms)
        
        # tape callback
        if arm in self.arms:
            rospy.Subscriber(Constants.GripperTape.Topic+'_'+arm, PoseStamped, partial(self._truthCallback,arm))    
    
    def _truthCallback(self,arm,msg):
        truthPose = tfx.convertToFrame(msg, Constants.Frames.Link0)
        calcPose = self.calcPose[arm]
        prevTruthPose = self.truthPose[arm]
        prevCalcPose = self.calcPoseAtTruth[arm]
        
        if prevTruthPose is not None:
            deltaTruthPose = prevTruthPose.as_tf().inverse() * truthPose.as_tf()
            deltaCalcPose = prevCalcPose.as_tf().inverse() * truthPose.as_tf()
            adjustment = deltaTruthPose * deltaCalcPose.inverse()
            self.adjustment[arm] = adjustment
        
        self.truthPose[arm] = truthPose
        self.calcPoseAtTruth[arm] = calcPose
        self.estimatedPose[arm] = (truthPose,False)
    
    def _ravenStateCallback(self,msg):
        for arm in self.arms:
            arm_msg = [arm for arm in msg.arms if arm.name == arm][0]
            self.calcPose[arm] = tfx.pose(arm_msg.tool.pose,header=msg.header)
            self._updateEstimatedPose(arm)
    
    def _updateEstimatedPose(self,arm):
        calcPose = self.calcPose[arm]
        prevTruthPose = self.truthPose[arm]
        if prevTruthPose is None:
            return
        prevCalcPose = self.calcPoseAtTruth[arm]
        adjustment = self.adjustment[arm]
        
        deltaPose = adjustment * prevCalcPose.inverse() * calcPose
        estPose = tfx.pose(prevTruthPose.as_tf() * deltaPose,frame=calcPose.frame,stamp=calcPose.stamp)
        self.estimatedPose[arm] = (estPose,True)
    
    def getGripperPose(self, armName, full=False):
        """
        armName must be from Constants.Arm
        """
        pose, isEst = self.estimatedPose[armName]
        if full:
            return pose, isEst
        else:
            return pose
      

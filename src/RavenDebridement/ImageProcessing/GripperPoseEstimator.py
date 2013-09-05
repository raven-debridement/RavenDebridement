#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import tf
import tf.transformations as tft
import tfx

from raven_2_msgs.msg import *
from geometry_msgs.msg import PoseStamped

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
        for arm in self.arms:
            rospy.Subscriber(Constants.GripperTape.Topic+'_'+arm, PoseStamped, partial(self._truthCallback,arm))
            
        rospy.Subscriber(Constants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)   
    
    def _truthCallback(self,arm,msg):
        truthPose = tfx.convertToFrame(msg, Constants.Frames.Link0)
        #truthPose = Util.convertToFrame(msg, Constants.Frames.Link0)
        calcPose = self.calcPose.get(arm)
        prevTruthPose = self.truthPose.get(arm)
        prevCalcPose = self.calcPoseAtTruth.get(arm)
        
        if prevTruthPose is not None and prevCalcPose is not None:
            deltaTruthPose = prevTruthPose.as_tf().inverse() * truthPose.as_tf()
            deltaCalcPose = prevCalcPose.as_tf().inverse() * calcPose.as_tf()
            adjustment = deltaTruthPose * deltaCalcPose.inverse()#adjustment = deltaCalcPose.inverse()
            self.adjustment[arm] = adjustment
        
        self.truthPose[arm] = truthPose
        self.calcPoseAtTruth[arm] = calcPose
        self.estimatedPose[arm] = (truthPose,False)
    
    def _ravenStateCallback(self,msg):
        for arm in self.arms:
            arm_msgs = [msg_arm for msg_arm in msg.arms if msg_arm.name == arm]
            if len(arm_msgs) > 0:
                arm_msg = arm_msgs[0]
            else:
                return
            self.calcPose[arm] = tfx.pose(arm_msg.tool.pose,header=msg.header)
            self._updateEstimatedPose(arm)
    
    def _updateEstimatedPose(self,arm):
        calcPose = self.calcPose[arm]
        prevTruthPose = self.truthPose.get(arm)
        if prevTruthPose is None:
            return
        prevCalcPose = self.calcPoseAtTruth[arm]
        adjustment = self.adjustment[arm]
        
        deltaPose = adjustment * prevCalcPose.inverse().as_tf() * calcPose.as_tf()
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
        
        
def test():
    from visualization_msgs.msg import Marker
    
    rospy.init_node('testGripperPoseEstimator',anonymous=True)
    rospy.sleep(2)
    arms = Constants.Arm.Both
    gpe = GripperPoseEstimator(arms)
    rospy.sleep(2)
    
    estimatePub = dict()
    truthPub = dict()
    prevPoseAndIsEstimate = dict()
    
    for arm in arms:
        estimatePub[arm] = rospy.Publisher('estimated_gripper_pose_%s'%arm,PoseStamped)
        truthPub[arm] = rospy.Publisher('truth_gripper_pose_%s'%arm,PoseStamped)
        
        
    printEvery = Util.Timeout(1)
    printEvery.start()
    
    while not rospy.is_shutdown():
        for arm in arms:
            truthPose = gpe.truthPose.get(arm)
            estimatedPoseisEstimate = gpe.estimatedPose.get(arm)
            
            if truthPose is not None:
                #print 'Publishing truthPose %s' % arm
                truthPub[arm].publish(truthPose.msg.PoseStamped())
            if estimatedPoseisEstimate is not None:
                estimatedPose, isEstimate = estimatedPoseisEstimate
                #print 'Publish estimatedPose %s' % arm
                estimatePub[arm].publish(estimatedPose.msg.PoseStamped())
                
            if estimatedPoseisEstimate is not None and prevPoseAndIsEstimate.has_key(arm):
                prevPose, prevIsEstimate = prevPoseAndIsEstimate[arm]
                if prevIsEstimate is True and isEstimate is False:
                    deltaPose = Util.deltaPose(prevPose, estimatedPose)
                    print 'deltaPose between last estimate and current truth'
                    print deltaPose
                #print 'truthPose_{0}: {1}'.format(arm,truthPose)
                #print 'isEstimate: {0}'.format(isEstimate)
                #print 'estimatedPose_{0}: {1}'.format(arm,estimatedPose)
            
            if estimatedPoseisEstimate is not None:
                prevPoseAndIsEstimate[arm] = (estimatedPose, isEstimate)
        
        rospy.sleep(.02)
        
if __name__ == '__main__':
    test()
      

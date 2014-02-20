'''
Created on Feb 18, 2014

@author: jmahler
'''
#!/usr/bin/env python

import roslib
import rospy
roslib.load_manifest('RavenDebridement')
import random
import tfx
import argparse
import time

import numpy as np
from sklearn.mixture import GMM


from raven_2_trajectory.raven_arm import RavenArm
from raven_2_trajectory.raven_planner import RavenPlanner
from raven_2_control import kinematics
from raven_2_vision import GripperPoseEstimator
from raven_2_utils import raven_constants
from raven_2_utils import raven_util

import IPython

DEFAULT_GRASP = 1.2
NUM_STEPS = 50

class RepeatibilityTester():
    def __init__(self, armName, numPoses=1000, speed=0.01):
        self.ravenArm = RavenArm(armName, defaultPoseSpeed=speed)
        self.ravenPlanner = RavenPlanner(armName)
        self.gripperPoseEstimator = GripperPoseEstimator(armName)
        self.arm = armName
        
        if armName == 'R':
            self.homePose = raven_constants.HomePose.Right
        else:
            self.homePose = raven_constants.HomePose.Left
        
        self.grasp = DEFAULT_GRASP
        self.n_steps = NUM_STEPS
        self.numPoses = numPoses
        
        
        # sample values
        n_components = 3
        means =  np.array([[-1], [0], [3]])
        covariances = np.array([[1.5], [1], [0.5]]) ** 2
        weights = np.array([0.3, 0.5, 0.2])
        
        self.gmm = GMM(n_components)
        self.gmm.means_ = np.array(means)
        self.gmm.covars_ = np.array(covariances)
        self.gmm.weights_ = np.array(weights)
        
    def sample(self, number_of_samples=1):
        
        return self.gmm.sample(number_of_samples)
   
    
    def move(self, homePose, maxPoses=1000):
        rospy.loginfo('Generating pose samples...')
        
        self.ravenArm.goToGripperPose(self.homePose)
        self.ravenArm.setGripper(self.grasp, duration=1.0)
       
        for i in range(self.numPoses):
            poseErrors = []
            for j in range(self.numTries):
                [targetPose, deltaPose] = self.sample()
                inverseDeltaPose = tfx.inverse_tf(deltaPose)
                prev_pose = raven_util.endPose(targetPose, inverseDeltaPose)
                
                IPython.embed()
                try:
                    deltaPoseTraj = self.ravenPlanner.getTrajectoryFromPose(self.ravenArm.name, prev_pose, startPose=self.homePose, n_steps=self.n_steps)
                    deltaPoseTraj.append(deltaPose)
                except RuntimeError as e:
                    rospy.loginfo(e)
                    return 'IKFailure'
            
                self.ravenArm.executeDeltaPoseTrajectory(deltaPoseTraj, block=True)
                rospy.sleep(0.5)
                
                actualPose = tfx.pose(self.gripperPoseEstimator.getGripperPose(self.armName)).copy()
                errorDeltaPose = raven_util.deltaPose(actualPose, targetPose)
                
                translationError = errorDeltaPose.translation
                angleError = errorDeltaPose.tb_angles
                errorVector = np.r_[translationError, angleError]
                
                poseErrors.append(errorVector)
            
            # extract translation, rotation and run statistics
                
            
                
        
    def run(self):
        self.ravenArm.start()
        
        rospy.loginfo('Press enter to start')
        raw_input()
        
        homePose = self.homePose
        self.move(homePose, self.maxPoses)
                
        if rospy.is_shutdown():
            return
                
        self.ravenArm.stop()
        
if __name__ == '__main__':
    rospy.init_node('GridMove', anonymous=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('arm',nargs='?',default='L')
    parser.add_argument('--speed',nargs='?',default=0.01)
    args = parser.parse_args(rospy.myargv()[1:])
    arm_side = args.arm
    del args.arm
    speed = float(args.speed)
    del args.speed
    
    rospy.sleep(2)
    gm = GridMove(arm_side, rand, testAngles=False, maxPoses=100, speed=speed)
    gm.run()        
        
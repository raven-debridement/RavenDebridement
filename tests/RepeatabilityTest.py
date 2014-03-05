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
import scipy.io

from raven_2_trajectory.raven_arm import RavenArm
from raven_2_trajectory.raven_planner import RavenPlanner
from raven_2_vision.gripper_pose_estimator import GripperPoseEstimator
from raven_2_control import kinematics
from raven_2_calibration import error_model
from raven_2_calibration import transformations
from raven_2_utils import raven_constants
from raven_2_utils import raven_util

from geometry_msgs.msg import PoseStamped, TransformStamped

import IPython

DEFAULT_GRASP = 1.2

GMM_MODEL_NAME = 'gmmSave'
NUM_COMPONENTS_INDEX = 0
PROB_COMPONENTS_INDEX = 1
MU_INDEX = 2
SIGMA_INDEX = 3

POSE_KEY = 'pose'
MEAN_KEY = 'mean'
MEDIAN_KEY = 'median'
RMS_KEY = 'rms'
MAX_KEY = 'max'
MIN_KEY = 'min'
COV_KEY = 'covariance'

PHASESPACE_POSE_TOPIC = '/phasespace_gripper_pose'

from RavenDebridement import data_recorder

class PoseError(object):
    def __init__(self, medianError, meanError, rmsError, minError, maxError, stdError):
        self.medianError = medianError
        self.meanError = meanError
        self.rmsError = rmsError
        self.minError = minError
        self.maxError = maxError
        self.stdError = stdError
        
    def __str__(self):
        output = ''
        output += 'Mean:' + str(self.meanError) + '\n'
        output += 'Median:' + str(self.medianError) + '\n'
        output += 'RMS:' + str(self.rmsError) + '\n'
        output += 'Min:' + str(self.minError) + '\n'
        output += 'Max:' + str(self.maxError) + '\n'
        output += 'Std Error:' + str(self.stdError) + '\n'
        return output

class RepeatibilityTester():
    def __init__(self, armName, errorModelFile, outputMatFile='repeatability.mat', numPoses=10, numTries=1, speed=0.01):
        self.speed = speed
        self.arm = armName
        self.outputMatFile = outputMatFile

        self.errorModel = None
        if armName == 'L':
            self.errorModel = error_model.RavenErrorModel(leftModelFile=errorModelFile)
        else:
            self.errorModel = error_model.RavenErrorModel(rightModelFile=errorModelFile)

        self.ravenArm = RavenArm(armName, defaultPoseSpeed=speed)
        self.ravenPlanner = RavenPlanner(armName, errorModel= self.errorModel , addNoise=False)
        self.gripperPoseEstimator = GripperPoseEstimator(armName)
        
        receptaclePose = tfx.pose([-.060, .010, -.135], tfx.tb_angles(-90,90,0),frame='0_link')
        if armName == 'R':
            self.holdingPose = receptaclePose + [-.04, -.03, .03]
            self.homePose = raven_constants.HomePose.Right
        else:
            self.holdingPose = receptaclePose + [.04, -.03, .03]
            self.homePose = raven_constants.HomePose.Left
        
        self.homePoseConverted, dummy = self.errorModel.predictSinglePose(self.arm, self.homePose, self.homePose)
        
        
        self.grasp = DEFAULT_GRASP
        self.numPoses = numPoses
        self.numTries = numTries
        self.observedPose = None

        self.allErrors = np.zeros((0,6))
        
        self.data = {}
        self.data[POSE_KEY] = []
        self.data[MEAN_KEY] = []
        self.data[MEDIAN_KEY] = []
        self.data[RMS_KEY] = []
        self.data[MIN_KEY] = []
        self.data[MAX_KEY] = []
        self.data[COV_KEY] = []
        
        self.running = False
        self.trajNum = 0
        self.filename = 'test_%d.pkl'

        #p, d = self.sample()
        #IPython.embed()
        x_inc = .04
        y_inc = .08
        x_levels = 20
        y_levels = 40
        angles = [tfx.tb_angles(-90,90,0)]
        if armName == 'L':
            foamHome = tfx.pose([-.03,-.02,-.16],tfx.tb_angles(-90,90,0), frame='0_link')
        else:
            foamHome = tfx.pose([-.08,-.02,-.16],tfx.tb_angles(-90,90,0), frame='0_link')
        self.foamPoseIndex = 0
        self.foamPoseGrid = self.getGrid(foamHome, x_inc, y_inc, x_levels, y_levels, angles)
        
        random.seed(4000)
        random.shuffle(self.foamPoseGrid)

        #print raven_constants.GripperTape.Topic + '_' + armName
        rospy.Subscriber(PHASESPACE_POSE_TOPIC+ '_' + armName, PoseStamped, self.cameraPoseCallback)
    
    def cameraPoseCallback(self, msg):
        
        # convert to 0 link and post
        try:
            posePhasespace = tfx.pose(msg)
            self.observedPose = tfx.convertToFrame(posePhasespace, '0_link', wait=1.0)
        except:
            failCount = 1

    def getPhasespacePose(self):
        tries = 0
        while self.observedPose is None and tries < 200:
            rospy.sleep(0.01)
            tries = tries + 1
        ret = None
        if tries < 200:
            ret = self.observedPose.copy()
        self.observedPose = None
        return ret
    
    def startDataCollection(self):
        if not self.running:
            rospy.loginfo('Starting data collection...')
            self.running = True
            self.collectionFinished = False
            self.dataRecorder = data_recorder.DataRecorder(self.arm)
        
    def stopDataCollection(self, filename, sort=False, plot=False, targetPose=None):
        sync = True
        if self.running:
            rospy.loginfo('Stopping data collection')
            self.dataRecorder.stop_recording(sort)
            if plot:
                self.dataRecorder.plot(self.armName)
            if self.dataRecorder.write(filename, synchronized=sync, targetPose=targetPose):
                self.trajNum = self.trajNum+1
            self.running = False
            self.collectionFinished = True
            rospy.loginfo('Saved data')

    def getGrid(self, homePose, x_inc, y_inc, x_levels, y_levels, angles):
        grid = []
            
        for i in xrange(x_levels):
            for j in xrange(y_levels):
                for k in xrange(len(angles)):
                    # positive x
                    pose = homePose + [float(i) * (x_inc / float(x_levels)), float(j) * (-y_inc / y_levels), 0]
                    pose.tb_angles = angles[k]
                    grid += [pose.copy()]
                    # negative x
                    pose = homePose + [-float(i) * (x_inc / float(x_levels)), float(j) * (-y_inc / y_levels), 0]
                    pose.tb_angles = angles[k]
                    grid += [pose.copy()]
                    
        return grid
    
    def computeErrors(self, poseErrors):
        meanError = np.mean(np.abs(poseErrors), axis=0)
        medianError = np.median(np.abs(poseErrors), axis=0)
        rmsError = np.sqrt(np.mean(np.square(poseErrors), axis=0))
        minError = np.min(np.abs(poseErrors), axis=0)
        maxError = np.max(np.abs(poseErrors), axis=0)
        stdError = np.cov(np.abs(poseErrors).T)
        return PoseError(meanError, medianError, rmsError, minError, maxError, stdError)
    
    def storeErrors(self, repeatabilityErrors):
        
        for (targetPose, poseError) in repeatabilityErrors:
            self.data[POSE_KEY].append(targetPose)
            self.data[MEAN_KEY].append(poseError.meanError)
            self.data[MEDIAN_KEY].append(poseError.medianError)
            self.data[MIN_KEY].append(poseError.minError)
            self.data[MAX_KEY].append(poseError.maxError)
            self.data[RMS_KEY].append(poseError.rmsError)
            self.data[COV_KEY].append(poseError.stdError)
        
        scipy.io.savemat(self.outputMatFile, self.data)
        
    def sample(self):
        delta = tfx.pose([0,0,0], frame='0_link')
        pose = self.foamPoseGrid[self.foamPoseIndex]
        self.foamPoseIndex = self.foamPoseIndex+1

        return pose, delta
    
    def move(self, targetPose = None):
        poseErrors = np.zeros((0,6))
        deltaPose = tfx.pose([0,0,0])
        if targetPose == None:
            targetPose, deltaPose = self.sample()
        
        #cameraEndPose, dummy = self.sample()

        for j in range(self.numTries):
            if rospy.is_shutdown():
                return None
            try:
                # reset
                self.ravenArm.goToGripperPose(self.holdingPose)
                self.ravenArm.setGripper(self.grasp, duration=1.0)
                rospy.sleep(1)
                
                startPose = None
                while startPose is None:
                    if rospy.is_shutdown():
                        return None
                    startPose = self.getPhasespacePose()
                
                n_steps = 20
                robotStartPose = self.gripperPoseEstimator.getGripperPose(self.arm).copy()

                print robotStartPose
                # used for error model
                # cameraStartPose = tfx.pose([-0.107, 0.9899, 0.0930, -0.0455, -0.2510, 0.0636, -0.9659, -0.019, -0.9621, -0.1267, 0.2416, -0.0949, 0, 0, 0, 1.0000])
                # cameraEndPose   = tfx.pose([0.1263, 0.9422, 0.3103, 0.0063, -0.3207, 0.3348, -0.8860, -0.0304, -0.9387, 0.0124, 0.3445, -0.1530, 0, 0, 0, 1.0000])

                # startPose = tfx.pose([0.0008, 1.000, 0.0003, -0.0200, 0.0012, 0.0003, -1.0000, -0.0197, -1.000, 0.0008, -0.0012, -0.1051, 0, 0, 0, 1.0000])
                # endPose = tfx.pose([-0.0000, 1.0000, 0.0000, 0.0355, 0.0008, 0.0000, -1.0000, -0.0200, -1.000, -0.0000, -0.0008, -0.1580, 0, 0, 0, 1.0000])

                # cameraStartPose.frame = '0_link'
                # cameraEndPose.frame = '0_link'
                # startPose.frame = '0_link'
                # endPose.frame = '0_link'

                # good trajectory
                cameraStartPose = tfx.pose([-0.1075, 0.9868, 0.1208, -0.0468, -0.3099, 0.0822, -0.9472, -0.0201, -0.9447, -0.1393, 0.2970, -0.0968,  0, 0, 0, 1.0000])
                #cameraEndPose   = tfx.pose([-0.1163, 0.9858, 0.1207, -0.1170, -0.2292, 0.0916, -0.9691, -0.0582, -0.9664, -0.1404, 0.2153, -0.1489, 0, 0, 0, 1.0000])
                #cameraEndPose   = tfx.pose([0, 1, 0, -0.02982, 0, 0, -1, -0.07282, -1, 0, 0, -0.162, 0, 0, 0, 1.0000])

                #startPose = tfx.pose([-0.0003, 1.0000, -0.0001, -0.0200, 0.0003, -0.0001, -1.0000, -0.0200, -1.0000, -0.0003, -0.0003, -0.1050, 0, 0, 0, 1.0000])
                endPose   = tfx.pose([-0.0001, 1.0000, -0.0000, -0.0755, -0.0019, -0.0000, -1.0000, -0.0605, -1.0000, -0.0001, 0.0019, -0.1698, 0, 0, 0, 1.0000])

                cameraEndPose   = tfx.pose([-0.0825, -0.0603, -0.1487], [0.5, 0.5, -0.5, 0.5])
                startPose = tfx.pose([-0.0995, -0.0213, -0.1064], [0.4936, 0.5026, -0.5029, 0.5007])

                cameraStartPose.frame = '0_link'
                cameraEndPose.frame = '0_link'
                startPose.frame = '0_link'
                endPose.frame = '0_link'

                '''
                startPose = tfx.pose([-0.0210, 0.9996, -0.0188, -0.0204, 0.0127, -0.0191, -0.9997, -0.0209, -0.9997, -0.0208, 0.0131, -0.1053, 0, 0,0, 1])
                #startPose.frame = '0_link'

                #[-0.0521, 0.9986, 0.0037, 0.0082, -0.0099, 0.0032, -0.9999, -0.0886, -0.9986, -0.0521, 0.0098, -0.1693, 0,0,0, 1.000]
                cameraStartPose = tfx.pose([-0.0943, 0.9917, 0.0874, -0.0458, -0.2127, 0.0657, -0.9749, -0.0188, -0.9726, -0.1105, 0.2047, -0.0950, 0, 0, 0, 1])
                cameraStartPose.frame = '0_link'

                cameraEndPose = tfx.pose([-0.0559, 0.9882, 0.1424, -0.0357, -0.3918, 0.1094, -0.9135, -0.0839, -0.9183, -0.1069, 0.3811, -0.1343, 0,0,0,1])
                cameraEndPose.frame = '0_link'

                endPose = tfx.pose([0, 1.000, 0, 0.0025, 0, 0, -1.0, -0.0733, -1, 0, 0, -0.1510, 0,0,0,1])
                endPose.frame = '0_link'
                #IPython.embed()
                '''
                
                (correctedStartPose, deltaPoseTraj) = self.ravenPlanner.getFullTrajectoryFromPose(self.ravenArm.name, cameraEndPose,
                    startPose=startPose, n_steps=n_steps, approachDir=np.array([0,.1,.9]), speed=self.speed, correctTrajectory=True)

            except RuntimeError as e:
                rospy.loginfo(e)
                return 'IKFailure'  
            
            print 'Executing trajectory attempt', j
            #self.startDataCollection()
            #rospy.sleep(1)

            #rospy.sleep(2)
            self.ravenArm.executeDeltaPoseTrajectory(deltaPoseTraj, block=True, startPose=correctedStartPose)
        
            #rospy.sleep(5)
            #self.stopDataCollection(self.filename %(self.trajNum), plot=False, targetPose=cameraEndPose)

            rospy.sleep(2)
            actualPose = self.getPhasespacePose()
            errorDeltaPose = raven_util.deltaPose(actualPose, cameraEndPose)
            translationError = errorDeltaPose.translation.array
            angleError = np.array(transformations.euler_from_matrix(errorDeltaPose.matrix[:3,:3]))
            
            print
            print 'ERROR'
            print translationError
            print angleError
            print
            errorVector = np.c_[[translationError], [angleError]]
            poseErrors = np.r_[poseErrors, errorVector]
            rospy.sleep(1)

        return (targetPose.matrix, poseErrors)
        
        
    def run(self):
        self.ravenArm.start()
        
        rospy.loginfo('Press enter to start')
        raw_input()
        
        rospy.loginfo('Generating pose samples...')
        
        self.ravenArm.goToGripperPose(self.holdingPose)
        self.ravenArm.setGripper(self.grasp, duration=1.0)
        repeatabilityErrors = []
        targetPose = None
        
        for i in range(self.numPoses):
            poseErrorTuple = self.move(targetPose=targetPose)
            
            poseErrorSummary = self.computeErrors(poseErrorTuple[1])
            print "Pose errors"
            print poseErrorSummary

            repeatabilityErrors.append((poseErrorTuple[0], poseErrorSummary))
            self.allErrors = np.r_[self.allErrors, poseErrorTuple[1]]

            if rospy.is_shutdown():
                break

        print 'Done with move'

        # compute statistics of all tries
        poseErrorSummary = self.computeErrors(self.allErrors)
        repeatabilityErrors.append((tfx.pose([0,0,0]), poseErrorSummary))

        self.storeErrors(repeatabilityErrors)
        print 'Done Saving Errors'
        self.ravenArm.stop()
        
if __name__ == '__main__':
    rospy.init_node('RepeatabilityTest', anonymous=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('arm',nargs = '?',default = 'L')
    parser.add_argument('--model',nargs = '?',default = None)
    parser.add_argument('--mixture',nargs = '?',default = None)
    parser.add_argument('--speed',nargs = '?',default = 0.01)
    args = parser.parse_args(rospy.myargv()[1:])
    
    arm_side = args.arm
    del args.arm
    errorModelFile = args.model
    del args.model
    gmmFile = args.mixture
    del args.mixture
    speed = float(args.speed)
    del args.speed
    
    rospy.sleep(2)
    gm = RepeatibilityTester(arm_side, errorModelFile, speed=speed)
    gm.run()        
    
    """
    import scipy.io
    dataMat = scipy.io.loadmat('trajectory_data_phasespace5/move_towards_foam_data_L_0_L.mat')
    
    errorModel = error_model.RavenErrorModel(arm_side, errorModelFile)
    
    raw = []
    predicted = []
    expected = []
    
    for i in range(1, dataMat['c'].shape[0]):
        curPose = np.reshape(dataMat['c'][i,:,:], (4,4))
        prevPose = np.reshape(dataMat['c'][i-1,:,:], (4,4))
        robotPose = np.reshape(dataMat['r'][i,:,:], (4,4))
        
        curPoseTfx = tfx.pose(curPose)
        prevPoseTfx = tfx.pose(prevPose)
    
        gpPred, sysPred = errorModel.predictSinglePose(arm_side, curPoseTfx, prevPoseTfx)
        
        raw.append(curPose)
        predicted.append(gpPred)
        expected.append(robotPose)
    
    errors = errorModel.componentwiseError(predicted, expected)
    
    IPython.embed()
    """    
        
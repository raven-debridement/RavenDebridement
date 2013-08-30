#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math
from math import *

import numpy as np

import tf
import tf.transformations as tft
import tfx


from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion
from raven_pose_estimator.srv import ThreshRed

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.ImageProcessing.ImageDetection import ImageDetector


import IPython

def testCheckPickup(armName=Constants.Arm.Right):
    rospy.init_node('testCheckPickup',anonymous=True)
    imageDetector = ImageDetector()
    rospy.sleep(4)
    
    rospy.loginfo('Press enter to get {0} gripper point'.format(armName))
    raw_input()
    
    gripperPose = imageDetector.getGripperPose(armName)
    convGripperPose = tfx.pose(Util.convertToFrame(gripperPose, Constants.Frames.Link0))
    lastGripperPoint = convGripperPose.position.msg.PointStamped()
    
    lastGripperPoint.point.z += 0
    
    rospy.loginfo('Press enter to thresh for red at last gripper point')
    raw_input()
    
    serviceName = '{0}_{1}'.format(Constants.Services.isFoamGrasped, armName)

    try:
        rospy.wait_for_service(serviceName, timeout=5)
        foamGraspedService = rospy.ServiceProxy(serviceName, ThreshRed)
        isFoamGrasped = foamGraspedService(lastGripperPoint).output
        
        if isFoamGrasped == 1:
            rospy.loginfo('Found red piece!')
        else:
            rospy.loginfo('Did not find red piece!')
    except e:
        rospy.loginfo('Thresh service failed: {0}'.format(e))
        
    rospy.spin()

if __name__ == '__main__':
    testCheckPickup()
#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math

from RavenDebridement.srv import InvKinSrv
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.RavenCommand.RavenPlanner2 import RavenPlanner
from RavenDebridement.Utils import Constants as MyConstants

from raven_2_msgs.msg import *

import numpy as np

import tfx

import code

def invKinClient():
    rospy.init_node('inv_kin_client',anonymous=True)
    rospy.sleep(4)
    ravenArm = RavenArm(MyConstants.Arm.Right)
    rospy.sleep(4)
    
    start = tfx.pose(ravenArm.getGripperPose())
    end = start + [0,-.05, 0]
    
    try:
        rospy.wait_for_service('inv_kin_server',timeout=5)
        inv_kin_service = rospy.ServiceProxy('inv_kin_server', InvKinSrv)
        arm = Constants.ARM_TYPE_GREEN
        pose = tfx.pose(ravenArm.getGripperPose())
        grasp = ravenArm.ravenController.currentGrasp
        rospy.loginfo('Find ik for ' + str(pose))
        respStart = inv_kin_service(arm, grasp, start)
        respEnd = inv_kin_service(arm, grasp, end)
        rospy.loginfo('Called service')
    except (rospy.ServiceException, rospy.ROSException) as e:
        print "Service call failed: %s"%e
        return

    startJoints = []
    for joint in respStart.joints:
        if joint.type in RavenPlanner.rosJointTypes:
            startJoints.append(joint.position)
        #print("({0},{1})".format(joint.type,joint.position))
        
    endJoints = []
    for joint in respEnd.joints:
        if joint.type in RavenPlanner.rosJointTypes:
            endJoints.append(joint.position)
        #print("({0},{1})".format(joint.type,joint.position))
        
    startJoints = np.array(startJoints)
    endJoints = np.array(endJoints)
    timeSteps = 50
    for i in range(timeSteps):
        print(list(startJoints + (float(i)/float(timeSteps)*(endJoints-startJoints))))


if __name__ == '__main__':
    invKinClient()

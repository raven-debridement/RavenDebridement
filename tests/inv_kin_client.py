#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math

from RavenDebridement.srv import InvKinSrv
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.Utils import Constants as MyConstants

from raven_2_msgs.msg import *

import tfx

import code

def invKinClient():
    rospy.init_node('inv_kin_client',anonymous=True)
    rospy.sleep(4)
    ravenArm = RavenArm(MyConstants.Arm.Left)
    rospy.sleep(4)
    try:
        rospy.wait_for_service('inv_kin_server',timeout=5)
        inv_kin_service = rospy.ServiceProxy('inv_kin_server', InvKinSrv)
        arm = Constants.ARM_TYPE_GOLD
        pose = tfx.pose(ravenArm.getGripperPose())
        grasp = ravenArm.ravenController.currentGrasp
        rospy.loginfo('Find ik for ' + str(pose))
        resp = inv_kin_service(arm, grasp, pose)
        rospy.loginfo('Called service')
    except (rospy.ServiceException, rospy.ROSException) as e:
        print "Service call failed: %s"%e
        return


    for joint in resp.joints:
        rospy.loginfo("(%d, %f)",joint.type,(180.0/math.pi)*(joint.position))

if __name__ == '__main__':
    invKinClient()

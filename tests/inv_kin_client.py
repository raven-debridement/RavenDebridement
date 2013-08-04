#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math

from RavenDebridement.srv import InvKinSrv

from raven_2_msgs.msg import *

import tfx

import code

def invKinClient():
    rospy.init_node('inv_kin_client',anonymous=True)
    try:
        rospy.wait_for_service('inv_kin_server',timeout=5)
        inv_kin_service = rospy.ServiceProxy('inv_kin_server', InvKinSrv)
        arm = Constants.ARM_TYPE_GREEN
        angle = tfx.tb_angles(-69.8, 68.7, 38.9) # -20.2, 84.0, 50.5
        pose = tfx.pose([-.136,-.017,-.068], angle).msg.Pose()
        rospy.loginfo('Find ik for ' + str(pose))
        resp = inv_kin_service(arm, pose)
        rospy.loginfo('Called service')
    except (rospy.ServiceException, rospy.ROSException) as e:
        print "Service call failed: %s"%e
        return


    for joint in resp.joints:
        rospy.loginfo("(%d, %f)",joint.type,(180.0/math.pi)*(joint.position))

if __name__ == '__main__':
    invKinClient()

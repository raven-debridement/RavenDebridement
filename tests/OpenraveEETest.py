import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from math import *
import copy
import sys, os

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *


import openravepy as rave

import tf
import tfx
import numpy as np

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants as MyConstants
from RavenDebridement.ImageProcessing.ImageDetection import ImageDetector
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.RavenCommand.RavenPlanner2 import RavenPlanner

def test():
    rospy.init_node('test',anonymous=True)
    rp = RavenPlanner(MyConstants.Arm.Both)
    rospy.sleep(4)
    rp.updateOpenraveJoints(MyConstants.Arm.Left)
    
    poseMatInRef = np.eye(4)
    refLinkName = MyConstants.Frames.LeftTool
    targLinkName = MyConstants.Frames.Link0
            
    # ref -> world
    refFromWorld = rp.robot.GetLink(refLinkName).GetTransform()

    # target -> world
    targFromWorld = rp.robot.GetLink(targLinkName).GetTransform()

    # target -> ref
    targFromRef = np.dot(np.linalg.inv(targFromWorld), refFromWorld)

    poseMatInTarg = np.dot(targFromRef, poseMatInRef)
    pose = tfx.pose(poseMatInTarg, frame=targLinkName)
    
    rospy.loginfo('publishing')
    pub = rospy.Publisher('left_tool_rave',PoseStamped)
    while not rospy.is_shutdown():
        pub.publish(pose.msg.PoseStamped())
        rospy.sleep(.5)
        

if __name__ == '__main__':
    test()
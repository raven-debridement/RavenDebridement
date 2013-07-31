#!/usr/bin/env python

"""
Contains general useful functions
for conversions, etc.
"""

# Import required Python code.
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker
import tf
import tf.transformations as tft
import numpy as np
import math

import tfx

import code

def createMarker(pose, id_):
    marker = Marker()
    marker.id = id_
    marker.header.frame_id = pose.header.frame_id
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.002
    marker.scale.y = 0.002
    marker.scale.z = 0.002
    marker.color.a = 1.0
    marker.color.r = 255
    marker.color.g = (id_) * 127
    marker.pose = pose.pose
    return marker

def withinBounds(ps0, ps1, transBound, rotBound, transFrame=None, rotFrame=None):
    """
    Returns if ps0 and ps1 (PoseStamped) are within translation and rotation bounds of each other

    Note: rotBound is in degrees
    """

    dPose = tfx.pose(deltaPose(ps0, ps1, transFrame, rotFrame))
    
    deltaPositions = dPose.position.list
    for deltaPos in deltaPositions:
        if abs(deltaPos) > transBound:
            return False

    between = angleBetweenQuaternions(tfx.tb_angles([0,0,0,1]).msg, dPose.orientation)
    if between > rotBound:
        return False
    
    return True


def deltaPose(currPose, desPose, posFrame=None, rotFrame=None):
    """
    Returns pose0 - pose1
    """

    currPose, desPose = tfx.pose(currPose), tfx.pose(desPose)
    currPos, desPos = currPose.position, desPose.position
    currRot, desRot = currPose.orientation, desPose.orientation

    if posFrame != None:
        tf_currPos_to_posFrame = tfx.lookupTransform(posFrame, currPos.frame, wait=10)
        currPos = tf_currPos_to_posFrame * currPos

        tf_desPos_to_posFrame = tfx.lookupTransform(posFrame, desPos.frame, wait=10)
        desPos = tf_desPos_to_posFrame * desPos

    if rotFrame != None:
        tf_currRot_to_rotFrame = tfx.lookupTransform(rotFrame, currRot.frame, wait=10)
        currRot = tf_currRot_to_rotFrame * currRot

        tf_desRot_to_rotFrame = tfx.lookupTransform(rotFrame, desRot.frame, wait=10)
        desRot = tf_desRot_to_rotFrame * desRot

    deltaPosition = desPos - currPos
    
    currQuat, desQuat = currRot.orientation.quaternion, desRot.orientation.quaternion
    deltaQuat = tft.quaternion_multiply(tft.quaternion_inverse(currQuat), desQuat)

    deltaPose = tfx.pose(deltaPosition, deltaQuat)

    return deltaPose.msg.Pose()
    
    
def angleBetweenQuaternions(quat0, quat1):
    """
    Returns angle between quat0 and quat1 in degrees
    """
    q0 = np.array([quat0.x, quat0.y, quat0.z, quat0.w])
    q1 = np.array([quat1.x, quat1.y, quat1.z, quat1.w])

    try:
        theta = math.acos(2*np.dot(q0,q1)**2 - 1)
    except ValueError:
        return 0

    theta = theta*(180.0/math.pi)

    return theta

def convertToFrame(p, frame):
    """
    Takes in a tfx point/pose stamped and returns it in frame
    """
    if p.frame != None and p.frame != frame:
        tf_pframe_to_frame = tfx.lookupTransform(frame, p.frame, wait=10)
        p = tf_pframe_to_frame * p

    return p



class Timeout():
    def __init__(self, timeoutTime):
        """
        timeoutTime is integer of how long until times out
        """
        self.timeoutTime = timeoutTime

    def start(self):
        """
        Restarts timeout every time this method is called
        """
        self.endTime = rospy.Time.now() + rospy.Duration(self.timeoutTime)

    def hasTimedOut(self):
        """
        returns true if time since start method called is
        greater than the current time
        """
        return rospy.Time.now() > self.endTime 


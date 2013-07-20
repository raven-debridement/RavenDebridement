#!/usr/bin/env python

"""
Contains general useful functions
for conversions, etc.
"""

# Import required Python code.
import roslib
roslib.load_manifest('master-control')
import rospy
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion, Point, TransformStamped
from visualization_msgs.msg import Marker
import tf
import tf.transformations as tft
import operator
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

def positionSubtract(p1, offset):
    pos = Point()
    pos.x = p1.x - offset.x
    pos.y = p1.y - offset.y
    pos.z = p1.z - offset.z
    return pos

def makeTransform(parent, child, trans, rot, time):
    transform = TransformStamped()
    transform.header.stamp = time
    transform.header.frame_id = parent
    transform.transform.translation.x = trans[0]
    transform.transform.translation.y = trans[1]
    transform.transform.translation.z = trans[2]
    transform.transform.rotation.x = rot[0]
    transform.transform.rotation.y = rot[1]
    transform.transform.rotation.z = rot[2]
    transform.transform.rotation.w = rot[3]
    transform.child_frame_id = child
    return transform

def xyzAndFrameToPointStamped(x, y, z, frame_id):
    """
    Converts xyz and frame_id to PointStamped
    """
    pStamped = PointStamped()
    
    pStamped.header.frame_id = frame_id
    pStamped.point.x = x
    pStamped.point.y = y
    pStamped.point.z = z

    return pStamped

def posAndOrientAndFrameToPoseStamped(x_trans, y_trans, z_trans, w, x_rot, y_rot, z_rot, frame_id):
    """
    Converts xyz, orientation, and frame_id to PoseStamped
    """
    pStamped = PoseStamped()
    
    pStamped.header.frame_id = frame_id
    pStamped.pose.position.x = x_trans
    pStamped.pose.position.y = y_trans
    pStamped.pose.position.z = z_tranas

    pStamped.pose.orienation.w = w
    pStamped.pose.orienation.x = x_rot
    pStamped.pose.orienation.y = y_rot
    pStamped.pose.orienation.z = z_rot

    return pStamped


def pointStampedToPoseStamped(pointStamped, orientation=None):
    """
    Combines pointStamped and orientation quaternion into PoseStamped

    Quaternion of new poseStamped defaults to no rotation
    """
    poseStamped = PoseStamped()
    poseStamped.header = pointStamped.header
    poseStamped.pose.position.x = pointStamped.point.x
    poseStamped.pose.position.y = pointStamped.point.y
    poseStamped.pose.position.z = pointStamped.point.z

    if orientation == None:
        poseStamped.pose.orientation.w = 1
    else:
        poseStamped.pose.orientation = orientation

    return poseStamped

def poseStampedToPointStamped(poseStamped):
    """
    Converts PoseStamped to PointStamped
    """
    pointStamped = PointStamped()
    pointStamped.header = poseStamped.header
    pointStamped.point.x = poseStamped.pose.position.x
    pointStamped.point.y = poseStamped.pose.position.y
    pointStamped.point.z = poseStamped.pose.position.z

    return pointStamped

def makeQuaternion(w, x, y, z):
    """
    wxyz to a geometry_msgs.msg Quaternion
    """
    newQuat = Quaternion()
    
    newQuat.w = w
    newQuat.x = x
    newQuat.y = y
    newQuat.z = z

    return newQuat
    
def reversePoseStamped(poseStamped):
    """
    Returns a new PoseStamped with the reverse orientation of poseStamped
    """
    poseStamped.pose.orientation = reverseQuaternion(poseStamped.pose.orientation)
    return poseStamped

def reverseQuaternion(quat):
    """
    Returns a new geometry_msgs.msg Quaternion of reversed quat
    """
    newQuat = Quaternion()

    newQuat.w = quat.w
    newQuat.x = -quat.x
    newQuat.y = -quat.y
    newQuat.z = -quat.z

    return newQuat

def convertToSameFrameAndTime(ps0, ps1, listener):
    """
    Converts point/pose 0 and point/pose 1 to the same frame

    Returns (None, None) if tf fails
    """
    ps0frame, ps1frame = ps0.header.frame_id, ps1.header.frame_id

    # need to be on same time so transformation will work
    # sometimes exceptions are thrown, DEAL WITH THIS
    try:
        commonTime = listener.getLatestCommonTime(ps0frame, ps1frame)
    except tf.Exception:
        return (None,None)

    ps0.header.stamp = ps1.header.stamp = commonTime

    if type(ps0) == PointStamped:
        return (listener.transformPoint(ps1frame, ps0), ps1)
    elif type(ps0) == PoseStamped:
        return (listener.transformPose(ps1frame, ps0), ps1)
    return (None, None)
    #return (ps0, listener.transformPoint(ps0frame, ps1))


def euclideanDistance(ps0, ps1, listener=None, xPlane=True, yPlane=True, zPlane=True):
    """
    Returns euclidean distance between two PointStamped

    xPlane, yPlane, zPlane booleans determine which planes the euclidean distance
    is computed over
    """
    # must be in same reference frame
    try:
        ps0, ps1 = convertToSameFrameAndTime(ps0, ps1, listener)
    except tf.Exception:
        return float("inf")

    
    if ps0 == None or ps1 == None:
        return False

    x0, y0, z0 = ps0.point.x, ps0.point.y, ps0.point.z
    x1, y1, z1 = ps1.point.x, ps1.point.y, ps1.point.z

    #flags for which planes euclidean distance is computed over
    if not xPlane:
        x0 = x1 = 0
    if not yPlane:
        y0 = y1 = 0
    if not zPlane:
        z0 = z1 = 0

    return ((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)**.5

def withinBounds(ps0, ps1, transBound, rotBound, transFrame=None, rotFrame=None):
    """
    Returns if ps0 and ps1 (PoseStamped) are within translation and rotation bounds of each other

    Note: rotBound is in degrees
    """

    dPose = tfx.pose(deltaPose(ps0, ps1, transFrame, rotFrame))
    
    deltaPositions = dPose.position.list
    print('deltaPositions')
    print(deltaPositions)
    for deltaPos in deltaPositions:
        if abs(deltaPos) > transBound:
            return False

    between = angleBetweenQuaternions(tfx.tb_angles([0,0,0,1]).msg, dPose.orientation)
    print('angleBetween')
    print(between)
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
    #deltaQuat = tft.quaternion_multiply(desQuat, tft.quaternion_inverse(currQuat))
    deltaQuat = tft.quaternion_multiply(tft.quaternion_inverse(currQuat), desQuat)

    deltaPose = tfx.pose(deltaPosition, deltaQuat)

    return deltaPose.msg.Pose()
    

def rotationFromTo(quat0, quat1):
    """
    DEPRECATED

    Returns the quaternion that rotates
    quat0 to quat1
    
    Assumes quat0 and quat1 in same frame
    """
    rot0 = tfx.rotation_tb(quat0)
    rot1 = tfx.rotation_tb(quat1)

    fromToRot = tfx.canonical.CanonicalRotation(rot1.matrix*rot0.inverse().matrix)
    fromToQuat = fromToRot.quaternion


    return Quaternion(*fromToQuat)
    
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



class TimeoutClass():
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


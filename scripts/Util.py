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
import tf
import tf.transformations as tft
import operator

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

def withinBounds(ps0, ps1, transBound, rotBound, listener=None):
    """
    Returns if ps0 and ps1 (PoseStamped) are within translation and rotation bounds of each other

    Note: rotBound is for euler angles
    """
    # must be in same reference frame
    if listener != None:
        try:
            ps0, ps1 = convertToSameFrameAndTime(ps0, ps1, listener)
        except tf.Exception:
            return False

    if ps0 == None or ps1 == None:
        return False

    xtrans0, ytrans0, ztrans0 = ps0.pose.position.x, ps0.pose.position.y, ps0.pose.position.z
    xtrans1, ytrans1, ztrans1 = ps1.pose.position.x, ps1.pose.position.y, ps1.pose.position.z

    wrot0, xrot0, yrot0, zrot0 = ps0.pose.orientation.w, ps0.pose.orientation.x, ps0.pose.orientation.y, ps0.pose.orientation.z    
    wrot1, xrot1, yrot1, zrot1 = ps1.pose.orientation.w, ps1.pose.orientation.x, ps1.pose.orientation.y, ps1.pose.orientation.z

    # convert to euler angles
    ps0rot0, ps0rot1, ps0rot2 = tft.euler_from_quaternion([xrot0, yrot0, zrot0, wrot0])
    ps1rot0, ps1rot1, ps1rot2 = tft.euler_from_quaternion([xrot1, yrot1, zrot1, wrot1])

    within = True

    within &= abs(xtrans0 - xtrans1) < transBound
    within &= abs(ytrans0 - ytrans1) < transBound
    within &= abs(ztrans0 - ztrans1) < transBound
    
    within &= abs(ps0rot0 - ps1rot0) < rotBound
    within &= abs(ps0rot1 - ps1rot1) < rotBound
    within &= abs(ps0rot2 - ps1rot2) < rotBound

    return within


def combinePoses(ps0, ps1, op=operator.add, listener=None):
    """
    Returns a PoseStamped of op(ps0,ps1)
    """
    # must be in same reference frame
    if listener != None:
        try:
            ps0, ps1 = convertToSameFrameAndTime(ps0, ps1, listener)
        except tf.Exception:
            return PoseStamped()

    if ps0 == None or ps1 == None:
        return False

    xtrans0, ytrans0, ztrans0 = ps0.pose.position.x, ps0.pose.position.y, ps0.pose.position.z
    xtrans1, ytrans1, ztrans1 = ps1.pose.position.x, ps1.pose.position.y, ps1.pose.position.z

    wrot0, xrot0, yrot0, zrot0 = ps0.pose.orientation.w, ps0.pose.orientation.x, ps0.pose.orientation.y, ps0.pose.orientation.z    
    wrot1, xrot1, yrot1, zrot1 = ps1.pose.orientation.w, ps1.pose.orientation.x, ps1.pose.orientation.y, ps1.pose.orientation.z

    ps0rot0, ps0rot1, ps0rot2 = tft.euler_from_quaternion([xrot0, yrot0, zrot0, wrot0])
    ps1rot0, ps1rot1, ps1rot2 = tft.euler_from_quaternion([xrot1, yrot1, zrot1, wrot1])

    addedPoint = Point(op(xtrans0,xtrans1), op(ytrans0,ytrans1), op(ztrans0,ztrans1))
    addedEuler = [op(ps0rot0,ps1rot0), op(ps0rot1,ps1rot1), op(ps0rot2,ps1rot2)]
    addedQuaternion = tft.quaternion_from_euler(addedEuler[0], addedEuler[1], addedEuler[2])
    addedOrientation = Quaternion(addedQuaternion[0], addedQuaternion[1], addedQuaternion[2], addedQuaternion[3])

    addedPose = PoseStamped()
    addedPose.header = ps0.header
    addedPose.pose.position = addedPoint
    addedPose.pose.orientation = addedOrientation

    return addedPose

def addPoses(ps0, ps1, listener=None):
    """
    Returns a PoseStamped of ps0+ps1
    """
    return combinePoses(ps0,ps1,operator.add, listener)

def subPoses(ps0, ps1, listener=None):
    """
    Returns a PoseStamped of ps0-ps1
    """
    return combinePoses(ps0,ps1,operator.sub,listener)


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

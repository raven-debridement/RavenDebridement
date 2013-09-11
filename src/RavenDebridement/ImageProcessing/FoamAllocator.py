import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import tfx
import collections

import operator as op
import functools

import threading

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, PoseStamped

from RavenDebridement.msg import FoamPoints, FoamAllocation

from RavenDebridement.Utils import Constants as MyConstants

import IPython

class FoamAllocator(object):
    def __init__(self):
        self.ignore = False
        
        self.currentCenters = []
        
        self.centerHistoryLength = tfx.duration(1)
        self.centerHistory = {}
        
        self.centerCombiningThreshold = 0.007
        
        self.allocations = {}
        self.allocationRadius = 0.02
        
        self.waitForFoamDuration = tfx.duration(5)
        
        self.orientation = tfx.rotation_tb(yaw=-90,pitch=90,roll=0)
        
        self.lock = threading.RLock()
        
        self.estPoseExclusionRadius = 0.03
        self.estPose = {arm : None for arm in 'LR'}
        self.estPoseSubs = { arm : rospy.Subscriber('/estimated_gripper_pose_%s' % arm, PoseStamped,functools.partial(self._estPoseCallback,arm)) for arm in 'LR'}
        
        self.allocation_pub = rospy.Publisher('/foam_allocation',FoamAllocation)
        
        self.sub = rospy.Subscriber('/foam_points', FoamPoints, self._foam_points_cb)
        
        self._printThread = threading.Thread(target=self._printer)
        self._printThread.daemon = True
        #self._printThread.start()
        
        self.marker_pub = rospy.Publisher('/foam_allocator_markers', MarkerArray)
        
        self._publishThread = threading.Thread(target=self._publisher)
        self._publishThread.daemon = True
        self._publishThread.start()
    
    def _estPoseCallback(self, arm, msg):
        if self.ignore: return
        self.estPose[arm] = tfx.pose(msg)
    
    def _printState(self):
        with self.lock:
            s = []
            s.append('---foam segmenter---')
            allCenters = self._getAllCenters()
            if not allCenters:
                s.append('no centers!')
            else:
                s.append('centers:')
                for ctr in allCenters:
                    s.append('%s' % ctr.tostring())
                
                if not self.allocations:
                    s.append('no allocations!')
                else:
                    s.append('allocations:')
                    for armName, allocCtr in self.allocations.iteritems():
                        s.append('%s: %s' % (armName, str(allocCtr)))
            print '\n'.join(s)
    
    def _printer(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._printState()
            rate.sleep()
    
    def _publish_state(self):
        with self.lock:
            ma = MarkerArray()
            
            all_centers_marker = Marker()
            all_centers_marker.ns = 'all_centers'
            all_centers_marker.id = 0
            all_centers_marker.type = Marker.POINTS
            all_centers_marker.action = Marker.ADD
            all_centers_marker.pose = tfx.identity_tf().msg.Pose()
            all_centers_marker.scale.x = 0.002
            all_centers_marker.scale.y = 0.002
            all_centers_marker.lifetime = rospy.Duration(1)
            
            all_centers_marker.color.r = 1
            all_centers_marker.color.g = 1
            all_centers_marker.color.b = 0
            all_centers_marker.color.a = 0.5
            for center in self._getAllCenters():
                all_centers_marker.header = center.msg.Header()
                all_centers_marker.points.append(center.msg.Point())
                
            ma.markers.append(all_centers_marker)
            
            curr_centers_marker = Marker()
            curr_centers_marker.ns = 'current_centers'
            curr_centers_marker.id = 0
            curr_centers_marker.type = Marker.POINTS
            curr_centers_marker.action = Marker.ADD
            curr_centers_marker.pose = tfx.identity_tf().msg.Pose()
            curr_centers_marker.scale.x = 0.001
            curr_centers_marker.scale.y = 0.001
            curr_centers_marker.lifetime = rospy.Duration(1)
            
            curr_centers_marker.color.r = 0
            curr_centers_marker.color.g = 0
            curr_centers_marker.color.b = 1
            curr_centers_marker.color.a = 1
            for center in self.currentCenters:
                curr_centers_marker.header = center.msg.Header()
                curr_centers_marker.points.append(center.msg.Point())
            
            ma.markers.append(curr_centers_marker)
                
            for id, arm in enumerate('LR'):
                if self.allocations.get(arm) is None:
                    continue
                alloc = self.allocations[arm]
                alloc_marker = Marker()
                alloc_marker.ns = 'alloc'
                alloc_marker.id = id
                alloc_marker.type = Marker.CYLINDER
                alloc_marker.action = Marker.ADD
                alloc_marker.header = alloc.msg.Header()
                alloc_marker.pose = tfx.pose(alloc).msg.Pose()
                alloc_marker.scale.x = self.allocationRadius
                alloc_marker.scale.y = self.allocationRadius
                alloc_marker.scale.z = 0.01
                alloc_marker.lifetime = rospy.Duration(1)
                
                alloc_marker.color.r = 0
                alloc_marker.color.g = 1
                alloc_marker.color.b = 1
                alloc_marker.color.a = 0.25
                
                ma.markers.append(alloc_marker)
                
            
            self.marker_pub.publish(ma)
                
    
    def _publisher(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish_state()
            rate.sleep()
    
    def _filterCenterHistory(self):
        now = tfx.stamp.now()
        for t, v in self.centerHistory.items():
            if (now-t) > self.centerHistoryLength:
                #rospy.loginfo('deleting from history: {0}'.format(self.centerHistory[t]))
                del self.centerHistory[t]
    
    def _foam_points_cb(self,msg):
        if self.ignore: return
        if rospy.is_shutdown():
            return
        with self.lock:
            self._filterCenterHistory()
            
            t = tfx.stamp(msg.header.stamp)
            all_pts = tuple([tfx.convertToFrame(tfx.point(pt,frame=msg.header.frame_id),MyConstants.Frames.Link0) for pt in msg.points])#,header=msg.header
            pts = []
            for pt in all_pts:
                for _, estPose in self.estPose.iteritems():
                    if estPose is not None:
                        tfxPoint = tfx.convertToFrame(tfx.point(pt,frame=msg.header.frame_id),estPose.frame)
                        if estPose.position.distance(tfxPoint) < self.estPoseExclusionRadius:
                            break
                else:
                    pts.append(pt)
            self.centerHistory[t] = pts
            self.currentCenters = pts
    
    def _getAllCenters(self):
        with self.lock:
            self._filterCenterHistory()
            allCenters = []
            for centers in self.centerHistory.itervalues():
                for center in centers:
                    for c in allCenters:
                        center, c = tfx.point(center), tfx.point(c)
                        if (center-c).norm < self.centerCombiningThreshold:
                            break
                    else:
                        allCenters.append(center)
            return [tfx.point(center) for center in allCenters]
    
    def _getUnallocatedCenters(self, armName, centers, new=False):
        unallocCenters = []
        for center in centers:
            ok = True
            for allocArm, allocationCenter in self.allocations.iteritems():
                if allocationCenter is not None and allocationCenter.distance(center) < self.allocationRadius:
                    ok = ok and (allocArm == armName and new)
                #if allocationCenter is not None and allocationCenter.distance(center) <  self.allocationRadius and \
                #        not (not new and allocArm == armName):
                #    ok = False
            if ok:
                unallocCenters.append(center)
        return unallocCenters
    
    def hasFoam(self, armName, new=False):
        with self.lock:
            if new:
                rospy.loginfo('Releasing allocation for %s'%armName)
                self.releaseAllocation(armName)
                rospy.loginfo('Allocation released for %s'%armName)
            centers = self._getUnallocatedCenters(armName, self._getAllCenters(), new=new)
            foam_found = bool(centers)
            if not foam_found:
                rospy.loginfo('Foam not found')
                msg = FoamAllocation()
                msg.header.stamp = rospy.Time.now()
                msg.arm = 'HAS_FOAM_FALSE'
                self.allocation_pub.publish(msg)
            return foam_found
    
    def allocateFoam(self, armName, new=False):
        with self.lock:
            
            startTime = tfx.time.now()
            while not self.currentCenters:
                rospy.sleep(0.1)
            centers = self._getUnallocatedCenters(armName, self.currentCenters, new=new)
                    
            if not centers:
                msg = FoamAllocation()
                msg.header.stamp = rospy.Time.now()
                msg.arm = 'ALLOCATE_FOAM_NONE'
                self.allocation_pub.publish(msg)
                return None
            
            if armName == MyConstants.Arm.Left:
                cmp = op.gt
            else:
                cmp = op.lt
            
            best = None
            for center in centers:
                if best is None or cmp(center.x,best.x):
                    best = center
            
            self.allocations[armName] = best
            
            
            best = tfx.convertToFrame(best,MyConstants.Frames.Link0)
            print 'returning new allocation', armName, best
            self._printState()
            
            foamPose = tfx.pose(best,self.orientation)
            
            msg = FoamAllocation()
            msg.header.stamp = rospy.Time.now()
            msg.arm = armName
            msg.pose = foamPose.msg.Pose()
            msg.new = new
            self.allocation_pub.publish(msg)
            return foamPose
    
    def releaseAllocation(self, armName):
        with self.lock:
            self.allocations[armName] = None

class ArmFoamAllocator(object):
    def __init__(self, armName, allocator=None):
        self.armName = armName
        self.allocator = allocator or FoamAllocator()
    
    def hasFoam(self, new=False):
        return self.allocator.hasFoam(self.armName, new=new)
    
    def allocateFoam(self, new=False):
        return self.allocator.allocateFoam(self.armName, new=new)
    
    def releaseAllocation(self):
        return self.allocator.releaseAllocation(self.armName)
    
def test():
    import IPython
    rospy.init_node('testFoamAllocator',anonymous=True)
    a = FoamAllocator()
    ar = ArmFoamAllocator('R', allocator=a)
    al = ArmFoamAllocator('L', allocator=a)
    
    IPython.embed()
    rospy.spin()
    
    
if __name__ == '__main__':
    test()
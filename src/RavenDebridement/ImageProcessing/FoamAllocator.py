import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import tfx
import collections

import operator as op

import threading

from RavenDebridement.msg import FoamPoints

from RavenDebridement.Utils import Constants as MyConstants

import IPython

class FoamAllocator(object):
    def __init__(self):
        
        self.currentCenters = []
        
        self.centerHistoryLength = tfx.duration(10)
        self.centerHistory = {}
        
        self.centerCombiningThreshold = 0.002
        
        self.allocations = {}
        self.allocationRadius = 0.02
        
        self.waitForFoamDuration = tfx.duration(5)
        
        self.orientation = tfx.rotation_tb(yaw=-90,pitch=90,roll=0)
        
        self.lock = threading.RLock()
        
        self.sub = rospy.Subscriber('/foam_points', FoamPoints, self._foam_points_cb)
        
        self._printThread = threading.Thread(target=self._printer)
        self._printThread.daemon = True
        #self._printThread.start()
    
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
                        s.append('%s: %s' % (armName, allocCtr.tostring()))
            print '\n'.join(s)
    
    def _printer(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._printState()
            rate.sleep()
    
    def _foam_points_cb(self,msg):
        if rospy.is_shutdown():
            return
        with self.lock:
            now = tfx.stamp.now()
            for t, v in self.centerHistory.items():
                if (now-t) > self.centerHistoryLength:
                    del self.centerHistory[t]
            
            t = tfx.stamp(msg.header.stamp)
            pts = tuple([tfx.convertToFrame(tfx.point(pt,frame=msg.header.frame_id),MyConstants.Frames.Link0) for pt in msg.points])#,header=msg.header
            self.centerHistory[t] = pts
            self.currentCenters = pts
    
    def _getAllCenters(self):
        with self.lock:
            allCenters = []
            for centers in self.centerHistory.itervalues():
                for center in centers:
                    for c in allCenters:
                        if (center-c).norm < self.centerCombiningThreshold:
                            break
                    else:
                        allCenters.append(center)
            return allCenters
    
    def hasFoam(self, armName, new=False):
        with self.lock:
            if new:
                self.releaseAllocation(armName)
            centers = []
            for center in self._getAllCenters():
                for _, allocationCenter in self.allocations.iteritems():
                    if allocationCenter is not None and allocationCenter.distance(center) >  self.allocationRadius:
                        return True
            return False
    
    def allocateFoam(self, armName, new=False):
        with self.lock:
            
            startTime = tfx.time.now()
            while not self.currentCenters:
                rospy.sleep(0.1)
            centers = []
            #allocationCenter = self.allocations.get(armName)
            for center in self.currentCenters:
                ok = True
                for allocArm, allocationCenter in self.allocations.iteritems():
                    if allocationCenter is not None and allocationCenter.distance(center) <  self.allocationRadius and \
                            not (not new and allocArm == armName):
                        ok = False
                if ok:
                    centers.append(center)
                    
            if not centers:
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
            return tfx.pose(best,self.orientation)
    
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
    armName = MyConstants.Arm.Right
    afa = ArmFoamAllocator(armName)
    
    IPython.embed()
    
    
    
if __name__ == '__main__':
    test()
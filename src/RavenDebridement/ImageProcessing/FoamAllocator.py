import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import tfx
import collections

import operator as op

import threading

from RavenDebridement.msg import FoamPoints

from RavenDebridement.Utils import Constants as MyConstants

class FoamAllocator(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/foam_points', FoamPoints, self._foam_points_cb)
        
        self.centers = {}
        self.allocations = {}
        self.prevAllocations = collections.defaultdict(set)
        
        self.orientation = tfx.rotation_tb(yaw=-90,pitch=90,roll=0)
        
        self.lock = threading.RLock()
        
        self._printThread = threading.Thread(target=self._printer)
        self._printThread.daemon = True
        #self._printThread.start()
    
    def _printState(self):
        with self.lock:
            s = []
            s.append('---foam segmenter---')
            if not self.centers:
                s.append('no centers!')
            else:
                s.append('centers:')
                for id, ctr in self.centers.iteritems():
                    s.append('%d: %s' % (id, ctr.tostring()))
                
                if not self.allocations:
                    s.append('no allocations!')
                else:
                    s.append('allocations:')
                    for armName, id in self.allocations.iteritems():
                        s.append('%s: %d' % (armName, id))
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
            for id, pt in zip(msg.ids,msg.points):
                self.centers[ord(id)] = tfx.point(pt,header=msg.header)
    
    def hasFoam(self, armName, new=False):
        with self.lock:
            taken = set()
            if new:
                taken.update(self.allocations.values())
            else:
                taken.update(v for k,v in self.allocations.iteritems() if k != armName)
            [taken.update(v) for v in self.prevAllocations.itervalues()]
            
            left = set(self.centers.keys()).difference(taken)
            self._printState()
            print 'taken', taken
            print 'left', left
            return bool(left)
    
    def allocateFoam(self, armName, new=False):
        with self.lock:
            if not new:
                if self.allocations.has_key(armName):
                    print 'returning existing allocation', armName, self.allocations[armName]
                    self._printState()
                    return tfx.pose(self.centers[self.allocations[armName]],self.orientation)
            
            if self.allocations.has_key(armName):
                self.prevAllocations[armName].add(self.allocations.pop(armName))
            
            taken = set()
            taken.update(self.allocations.values())
            [taken.update(v) for v in self.prevAllocations.itervalues()]
            
            left = set(self.centers.keys()).difference(taken)
            
            if armName == MyConstants.Arm.Left:
                cmp = op.gt
            else:
                cmp = op.lt
            
            best = None
            best_id = None
            for id in left:
                center = self.centers[id]
                if best is None or cmp(center.x,best.x):
                    best = center
                    best_id = id
            
            if best is None:
                return None
            
            self.allocations[armName] = id
            print 'returning new allocation', armName, best_id
            self._printState()
            tf = tfx.lookupTransform(MyConstants.Frames.Link0, best.frame)
            print tfx.pose(tf * best,self.orientation)
            return tfx.pose(tf * best,self.orientation)

class ArmFoamAllocator(object):
    def __init__(self, armName, allocator=None):
        self.armName = armName
        self.allocator = allocator or FoamAllocator()
    
    def hasFoam(self, new=False):
        return self.allocator.hasFoam(self.armName, new=new)
    
    def allocateFoam(self, new=False):
        return self.allocator.allocateFoam(self.armName, new=new)
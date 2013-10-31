#!/usr/bin/env python

import roslib
import rospy
roslib.load_manifest('RavenDebridement')
import random
import tfx
import argparse

from raven_2_trajectory.raven_arm import RavenArm

import IPython

class GridMove():
    def __init__(self, armName, rand=False, x=.03, y=.05, z=.004, testAngles = False):
        self.ravenArm = RavenArm(armName)
        
        if armName == 'R':
            self.homePose = tfx.pose([-.12,-.02,-.135],tfx.tb_angles(-90,90,0))
        else:
            self.homePose = tfx.pose([-.03,-.02,-.135],tfx.tb_angles(-90,90,0))
        self.random = rand
        
        self.x_levels = 3
        self.y_levels = 3
        self.z_levels = 5
        
        self.x = x
        self.y = y
        self.z = z
        
        # generate all angle combinations in order to sample the rotations as well
        self.angles = []
        
        self.angles.append(tfx.tb_angles(-90,90,0))
        
        if testAngles:
            self.angles.append(tfx.tb_angles(-90,75,0))
            self.angles.append(tfx.tb_angles(-90,105,0))
            
            self.angles.append(tfx.tb_angles(-75,90,0))
            self.angles.append(tfx.tb_angles(-75,75,0))
            self.angles.append(tfx.tb_angles(-75,105,0))
            
            self.angles.append(tfx.tb_angles(-105,90,0))
            self.angles.append(tfx.tb_angles(-105,75,0))
            self.angles.append(tfx.tb_angles(-105,105,0))
        
        self.grid = []
        self.grid += [self.homePose + [float(i)*(x/self.x_levels),0,0] for i in xrange(self.x_levels)]
        self.grid += [self.homePose + [x, float(i)*(-y/self.y_levels),0] for i in xrange(self.x_levels)]
        self.grid += [self.homePose + [-float(i)*(x/self.x_levels),-y,0] for i in xrange(self.x_levels)]
        self.grid += [self.homePose + [0, -float(i)*(-y/self.y_levels),0] for i in xrange(self.x_levels)]
        
        """
        self.grid = []
        self.grid += [tfx.pose([x/self.x_levels,0,0]) for _ in xrange(self.x_levels)]
        self.grid += [tfx.pose([0,-y/self.y_levels,0]) for _ in xrange(self.y_levels)]
        self.grid += [tfx.pose([-x/self.x_levels,0,0]) for _ in xrange(self.x_levels)]
        self.grid += [tfx.pose([0,y/self.y_levels,0]) for _ in xrange(self.y_levels)]
        """
        
        """
        # start in front right
        self.grid = [tfx.pose([x,0,-z]),
                     tfx.pose([0,-y,z]),
                     tfx.pose([-x,0,-z]),
                     tfx.pose([0,y,z])]
        """
        
    def getGrid(self, homePose):
        grid = []
        
        for i in xrange(self.x_levels):
            for j in xrange(len(self.angles)):
                pose = homePose + [float(i)*(self.x/self.x_levels),0,0]
                pose.tb_angles = self.angles[j]
                grid += [pose]
        
        for i in xrange(self.x_levels):
            for j in xrange(len(self.angles)):
                pose = homePose + [self.x, float(i)*(-self.y/self.y_levels),0]
                pose.tb_angles = self.angles[j]
                grid += [pose]
                
        for i in xrange(self.x_levels):
            for j in xrange(len(self.angles)):
                pose = homePose + [self.x-float(i)*(self.x/self.x_levels),-self.y,0]
                pose.tb_angles = self.angles[j]
                grid += [pose]
                
        for i in xrange(self.x_levels):
            for j in xrange(len(self.angles)):
                pose = homePose + [0, -self.y+float(i)*(self.y/self.y_levels),0]
                pose.tb_angles = self.angles[j]
                grid += [pose]
        
        return grid
        
    def moveByGrid(self, homePose):
        grid = []
        for _ in xrange(self.z_levels):
            homePose.position.z -= self.z
            grid += self.getGrid(homePose)
        
        print 'moveByGrid'
        print "GRID", len(grid)
        
        if not self.random:
            print 'seeded'
            random.seed(2000)
        random.shuffle(grid)
        homePose = grid[0]
        self.ravenArm.goToGripperPose(homePose)
        for pose in grid:
            print pose
            rospy.sleep(3)
            if rospy.is_shutdown():
                return
            self.ravenArm.goToGripperPose(pose)
        
    def run(self):
        self.ravenArm.start()
        
        rospy.loginfo('Press enter to start')
        raw_input()
        
        homePose = self.homePose
        self.moveByGrid(homePose)
                
        if rospy.is_shutdown():
            return
                
        self.ravenArm.stop()
        
if __name__ == '__main__':
    rospy.init_node('GridMove', anonymous=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('arm',nargs='?',default='L')
    parser.add_argument('rand',nargs='?',default=False)
    args = parser.parse_args(rospy.myargv()[1:])
    arm_side = args.arm
    del args.arm
    rand = args.rand
    del args.rand
    
    rospy.sleep(2)
    gm = GridMove(arm_side, rand)
    gm.run()        
        
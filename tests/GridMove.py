import roslib
import rospy
roslib.load_manifest('RavenDebridement')
import tfx

from raven_2_trajectory.raven_arm import RavenArm

import IPython

class GridMove():
    def __init__(self, armName, x=.03, y=.05, z=.004):
        self.ravenArm = RavenArm(armName)
        
        if armName == 'R':
            self.homePose = tfx.pose([-.12,-.02,-.135],tfx.tb_angles(-90,90,0))
        else:
            self.homePose = tfx.pose([-.03,-.02,-.135],tfx.tb_angles(-90,90,0))
        
        self.x_levels = 5
        self.y_levels = 5
        self.z_levels = 5
        
        self.x = x
        self.y = y
        self.z = z
        
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
        grid += [homePose + [float(i)*(self.x/self.x_levels),0,0] for i in xrange(self.x_levels)]
        grid += [homePose + [self.x, float(i)*(-self.y/self.y_levels),0] for i in xrange(self.x_levels)]
        grid += [homePose + [self.x-float(i)*(self.x/self.x_levels),-self.y,0] for i in xrange(self.x_levels)]
        grid += [homePose + [0, -self.y+float(i)*(self.y/self.y_levels),0] for i in xrange(self.x_levels)]
        
        return grid
        
    def moveByGrid(self, homePose):
        grid = self.getGrid(homePose)
        
        print 'moveByGrid'
        
        self.ravenArm.goToGripperPose(homePose)
        for pose in grid:
            print pose
            rospy.sleep(3)
            self.ravenArm.goToGripperPose(pose)
        
    def run(self):
        self.ravenArm.start()
        
        rospy.loginfo('Press enter to start')
        raw_input()
        
        homePose = self.homePose
        
        while not rospy.is_shutdown():
            for _ in xrange(self.z_levels):
                self.moveByGrid(homePose)
                homePose.position.z -= self.z
                
                if rospy.is_shutdown():
                    return
            
                
        self.ravenArm.stop()
        
if __name__ == '__main__':
    rospy.init_node('GridMove', anonymous=True)
    rospy.sleep(2)
    gm = GridMove('R')
    gm.run()        
        
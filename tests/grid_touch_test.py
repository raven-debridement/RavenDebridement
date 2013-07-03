#!/usr/bin/env python

import roslib
#roslib.load_manifest('raven_2_trajectory')
roslib.load_manifest('master-control')
import rospy
from geometry_msgs.msg import *
from math import *
from raven_2_msgs.msg import *
from std_msgs.msg import Header
import copy
import sys,argparse, os.path
from numpy.linalg import norm

import tf
import tfx

from raven_2_trajectory.trajectory_player import TrajectoryPlayer, Stage

class InitPoseHolder:
	init_pose = None
	
	@staticmethod
	def callback(msg):
		if InitPoseHolder.init_pose is None:
			InitPoseHolder.init_pose = tfx.pose(msg)

def main():
	rospy.init_node(os.path.basename(sys.argv[0]),anonymous=True)
	
	parser = argparse.ArgumentParser(add_help=False)
	
	parser.add_argument('grid_size',type=int,nargs=2)
	
	#parser.add_argument('-s','--spacing',type=float,default=(0.00635*2))
	parser.add_argument('-l','--lift',type=float,default=0.01)
	
	args = parser.parse_args(rospy.myargv()[1:])
	
#	sub = rospy.Subscriber('tool_pose/R',PoseStamped,InitPoseHolder.callback)
#	rate = rospy.Rate(10)
#	while InitPoseHolder.init_pose is None and not rospy.is_shutdown():
#		rate.sleep()
#	init_pose = InitPoseHolder.init_pose

	orientation = tfx.tb_angles(0,90,0)
	
	grid_height = -0.155
	
	grid_11 = tfx.point(-0.060, -0.039, grid_height)
	
	#grid_11 = tfx.point(-0.055, -0.032, grid_height)
	#grid_1N = tfx.point(-0.105, -0.034, grid_height)
	grid_1N = grid_11 - [0.00635*2 * (args.grid_size[0]-1),0,0]
	#grid_M1 = tfx.point(-0.057, -0.085, grid_height)
	grid_M1 = grid_11 - [0,0.00635*2 * (args.grid_size[1]-1),0]
	grid_MN = None
	
	row_vector = grid_1N-grid_11
	col_vector = grid_M1-grid_11
	
	row_spacing = norm(row_vector) / (args.grid_size[0]-1)
	col_spacing = norm(row_vector) / (args.grid_size[0]-1)
	
	print 'spacing: %f, %f' % (row_spacing * 100 / 2.54, col_spacing * 100 / 2.54)
	
	
	def get_pt(row,col):
		row_interp = (float(row-1)/(args.grid_size[0]-1))
		col_interp = (float(col-1)/(args.grid_size[1]-1))
		point = grid_11 + row_interp * row_vector + col_interp * col_vector 
		return point
	
	player = TrajectoryPlayer(arms='R')
	
	d = 1.75
	
	
	player.add_set_gripper(0)
	player.add_goto_first_pose(tfx.pose(grid_11 + [0,0,2*args.lift],orientation),speed=0.02, name='Move to 1,1')
	
	for row in xrange(1,args.grid_size[0]+1):
		if row != 1:
			player.add_point_to_point_with_lift('Move to %d,%d' % (row,1),get_pt(row-1,args.grid_size[1]),get_pt(row,1),orientation,args.lift,duration=d)
		for col in xrange(2,args.grid_size[1]+1):
			player.add_point_to_point_with_lift('Move to %d,%d' % (row,col),get_pt(row,col-1),get_pt(row,col),orientation,args.lift,duration=d)
	player.add_point_to_point_with_lift('Move back to 1,1',get_pt(args.grid_size[0],args.grid_size[1]),get_pt(1,1),orientation,args.lift,duration=3)
	player.add_point_to_point('Lift at 1,1',get_pt(1,1),get_pt(1,1)+[0,0,args.lift],orientation,duration=3)

	success = player.play(dry_run=False)
	
if __name__ == '__main__':
	main()

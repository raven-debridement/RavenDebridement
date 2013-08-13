#!/usr/bin/env python 	

from openravepy import *
from openravepy.misc import OpenRAVEGlobalArguments
from itertools import izip

import time
import trajoptpy.make_kinbodies as mk
import sensor_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import numpy as np, os.path as osp
import os
import cloudprocpy,trajoptpy,openravepy

def remove_floor(cloud):
    notfloor = get_xyz_world_frame(cloud)[:,2] > .1
    cloud = cloudprocpy.maskFilter(cloud, notfloor, True)
    return cloud

def get_xyz_world_frame(cloud):
    xyz1 = cloud.to2dArray()
    xyz1[:,3] = 1
    return xyz1.dot(T_w_k.T)[:,:3]

def generate_mesh(cloud):
    #cloud = cloudprocpy.fastBilateralFilter(cloud, 15, .05) # smooth out the depth image
    cloud = remove_floor(cloud) # remove points with low height (in world frame)
    big_mesh = cloudprocpy.meshOFM(cloud, 3, .1) # use pcl OrganizedFastMesh to make mesh
    simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .02) # decimate mesh with VTK function
    return simple_mesh

env = openravepy.Environment()
env.StopSimulation()


filenames = []
for root, dirs, files in os.walk('/home/annal/src/RavenDebridement/pc_data/'):
    for f in files:
	if f.endswith('.pcd'):
	    filenames.append(f)
    break
filenames.sort()
cloud_orig = cloudprocpy.readPCDXYZ(osp.join('/home/annal/src/RavenDebridement/pc_data/', filenames[len(files)-1]))
mesh = generate_mesh(cloud_orig)
mesh_body = mk.create_trimesh(env, get_xyz_world_frame(mesh.getCloud()), np.array(mesh.getFaces()), name="simple_mesh")

viewer = trajoptpy.GetViewer(env)
cloud_orig_colored = cloudprocpy.readPCDXYZRGB(osp.join('/home/annal/src/RavenDebridement/pc_data/', filenames[len(files)-1]))
rgbfloats = cloud_orig_colored.to2dArray()[:,4]
rgb0 = np.ndarray(buffer=rgbfloats.copy(),shape=(480*640,4),dtype='uint8')
xyz=get_xyz_world_frame(cloud_orig)
goodinds = np.isfinite(xyz[:,0])
cloud_handle = env.plot3(xyz[goodinds,:], 2,(rgb0[goodinds,:3][:,::-1])/255. )
viewer.Idle()
del cloud_handle



	

#!/usr/bin/env python
import roslib
roslib.load_manifest('RavenDebridement')
import rospy
import math

from geometry_msgs.msg import *
from raven_2_msgs.msg import *

import trajoptpy
import json

from RavenDebridement.Utils import Util
from RavenDebridement.Utils import Constants
from RavenDebridement.RavenCommand.RavenArm import RavenArm
from RavenDebridement.RavenCommand.RavenPlanner import RavenPlanner, Request

import tfx

import IPython

def test():
    rospy.init_node('test_IK',anonymous=True)
    rp = RavenPlanner(Constants.Arm.Right)
    rospy.sleep(2)
    
    startJoints = {0:0.51091998815536499,
                   1:1.6072717905044558,
                   2:-0.049991328269243247,
                   4:0.14740140736103061,
                   5:0.10896652936935426,
                   8:-0.31163200736045837}

    endJoints = {0:0.45221099211619786,
                 1:2.2917657932075581,
                 2:-0.068851854076958902,
                 4:0.44096283117933965,
                 5:0.32085205361054148,
                 8:-0.82079765727524379}
    
    rp.updateOpenraveJoints(endJoints)
    endEE = rp.manip.GetEndEffectorTransform()
    
    rp.updateOpenraveJoints(startJoints)
    startEE = rp.manip.GetEndEffectorTransform()
    
    rp.env.SetViewer('qtcoin')

    
    endJointPositions = []
    for raveJointType in rp.manip.GetArmIndices():
        rosJointType = rp.raveJointTypesToRos[raveJointType]
        endJointPositions.append(endJoints[rosJointType])

    n_steps = 15
    
    endEEPose = tfx.pose(endEE)
    desPos = endEEPose.position.list
    endEEQuat = endEEPose.orientation
    wxyzQuat = [endEEQuat.w, endEEQuat.x, endEEQuat.y, endEEQuat.z]
    
    request = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : rp.manip.GetName(),
                "start_fixed" : True
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]}
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [100],
                        "continuous" : True,
                        "dist_pen" : [0.001]
                        }
                    },
 
                ],
            "constraints" : [
               {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : desPos,
                                "wxyz" : wxyzQuat,
                                "link" : rp.toolFrame,
                                "rot_coeffs" : [5,5,5],
                                "pos_coeffs" : [100,100,100]
                                }
                    }
                ],
            "init_info" : {
                "type" : "straight_line",
                "endpoint" : endJointPositions
                }
            }
    
    # convert dictionary into json-formatted string
    s = json.dumps(request)
    # create object that stores optimization problem
    prob = trajoptpy.ConstructProblem(s, rp.env)
    # do optimization
    result = trajoptpy.OptimizeProblem(prob)

    # check trajectory safety
    from trajoptpy.check_traj import traj_is_safe
    prob.SetRobotActiveDOFs()
    if not traj_is_safe(result.GetTraj(), rp.robot):
        rospy.loginfo('Trajopt trajectory is not safe. Trajopt failed!')
        rospy.loginfo('Still returning trajectory')
        #return

    j = rp.trajoptTrajToDicts(result.GetTraj())
    
    Util.plot_transform(rp.env, startEE)
    Util.plot_transform(rp.env, endEE)
    
    IPython.embed()

if __name__ == '__main__':
    test()
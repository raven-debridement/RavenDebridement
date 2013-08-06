#include "ros/ros.h"
#include "ros/console.h"
#include "RavenDebridement/InvKinSrv.h"

#include "raven_2_msgs/Constants.h"

#include <math.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <LinearMath/btTransform.h>

#include "raven/hmatrix.h"
#include "raven/defines.h"
#include "raven/utils.h"
#include "t_to_DAC_val.h"
#include "raven/state/runlevel.h"

#include "DS0.h"
#include "raven/kinematics/kinematics_defines.h"

// from raven_2_msgs
#define YAW 8

const double go_dh_al[6] = {0,              -A12,   M_PI - A23,  0, M_PI/2, -M_PI/2};
const double go_dh_a[6]  = {0,              0,      0,         0, 0, 0 };
const double gr_dh_al[6] = {M_PI,           A12,   A23,        M_PI, M_PI/2, M_PI/2};
const double gr_dh_a[6]  = {0,              0,      0,         0, 0, 0 };

float fix_angle(float angle,float center) {
	float test_angle = angle;
	int cnt = 1;
	while ((test_angle-center) > M_PI) {
		test_angle = angle - cnt * 2*M_PI;
		cnt++;
	}
	angle = test_angle;
	cnt = 1;
	while ((test_angle-center) < -M_PI) {
		test_angle = angle + cnt * 2*M_PI;
		cnt++;
	}
	return test_angle;
}

bool check_joint_limits1_new(float d_act, float thp_act, float g1_act, float g2_act,int validity[4]) {
	validity[0] = 0;
	validity[1] = 0;
	validity[2] = 0;
	validity[3] = 0;

	
	bool any_nan = false;
	if (d_act != d_act) { any_nan = true; };
	if (thp_act != thp_act) { any_nan = true; };
	if (g1_act != g1_act) {  any_nan = true; };
	if (g2_act != g2_act) { any_nan = true; };
	if (any_nan) {
		return false;
	}
	

	bool bad = false;
	if (d_act < Z_INS_MIN_LIMIT) {
		validity[0] = -1;
		bad = true;
	}
	if (d_act > Z_INS_MAX_LIMIT) {
		validity[0] = +1;
		bad = true;
	}
	if (thp_act < TOOL_WRIST_MIN_LIMIT) {
		validity[1] = -1;
		bad = true;
	}
	if (thp_act > TOOL_WRIST_MAX_LIMIT) {
		validity[1] = +1;
		bad = true;
	}
	if (g1_act < TOOL_GRASP1_MIN_LIMIT) {
		validity[2] = -1;
		bad = true;
	}
	if (g1_act > TOOL_GRASP1_MAX_LIMIT) {
		validity[2] = +1;
		bad = true;
	}
	if (g2_act < TOOL_GRASP2_MIN_LIMIT) {
		validity[3] = -1;
		bad = true;
	}
	if (g2_act > TOOL_GRASP2_MAX_LIMIT) {
		validity[3] = +1;
		bad = true;
	}
	return !bad;
}

bool check_joint_limits2_new(float ths_act, float the_act, float thr_act,float validity[3]) {
	validity[0] = 0;
	validity[1] = 0;
	validity[2] = 0;
	
	
	bool any_nan = false;
	if (ths_act != ths_act) { ROS_INFO("ths_act != ths_act, %f",ths_act); any_nan = true; };
	if (the_act != the_act) { ROS_INFO("the_act != the_act, %f",the_act); any_nan = true; };
	if (thr_act != thr_act) { ROS_INFO("thr_act != thr_act, %f",thr_act); any_nan = true; };
	if (any_nan) {
	    ROS_INFO("any_nan is true in check_joint_limits2_new");
		return false;
	}
	

	bool bad = false;
	if (ths_act < SHOULDER_MIN_LIMIT) {
		validity[0] = ths_act - SHOULDER_MIN_LIMIT;
		bad = true;
	}
	if (ths_act > SHOULDER_MAX_LIMIT) {
		validity[0] = ths_act - SHOULDER_MAX_LIMIT;
		bad = true;
	}
	if (the_act < ELBOW_MIN_LIMIT) {
		validity[1] = the_act - ELBOW_MIN_LIMIT;
		bad = true;
	}
	if (the_act > ELBOW_MAX_LIMIT) {
		validity[1] = the_act - ELBOW_MAX_LIMIT;
		bad = true;
	}
	if (thr_act < TOOL_ROLL_MIN_LIMIT) {
		validity[2] = thr_act - TOOL_ROLL_MIN_LIMIT;
		bad = true;
	}
	if (thr_act > TOOL_ROLL_MAX_LIMIT) {
		validity[2] = thr_act - TOOL_ROLL_MAX_LIMIT;
		bad = true;
	}
	return !bad;
}

int set_joints_with_limits1(mechanism* mech, float d_act, float thp_act, float g1_act, float g2_act) {
	mech->joint[Z_INS].jpos_d    = d_act;
	mech->joint[WRIST].jpos_d    = thp_act; //WRIST_HOME_ANGLE; int WARNING_WRIST_NOT_SET;
	mech->joint[GRASP1].jpos_d   = g1_act; //GRASP1_HOME_ANGLE; int WARNING_GRASP1_NOT_SET;
	mech->joint[GRASP2].jpos_d   = g2_act; //GRASP2_HOME_ANGLE; int WARNING_GRASP2_NOT_SET;

	int limits=0;

	if (mech->joint[Z_INS].jpos_d  < Z_INS_MIN_LIMIT) {
	  printf("hit insertion min limit");
		limits++;
		mech->joint[Z_INS].jpos_d = Z_INS_MIN_LIMIT;
	} else if (mech->joint[Z_INS].jpos_d  > Z_INS_MAX_LIMIT) {
	  printf("hit insertion max limit");
		limits++;
		mech->joint[Z_INS].jpos_d = Z_INS_MAX_LIMIT;
	}

	if (mech->joint[WRIST].jpos_d  < TOOL_WRIST_MIN_LIMIT) {
	  printf("hit tool wrist min limit");
		limits++;
		mech->joint[WRIST].jpos_d = TOOL_WRIST_MIN_LIMIT;
	} else if (mech->joint[WRIST].jpos_d  > TOOL_WRIST_MAX_LIMIT) {
	  printf("hit tool wrist max limit");
		limits++;
		mech->joint[WRIST].jpos_d = TOOL_WRIST_MAX_LIMIT;
	}

	/*
	//TODO: limit this elsewhere
	if (fabs(mech->joint[WRIST].jpos_d - mech->joint[WRIST].jpos) > 10 DEG2RAD) {
	  printf("hit tool wrist subtraction limit");
		if (mech->joint[WRIST].jpos_d > mech->joint[WRIST].jpos) {
		  mech->joint[WRIST].jpos_d = mech->joint[WRIST].jpos + 10 DEG2RAD;
		} else {
		  	mech->joint[WRIST].jpos_d = mech->joint[WRIST].jpos - 10 DEG2RAD;
		}
	}
	*/

	if (mech->joint[GRASP1].jpos_d  < TOOL_GRASP1_MIN_LIMIT) {
	  printf("hit grasp1 min limit");
		limits++;
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MIN_LIMIT;
	} else if (mech->joint[GRASP1].jpos_d  > TOOL_GRASP1_MAX_LIMIT) {
	  printf("hit grasp1 max limit");
		limits++;
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MAX_LIMIT;
	}

	if (mech->joint[GRASP2].jpos_d  < TOOL_GRASP2_MIN_LIMIT) {
	  printf("hit grasp2 min limit");
		limits++;
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MIN_LIMIT;
	} else if (mech->joint[GRASP2].jpos_d  > TOOL_GRASP2_MAX_LIMIT) {
	  printf("hit grasp2 max limit");
		limits++;
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MAX_LIMIT;
	}

	printf("Z_INS = %f\n", mech->joint[Z_INS].jpos_d RAD2DEG);
	printf("WRIST = %f\n", mech->joint[WRIST].jpos_d RAD2DEG);
	printf("GRASP1 = %f\n", mech->joint[GRASP1].jpos_d RAD2DEG);
	printf("GRASP2 = %f\n", mech->joint[GRASP2].jpos_d RAD2DEG);
	

	return limits;
}

int set_joints_with_limits2(mechanism* mech, float ths_act, float the_act, float thr_act) {
	mech->joint[SHOULDER].jpos_d = ths_act;
	mech->joint[ELBOW].jpos_d    = the_act;
	//mech->joint[TOOL_ROT].jpos_d = fix_angle(thr_act + M_PI); //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;
	mech->joint[TOOL_ROT].jpos_d = thr_act; //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;

	int limits = 0;
	if (mech->joint[SHOULDER].jpos_d < SHOULDER_MIN_LIMIT) {
	  printf("hit shoulder min limit");
		limits++;
		mech->joint[SHOULDER].jpos_d = SHOULDER_MIN_LIMIT;
	} else if (mech->joint[SHOULDER].jpos_d > SHOULDER_MAX_LIMIT) {
	  printf("hit shoulder max limit");
		limits++;
		mech->joint[SHOULDER].jpos_d = SHOULDER_MAX_LIMIT;
	}

	if (mech->joint[ELBOW].jpos_d < ELBOW_MIN_LIMIT) {
	  printf("hit elbow min limit");
		limits++;
		mech->joint[ELBOW].jpos_d = ELBOW_MIN_LIMIT;
	} else if (mech->joint[ELBOW].jpos_d > ELBOW_MAX_LIMIT) {
	  printf("hit elbow max limit");
		limits++;
		mech->joint[ELBOW].jpos_d = ELBOW_MAX_LIMIT;
	}

	if (mech->joint[TOOL_ROT].jpos_d < TOOL_ROLL_MIN_LIMIT) {
	  printf("hit tool roll min limit");
		limits++;
		mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MIN_LIMIT;
	} else if (mech->joint[TOOL_ROT].jpos_d > TOOL_ROLL_MAX_LIMIT) {
	  printf("hit tool roll max limit");
		limits++;
		mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MAX_LIMIT;
	}

	/*
	//TODO: limit this elsewhere
	if (fabs(mech->joint[TOOL_ROT].jpos_d - mech->joint[TOOL_ROT].jpos) > 10 DEG2RAD) {
	  printf("tool roll subraction limit");
		if (mech->joint[TOOL_ROT].jpos_d > mech->joint[TOOL_ROT].jpos) {
		  	mech->joint[TOOL_ROT].jpos_d = mech->joint[TOOL_ROT].jpos + 10 DEG2RAD;
		} else {
		  	mech->joint[TOOL_ROT].jpos_d = mech->joint[TOOL_ROT].jpos - 10 DEG2RAD;
		}
	}
	*/

	printf("SHOULDER = %f\n",mech->joint[SHOULDER].jpos_d RAD2DEG);
	printf("ELBOW = %f\n",mech->joint[ELBOW].jpos_d RAD2DEG);
	printf("TOOL_ROT = %f\n",mech->joint[TOOL_ROT].jpos_d RAD2DEG);

	return limits;
}



int invMechKinNew(struct mechanism *mech, float x, float y, float z, btMatrix3x3 matrix, float grasp) {
        struct position*    pos_d = &(mech->pos_d);
	struct orientation* ori_d = &(mech->ori_d);


	int armId = armIdFromMechType(mech->type);


	// desired tip position
	//btVector3 actualPoint = btVector3(pos_d->x,pos_d->y,pos_d->z) / MICRON_PER_M;
	btVector3 actualPoint = btVector3(x, y, z);
	//btMatrix3x3 actualOrientation = toBt(ori_d->R);
	btMatrix3x3 actualOrientation = matrix;
	btTransform actualPose(actualOrientation,actualPoint);

	btQuaternion quat = actualPose.getRotation();
	printf("quaternion %f %f %f %f\n",quat.x(),quat.y(),quat.z(),quat.w());

	printf("matrix\n");
	for(int i=0; i<3; i++){
	    printf("%f %f %f\n",matrix[i][0],matrix[i][1],matrix[i][2]);
	}

	printf("actual point %f %f %f\n", actualPoint[0], actualPoint[1], actualPoint[2]);
	tb_angles tb(actualOrientation);
	printf("actual orientation %f %f %f\n",tb.yaw_deg,tb.pitch_deg,tb.roll_deg);
	
	//float grasp = GRASP_TO_IK(armId,mech->ori_d.grasp);

	//ROS_INFO("ori_d grasp %f",mech->ori_d.grasp);
	ROS_INFO("grasp %f",grasp);


	/*
	 * Actual pose is in the actual world frame, so we have <actual_world to gripper>
	 * The ik world frame is the base frame.
	 * Therefore, we need <base to actual_world> to get <base to gripper>.
	 * <base to actual_world> is inverse of <actual_world to base>
	 *
	 * Since the ik is based on the yaw frame (to which the gripper is fixed), we
	 * take the pose of the yaw frame, not the gripper frame
	 */
	btTransform ik_pose = ik_world_to_actual_world(armId) * actualPose * Tg.inverse();

	btVector3 ik_origin = ik_pose.getOrigin();
	btMatrix3x3 ik_basis = ik_pose.getBasis();
	printf("ik point %f %f %f\n",ik_origin.x(),ik_origin.y(),ik_origin.z());
	tb_angles ik_tb(ik_basis);
	printf("ik orientation %f %f %f\n",ik_tb.yaw_deg,ik_tb.pitch_deg,ik_tb.roll_deg);
		
	
	const float th12 = THETA_12;
	const float th23 = THETA_23;

	const float ks12 = sin(th12);
	const float kc12 = cos(th12);
	const float ks23 = sin(th23);
	const float kc23 = cos(th23);

	const float dw = DW;

	btTransform Tgripper_to_world = ik_pose.inverse();

	btVector3 origin_in_gripper_frame = Tgripper_to_world.getOrigin();
	float px = origin_in_gripper_frame.x();
	float py = origin_in_gripper_frame.y();
	float pz = origin_in_gripper_frame.z();

	ROS_INFO("(px, py, pz) = (%f, %f, %f)",px,py,pz);

	float thy = atan2f(py,-px);

	float thp;
	if (fabs(thy) < 0.001) {
		thp = atan2f(-pz, -px/cos(thy) - dw);
	} else {
		thp = atan2f(-pz,  py/sin(thy) - dw);
	}

	float d = -pz / sin(thp);

	float d_act, thp_act, thy_act, g1_act, g2_act;
	d_act = D_FROM_IK(armId,d);
	thp_act = THP_FROM_IK(armId,thp);
	thy_act = THY_FROM_IK(armId,thy,grasp);
	g1_act = FINGER1_FROM_IK(armId,thy,grasp);
	g2_act = FINGER2_FROM_IK(armId,thy,grasp);

	//check angles
	int validity1[4];
	bool valid1 = check_joint_limits1_new(d_act,thp_act,g1_act,g2_act,validity1);
	if (!valid1) {
	    ROS_INFO("check_joint_limits1_new failed");
	    printf("ik %d invalid --1-- d [%d] % 2.4f \tp [%d] % 3.1f\ty [%d %d] % 3.1f\n",
					armId,
					validity1[0],              d_act,
					validity1[1],              thp_act RAD2DEG,
					validity1[2],validity1[3], thy_act RAD2DEG);
	    return 0;
	}


	ROS_INFO("d %f",d);
	ROS_INFO("thp %f",thp);
	ROS_INFO("thy %f",thy);

	btVector3 z_roll_in_world = btTransform(Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).invXform(btVector3(0,0,1));
	btVector3 x_roll_in_world = btTransform(Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).invXform(btVector3(1,0,0));


	float zx = z_roll_in_world.x();
	float zy = z_roll_in_world.y();
	float zz = z_roll_in_world.z();

	float xx = x_roll_in_world.x();
	float xy = x_roll_in_world.y();
	float xz = x_roll_in_world.z();


	float cthe = (zy + kc12*kc23) / (ks12*ks23);

	float the_1 = acos(cthe);
	float the_2 = -acos(cthe);

	ROS_INFO("cthe %f", cthe);
	ROS_INFO("the_1 %f", the_1);

	float the_opt[2];
	the_opt[0] = the_1;
	the_opt[1] = the_2;

	float ths_opt[2];
	float thr_opt[2];

	bool opts_valid[2];
	float validity2[2][4];

	float ths_act[2];
	float the_act[2];
	float thr_act[2];

	for (int i=0;i<2;i++) {
		float sthe_tmp = sin(the_opt[i]);
		float C1 = ks12*kc23 + kc12*ks23*cthe;
		float C2 = ks23 * sthe_tmp;
		float C3 = C2 + C1*C1 / C2;

		ths_opt[i] = atan2(
				-sgn(C3)*(zx - C1 * zz / C2),
				 sgn(C3)*(zz + C1 * zx / C2));

		float sths_tmp = sin(ths_opt[i]);
		float cths_tmp = cos(ths_opt[i]);

		float C4 = ks12 * sin(the_opt[i]);
		float C5 = kc12 * ks23 + ks12 * kc23 * cos(the_opt[i]);
		float C6 = kc23*(sthe_tmp * sths_tmp - kc12*cthe*cths_tmp) + cths_tmp*ks12*ks23;
		float C7 = cthe*sths_tmp + kc12*cths_tmp*sthe_tmp;

		ROS_INFO("C4 %f",C4);
		ROS_INFO("C5 %f",C5);
		ROS_INFO("C6 %f",C6);
		ROS_INFO("C7 %f",C7);
		ROS_INFO("xx %f",xx);
		ROS_INFO("xy %f",xy);
		
		thr_opt[i] = atan2(
				(xx - C7 * xy / C4) / (C6 + C7*C5/C4),
				(xx + C6 * xy / C5) / (-C6*C4/C5 - C7));

		ROS_INFO("ths_opt %f",ths_opt[i]);
		ROS_INFO("the_opt %f",the_opt[i]);
		ROS_INFO("thr_opt %f",thr_opt[i]);
		
		ths_act[i] = THS_FROM_IK(armId,ths_opt[i]);
		the_act[i] = THE_FROM_IK(armId,the_opt[i]);
		thr_act[i] = THR_FROM_IK(armId,thr_opt[i]);


		ROS_INFO("ths_act %f",ths_act[i]);
		ROS_INFO("the_act %f",the_act[i]);
		ROS_INFO("thr_act %f",thr_act[i]);

	
		bool valid2 = check_joint_limits2_new(ths_act[i],the_act[i],thr_act[i],validity2[i]);
		opts_valid[i] = valid2;

		if (valid2) {
				//set joints
				set_joints_with_limits1(mech,d_act,thp_act,g1_act,g2_act);
				set_joints_with_limits2(mech,ths_act[i],the_act[i],thr_act[i]);

						
		} else {
		    ROS_INFO("check_joint_limits_2_new failed");
		}
	}

	if (opts_valid[0]) {
		return 1;
	} else if (opts_valid[1]) {
		return 1;
	} else {
		const float maxValidDist = 3 DEG2RAD;
		float valid_dist[2];
		for (int i=0;i<2;i++) {
			float sum = 0;
			for (int j=0;j<3;j++) {
				float v = fabs(validity2[i][j]);
				sum += v*v;
			}
			valid_dist[i] = sqrt(sum);
		}

		if (valid_dist[0] < maxValidDist && valid_dist[0] < valid_dist[1]) {
		    set_joints_with_limits1(mech,d_act,thp_act,g1_act,g2_act);
		    set_joints_with_limits2(mech,ths_act[0],the_act[0],thr_act[0]);
		} else if (valid_dist[1] < maxValidDist) {
		    set_joints_with_limits1(mech,d_act,thp_act,g1_act,g2_act);
		    set_joints_with_limits2(mech,ths_act[1],the_act[1],thr_act[1]);
		}

		return 0;
	}
}











bool inv_kin(RavenDebridement::InvKinSrv::Request &req,
	     RavenDebridement::InvKinSrv::Response &res)
{
    

    float x = req.pose.position.x;
    float y = req.pose.position.y;
    float z = req.pose.position.z;


    btQuaternion quat = btQuaternion(req.pose.orientation.x,
				     req.pose.orientation.y,
				     req.pose.orientation.z,
				     req.pose.orientation.w);

    btMatrix3x3 mat = btMatrix3x3(quat);

    struct mechanism* mech = new mechanism();

    if (req.arm_type == raven_2_msgs::Constants::ARM_TYPE_GOLD) {
	mech->type = GOLD_ARM;
    } else if (req.arm_type == raven_2_msgs::Constants::ARM_TYPE_GREEN) {
	mech->type = GREEN_ARM;
    } else {
	return false;
    }

    //mech->ori_d.grasp = req.grasp;


    ROS_INFO("Solve IK for:");
    ROS_INFO("Position (%f,%f,%f)",x,y,z);
    ROS_INFO("Quaternion (%f,%f,%f,%f)",req.pose.orientation.x,req.pose.orientation.y,req.pose.orientation.z,req.pose.orientation.w);

    
    if (!invMechKinNew(mech, x, y, z, mat, req.grasp)) {
	ROS_INFO("IK service call failed");
    	return false;
    }

    int JOINT_NAMES[7] = {SHOULDER,
			  ELBOW,
			  Z_INS,
			  TOOL_ROT,
			  WRIST};

    for(int i = 0; i < 7; i++) {
	raven_2_msgs::JointState *joint = new raven_2_msgs::JointState();
	joint->type = JOINT_NAMES[i];
	joint->state = raven_2_msgs::JointState::STATE_READY;
	joint->position = mech->joint[JOINT_NAMES[i]].jpos_d;
	res.joints.push_back(*joint);
    }

    raven_2_msgs::JointState *joint = new raven_2_msgs::JointState();
    joint->type = YAW;
    joint->state = raven_2_msgs::JointState::STATE_READY;
    joint->position = (mech->joint[GRASP1].jpos_d - mech->joint[GRASP2].jpos_d)/2;
    res.joints.push_back(*joint);

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inv_kin_server_node");
    ros::NodeHandle node;

    ros::ServiceServer service = node.advertiseService("inv_kin_server", inv_kin);
    ROS_INFO("Raven IK server is ready");
    ros::spin();

    return 0;
}

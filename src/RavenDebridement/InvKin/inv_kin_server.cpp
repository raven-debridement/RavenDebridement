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
		limits++;
		mech->joint[Z_INS].jpos_d = Z_INS_MIN_LIMIT;
	} else if (mech->joint[Z_INS].jpos_d  > Z_INS_MAX_LIMIT) {
		limits++;
		mech->joint[Z_INS].jpos_d = Z_INS_MAX_LIMIT;
	}

	if (mech->joint[WRIST].jpos_d  < TOOL_WRIST_MIN_LIMIT) {
		limits++;
		mech->joint[WRIST].jpos_d = TOOL_WRIST_MIN_LIMIT;
	} else if (mech->joint[WRIST].jpos_d  > TOOL_WRIST_MAX_LIMIT) {
		limits++;
		mech->joint[WRIST].jpos_d = TOOL_WRIST_MAX_LIMIT;
	}
	//TODO: limit this elsewhere
	if (fabs(mech->joint[WRIST].jpos_d - mech->joint[WRIST].jpos) > 10 DEG2RAD) {
		if (mech->joint[WRIST].jpos_d > mech->joint[WRIST].jpos) {
			mech->joint[WRIST].jpos_d = mech->joint[WRIST].jpos + 10 DEG2RAD;
		} else {
			mech->joint[WRIST].jpos_d = mech->joint[WRIST].jpos - 10 DEG2RAD;
		}
	}

	if (mech->joint[GRASP1].jpos_d  < TOOL_GRASP1_MIN_LIMIT) {
		limits++;
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MIN_LIMIT;
	} else if (mech->joint[GRASP1].jpos_d  > TOOL_GRASP1_MAX_LIMIT) {
		limits++;
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MAX_LIMIT;
	}

	if (mech->joint[GRASP2].jpos_d  < TOOL_GRASP2_MIN_LIMIT) {
		limits++;
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MIN_LIMIT;
	} else if (mech->joint[GRASP2].jpos_d  > TOOL_GRASP2_MAX_LIMIT) {
		limits++;
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MAX_LIMIT;
	}

	return limits;
}

int set_joints_with_limits2(mechanism* mech, float ths_act, float the_act, float thr_act) {
	mech->joint[SHOULDER].jpos_d = ths_act;
	mech->joint[ELBOW].jpos_d    = the_act;
	//mech->joint[TOOL_ROT].jpos_d = fix_angle(thr_act + M_PI); //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;
	mech->joint[TOOL_ROT].jpos_d = thr_act; //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;

	int limits = 0;
	if (mech->joint[SHOULDER].jpos_d < SHOULDER_MIN_LIMIT) {
		limits++;
		mech->joint[SHOULDER].jpos_d = SHOULDER_MIN_LIMIT;
	} else if (mech->joint[SHOULDER].jpos_d > SHOULDER_MAX_LIMIT) {
		limits++;
		mech->joint[SHOULDER].jpos_d = SHOULDER_MAX_LIMIT;
	}

	if (mech->joint[ELBOW].jpos_d < ELBOW_MIN_LIMIT) {
		limits++;
		mech->joint[ELBOW].jpos_d = ELBOW_MIN_LIMIT;
	} else if (mech->joint[ELBOW].jpos_d > ELBOW_MAX_LIMIT) {
		limits++;
		mech->joint[ELBOW].jpos_d = ELBOW_MAX_LIMIT;
	}

	if (mech->joint[TOOL_ROT].jpos_d < TOOL_ROLL_MIN_LIMIT) {
		limits++;
		mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MIN_LIMIT;
	} else if (mech->joint[TOOL_ROT].jpos_d > TOOL_ROLL_MAX_LIMIT) {
		limits++;
		mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MAX_LIMIT;
	}

	//TODO: limit this elsewhere
	if (fabs(mech->joint[TOOL_ROT].jpos_d - mech->joint[TOOL_ROT].jpos) > 10 DEG2RAD) {
		if (mech->joint[TOOL_ROT].jpos_d > mech->joint[TOOL_ROT].jpos) {
			mech->joint[TOOL_ROT].jpos_d = mech->joint[TOOL_ROT].jpos + 10 DEG2RAD;
		} else {
			mech->joint[TOOL_ROT].jpos_d = mech->joint[TOOL_ROT].jpos - 10 DEG2RAD;
		}
	}

	return limits;
}



int invMechKinNew(struct mechanism *mech, float x, float y, float z, btMatrix3x3 matrix) {
        struct position*    pos_d = &(mech->pos_d);
	struct orientation* ori_d = &(mech->ori_d);


	int armId = armIdFromMechType(mech->type);


	// desired tip position
	//btVector3 actualPoint = btVector3(pos_d->x,pos_d->y,pos_d->z) / MICRON_PER_M;
	btVector3 actualPoint = btVector3(x, y, z) / MICRON_PER_M;
	//btMatrix3x3 actualOrientation = toBt(ori_d->R);
	btMatrix3x3 actualOrientation = matrix;
	btTransform actualPose(actualOrientation,actualPoint);

	
	
	float grasp = GRASP_TO_IK(armId,mech->ori_d.grasp);


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

		thr_opt[i] = atan2(
				(xx - C7 * xy / C4) / (C6 + C7*C5/C4),
				(xx + C6 * xy / C5) / (-C6*C4/C5 - C7));

		ths_act[i] = THS_FROM_IK(armId,ths_opt[i]);
		the_act[i] = THE_FROM_IK(armId,the_opt[i]);
		thr_act[i] = THR_FROM_IK(armId,thr_opt[i]);

	
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





int invMechKin_pos(struct mechanism *mech,float x,float y,float z);
int invMechKin_ori(struct mechanism *mech,btMatrix3x3 mat);

int check_joint_limits1(struct mechanism *_mech);
int check_joint_limits2(struct mechanism *_mech);


int invMechKin(struct mechanism *mech,float x, float y, float z, btMatrix3x3 mat)
{
    if (invMechKin_pos(mech,x,y,z) > 0)
    {
        // enforce joint limits on position DoF
        check_joint_limits1(mech);
        invMechKin_ori(mech,mat);
        // enforce joint limits on tool DoF
        check_joint_limits2(mech);
        return 1;
    }
    else
    {
        ROS_INFO("no_ori");
        return 0;
    }
}

int invMechKin_pos(struct mechanism *mech,float x, float y,float z)
{
    struct position    *pos_d = &(mech->pos_d);

    // output angles for robot
    float j1; //Shoulder
    float j2; //Elbow
    float d3; //Insertion

	// desired tip position
    float Px = x / MICRON_PER_M;//(pos_d->x) / MICRON_PER_M;
    float Py = y / MICRON_PER_M;//(pos_d->y) / MICRON_PER_M;
    float Pz = z / MICRON_PER_M;//(pos_d->z) / MICRON_PER_M;

    ROS_INFO("(x, y, z) = (%f, %f, %f)",x,y,z);
    ROS_INFO("(Px, Py, Pz) = (%f, %f, %f)",Px,Py,Pz);

	/// Solve for d3 (insertion) - always negative
	d3 = - sqrt(SQR(Px) + SQR(Py) + SQR(Pz));

    // Avoid mechanism singularity at d3=0
    const float min_insertion=0.001;
	if (fabs(d3) < min_insertion)
	{
	    ROS_INFO("mechanism singularity at d3");
	    ROS_INFO("fabs(d3) = %f",fabs(d3));
	    return 0;
	}

	/// ** Solve for j2 (Elbow) **
	float cbe=1, sbe=0, cal=1, sal=0;
	float c2,s2;
    if (mech->type == GOLD_ARM)
    {
        cbe = cos(M_PI - A23); 	// cos(Pi - beta)
        sbe = sin(M_PI - A23);  // sin(Pi - beta)
        cal = cos(-A12);        // cos(-alpha)
        sal = sin (-A12);       // sin(-alpha)
        c2 = (Pz - cbe * cal * d3)/( - sbe *  d3 * sal);
    }
    else
    {
        cbe = cos(A23);  // cos(beta)
        sbe = sin(A23);  // sin(beta)
        cal = cos(A12);  // cos(alpha)
        sal = sin(A12);  // sin(alpha)
        c2 = (Pz + cbe * cal * d3)/( sbe *  d3 * sal);
    }

    if (c2>1) c2=1;
    else if (c2<-1) c2=-1;

	// s2 is always positive, since 0<j2<180
	// This resolves multiple solutions
	s2 = sqrt(1-(c2*c2));
	j2 = atan2f(s2,c2);


	/// ** Solve for j1 (Shoulder) **
	float s1,c1;
    float a,b;
    if (mech->type == GOLD_ARM)
    {
        a = sbe*d3*s2;
        b = sbe*d3*c2*cal + cbe*d3*sal;
        c1 = (Px*a - Py*b) / (a*a + b*b);
        s1 = (Px*b + Py*a) / (a*a + b*b);
        s1 = fabs(s1); // s1 always positive, since 0<j1<90; resolves multiple sol'n
        c1 = fabs(c1); // c1 always positive, since 0<j1<90;
        j1 = atan2f ( s1, c1 ) - base_tilt;
    }
    else
    {
        s2=-s2;
        a = sbe*d3*s2;
        b = sbe*d3*c2*cal + cbe*d3*sal;

        c1 = (Px*a - Py*b) / (a*a + b*b);
        s1 = (Py*a + Px*b) / (-(a*a) - (b*b));
        s1 = fabs(s1); // s1 always positive, since 0<j1<90;
        c1 = fabs(c1); // c1 always positive, since 0<j1<90;
        j1 = atan2f ( s1, c1 ) - base_tilt;
    }

	// Now have solved for th1, th2, d3 ...
    mech->joint[SHOULDER].jpos_d = j1;
    mech->joint[ELBOW].jpos_d    = j2;
    mech->joint[Z_INS].jpos_d    = d3;

	return 1;   // actually has a solution
}

/**
*   Calculate the tool DOF angles from the desired orientation
*
*   Assumes that the shoulder, elbow, z have been calculated from desired position.
*
*   The rotation from base frame to tool frame is premultiplied to the
*   desired transform.  The resulting rotation is used to calculate the desired
*   tool angles.
*
*   Desired transform TD_0_6:  TD_3_0 * TD_0_6 = TD_3_6
*   TD_3_6 is then the desired transform from tool frame to tip.
*/
int invMechKin_ori(struct mechanism *mech, btMatrix3x3 mat)
{
    struct orientation   *ori_d = &(mech->ori_d);

    double j1 = mech->joint[SHOULDER].jpos_d;
	double j2 = mech->joint[ELBOW].jpos_d;
	double j3 = mech->joint[Z_INS].jpos_d;

    btMatrix3x3 xf;
//    (ori_d->R[0][0], ori_d->R[0][1], ori_d->R[0][2],
//                       ori_d->R[1][0], ori_d->R[1][1], ori_d->R[1][2],
//                       ori_d->R[2][0], ori_d->R[2][1], ori_d->R[2][2]);
	// copy R matrix
	// TODO: use above commented code.
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		    xf[i][j] = mat[i][j];//ori_d->R[i][j];

    // variable DH Parameters for gold and green arms
    double go_dh_d[3]  = {0,              0,      j3};
    double go_dh_th[3] = {j1 + base_tilt, j2,     0};

    double gr_dh_d[3]  = {0,              0,      j3};
    double gr_dh_th[3] = {j1 + base_tilt, j2,     0};

    double dh_al[6];
    double dh_a[6];
    double dh_d[3];
    double dh_th[3];

    if (mech->type == GOLD_ARM_SERIAL)
    {
        memcpy(dh_al, go_dh_al, 6*sizeof(double));
        memcpy(dh_a,  go_dh_a,  6*sizeof(double));
        memcpy(dh_d,  go_dh_d,  3*sizeof(double));
        memcpy(dh_th, go_dh_th,  3*sizeof(double));
    }
    else if (mech->type == GREEN_ARM_SERIAL)
    {
        memcpy(dh_al, gr_dh_al, 6*sizeof(double));
        memcpy(dh_a,  gr_dh_a,  6*sizeof(double));
        memcpy(dh_d,  gr_dh_d,  3*sizeof(double));
        memcpy(dh_th, gr_dh_th, 3*sizeof(double));
    }

    // Generate transformation matrices from DH parameters
    btMatrix3x3 temp;
    btMatrix3x3 Rot;
    Rot.setIdentity();

    btScalar cti, sti;  // cos(thetai), sin(thetai)
    btScalar cai, sai;  // cos(alphai), sin(alphai)

    /// Calculate the transform from base frame to tool frame
    for (int i=0;i<3;i++)
    {
        cti = cos(dh_th[i]);
        sti = sin(dh_th[i]);
        cai = cos(dh_al[i]);
        sai = sin(dh_al[i]);

        temp[0][0] = cti;
        temp[0][1] = -sti;
        temp[0][2] = 0;

        temp[1][0] = sti * cai;
        temp[1][1] = cti * cai;
        temp[1][2] = -sai;

        temp[2][0] = sti * sai;
        temp[2][1] = cti * sai;
        temp[2][2] = cai;

        Rot = Rot * temp;
    }

    /// Get the desired transform in tool frame
    /// Premultiply by inverse transform from base frame to tool frame
    xf = Rot.inverse() * xf;

//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++)
//			mech->ori_d.R[i][j] = xf[i][j];

    float j4[2];
    float j5[2];
    float j6[2];
    float j4_check, min_j4;;

    /// ** Solve j4 (tool roll) **
    if (mech->type == GOLD_ARM)
    {
        j4[0] = atan2f( -xf[1][2], -xf[0][2] );
        j4[1] = atan2f( xf[1][2], xf[0][2] );
        j4_check = mech->joint[TOOL_ROT].jpos - M_PI/2;
    }
    else
    {
        j4[0] = atan2f( -xf[1][2], xf[0][2] );
        j4[1] = atan2f( xf[1][2], -xf[0][2] );
        j4_check = mech->joint[TOOL_ROT].jpos - M_PI/2;
    }

    // select between multiple solutions by the smallest difference from current angle.
    min_j4 = j4[0];
    int whichcase=0;
    if ( fabsf(j4_check - j4[1]) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1];
        whichcase=1;
    }
    //check rollover conditions
    if ( fabsf(j4_check - (j4[1]+2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1]+2*M_PI;
        whichcase=2;
    }
    if ( fabsf(j4_check - (j4[0]+2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[0]+2*M_PI;
        whichcase=3;
    }
    if ( fabsf(j4_check - (j4[1]-2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1]-2*M_PI;
        whichcase=4;
    }
    if ( fabsf(j4_check - (j4[0]-2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[0]-2*M_PI;
        whichcase=5;
    }
//
//    if (mech->type == GREEN_ARM)
//    {
//        ROS_INFO("cs:%d j4c:%0.3f\t 0:%0.3f\t 1:%0.3f\t 2:%0.3f\t 3:%0.3f\t 4:%0.3f\t 5:%0.3f",
//            whichcase, j4_check,
//            j4[0],j4[1],
//            j4[1]+2*M_PI,
//            j4[0]+2*M_PI,
//            j4[1]-2*M_PI,
//            j4[0]-2*M_PI);
//    }

    if (mech->type == GOLD_ARM)
        mech->joint[TOOL_ROT].jpos_d = min_j4 + M_PI/2;
    else
        mech->joint[TOOL_ROT].jpos_d = -min_j4 - M_PI/2;

    ///  ** Solve j5 (wrist) **
    float c4 = cosf(min_j4);
    float s5 = xf[0][2] / c4;
    float c5 = xf[2][2];
    j5[0] = atan2f(-s5,-c5);

    if (mech->type == GOLD_ARM)
        mech->joint[WRIST].jpos_d  = j5[0] - M_PI/2;
    else
        mech->joint[WRIST].jpos_d  = -j5[0] + M_PI/2;

    /// ** Solve j6 (avg grasp) **
//    float s5_sign = (s5 < 0) ? -1:1;
    float s6 = xf[2][1]/-s5;
    float c6 = xf[2][0]/s5;
    j6[0] = atan2f(s6, c6);

    float grasp = ((float)mech->ori_d.grasp)/1000.0f;
    //ROS_INFO("%d:grasp:%0.3f", mech->type, grasp);


    if (mech->type == GOLD_ARM)
    {
        mech->joint[GRASP1].jpos_d = grasp/2;
        mech->joint[GRASP2].jpos_d = grasp/2;
    }
    else
    {
        mech->joint[GRASP1].jpos_d = grasp/2;
        mech->joint[GRASP2].jpos_d = grasp/2;
    }





//    float openangle;
//    if (mech->type == GOLD_ARM)
//        openangle = 45 DEG2RAD; //ori_d->grasp/1000;
//    else
//        openangle = -45 DEG2RAD;//ori_d->grasp/1000;
//
////    ROS_INFO("g(%d):%0.3f, \t (%0.3f)", mech->type, openangle, (float)mech->ori_d.grasp/1000);
//    if (mech->type == GOLD_ARM)
//    {
//        mech->joint[GRASP1].jpos_d = j6[0]+openangle/2;
//        mech->joint[GRASP2].jpos_d = j6[0]-openangle/2;
//    }
//    else
//    {
//        mech->joint[GRASP1].jpos_d = j6[0]-openangle/2;
//        mech->joint[GRASP2].jpos_d = j6[0]+openangle/2;
//    }

    mech->joint[TOOL_ROT].jpos_d = 0;
    mech->joint[WRIST].jpos_d = 0;

    return 1;
}







/**
* Check for joint.angle_d exceeding joint angles.
*     --> position
*/
int check_joint_limits1(struct mechanism *_mech)
{


    int limits = 0;
	if (_mech->joint[SHOULDER].jpos_d < SHOULDER_MIN_LIMIT)
	{
	    limits++;
	    ROS_INFO("shoulder limit");
		_mech->joint[SHOULDER].jpos_d = SHOULDER_MIN_LIMIT;
	}
	else if (_mech->joint[SHOULDER].jpos_d > SHOULDER_MAX_LIMIT)
	{
	    limits++;
	    ROS_INFO("shoulder limit");
		_mech->joint[SHOULDER].jpos_d = SHOULDER_MAX_LIMIT;
	}
	if (_mech->joint[ELBOW].jpos_d < ELBOW_MIN_LIMIT)
	{
	    limits++;
        ROS_INFO("elbow limit");
		_mech->joint[ELBOW].jpos_d = ELBOW_MIN_LIMIT;
	}
	else if (_mech->joint[ELBOW].jpos_d > ELBOW_MAX_LIMIT)
	{
 	    limits++;
        ROS_INFO("elbow limit");
		_mech->joint[ELBOW].jpos_d = ELBOW_MAX_LIMIT;
	}
	if (_mech->joint[Z_INS].jpos_d < Z_INS_MIN_LIMIT)
	{
	    limits++;
        ROS_INFO("Zins limit");
		_mech->joint[Z_INS].jpos_d = Z_INS_MIN_LIMIT;
	}
	else if (_mech->joint[Z_INS].jpos_d > Z_INS_MAX_LIMIT)
	{
	    limits++;
        ROS_INFO("Zins limit");
		_mech->joint[Z_INS].jpos_d = Z_INS_MAX_LIMIT;
	}

	return limits;
}
/**
* Check for joint.angle_d exceeding joint angles.
*     --> orientation
*/
int check_joint_limits2(struct mechanism *_mech)
{
    int limits=0;

	if (_mech->joint[TOOL_ROT].jpos_d  < TOOL_ROLL_MIN_LIMIT)
	{
	    limits++;
        ROS_INFO("roll limit");
		_mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MIN_LIMIT;
	}
	else if (_mech->joint[TOOL_ROT].jpos_d  > TOOL_ROLL_MAX_LIMIT)
	{
	    limits++;
        ROS_INFO("roll limit");
		_mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MAX_LIMIT;
	}
	if (_mech->joint[WRIST].jpos_d  < TOOL_WRIST_MIN_LIMIT)
	{
        ROS_INFO("wrist limit");
	    limits++;
		_mech->joint[WRIST].jpos_d = TOOL_WRIST_MIN_LIMIT;
	}
	else if (_mech->joint[WRIST].jpos_d  > TOOL_WRIST_MAX_LIMIT)
	{
	    limits++;
        ROS_INFO("wrist limit");
		_mech->joint[WRIST].jpos_d = TOOL_WRIST_MAX_LIMIT;
	}

	if (_mech->joint[GRASP1].jpos_d  < TOOL_GRASP1_MIN_LIMIT)
	{
        ROS_INFO("grasp1 limit");
	    limits++;
		_mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MIN_LIMIT;
	}
	else if (_mech->joint[GRASP1].jpos_d  > TOOL_GRASP1_MAX_LIMIT)
	{
	    limits++;
        ROS_INFO("grasp1 limit");
		_mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MAX_LIMIT;
	}

	if (_mech->joint[GRASP2].jpos_d  < TOOL_GRASP2_MIN_LIMIT)
	{
        ROS_INFO("grasp2 limit");
	    limits++;
		_mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MIN_LIMIT;
	}
	else if (_mech->joint[GRASP2].jpos_d  > TOOL_GRASP2_MAX_LIMIT)
	{
	    limits++;
        ROS_INFO("grasp2 limit");
		_mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MAX_LIMIT;
	}

	return limits;
}







bool inv_kin(RavenDebridement::InvKinSrv::Request &req,
	     RavenDebridement::InvKinSrv::Response &res)
{
    

    struct position pos;
    pos.x = req.pose.position.x;
    pos.y = req.pose.position.y;
    pos.z = req.pose.position.z;


    btQuaternion quat = btQuaternion(req.pose.orientation.x,
				     req.pose.orientation.y,
				     req.pose.orientation.z,
				     req.pose.orientation.w);

    btMatrix3x3 mat = btMatrix3x3(quat);


    
    struct orientation ori = orientation();
    for(int i = 0; i < 3; i++) {
	for(int j = 0; j < 3; j++) {
	    ori.R[i][j] = mat[i][j];
	}
    }


    
    struct mechanism* mech = new mechanism();

    if (req.arm_type == raven_2_msgs::Constants::ARM_TYPE_GOLD) {
	mech->type = GOLD_ARM;
    } else if (req.arm_type == raven_2_msgs::Constants::ARM_TYPE_GREEN) {
	mech->type = GREEN_ARM;
    } else {
	return false;
    }


    mech->pos_d = pos;
    mech->ori_d = ori;

    // temp
    mech->ori_d.grasp = 0;

    for(int i = 0; i < 3; i++) {
	ROS_INFO("%f %f %f",mech->ori_d.R[i][0],mech->ori_d.R[i][1],mech->ori_d.R[i][2]);
    }

    ROS_INFO("Solve IK for:");
    ROS_INFO("Position (%f,%f,%f)",req.pose.position.x,req.pose.position.y,req.pose.position.z);
    ROS_INFO("Quaternion (%f,%f,%f,%f)",req.pose.orientation.x,req.pose.orientation.y,req.pose.orientation.z,req.pose.orientation.w);

    
    if (!invMechKinNew(mech,req.pose.position.x,req.pose.position.y,req.pose.position.z,mat)) {
	ROS_INFO("IK service call failed");
    	return false;
    }

    for(int i = 0; i < MAX_DOF_PER_MECH; i++) {
	raven_2_msgs::JointState *joint = new raven_2_msgs::JointState();
	joint->type = mech->joint[i].type;
	joint->state = raven_2_msgs::JointState::STATE_READY;
	joint->position = mech->joint[i].jpos_d;
	res.joints.push_back(*joint);
    }

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inv_kin_server_node");
    ros::NodeHandle node;

    ros::ServiceServer service = node.advertiseService("inv_kin_server", inv_kin);
    ROS_INFO("Service is ready");
    ros::spin();

    return 0;
}

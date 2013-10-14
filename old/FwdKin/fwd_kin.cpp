/*******************************
 *
 *  File: fwd_kinmatics.c
 *
 *  Calculate the forward kinematics point P (px, py,pz) from joint angles
 *  expressed in base frame
 *
 *   Updated 25-Aug-2011 by HK
 *     Changed to work for Raven_II.  Code taken from UCSC implementation
 *
 **********************************/

#include <stdlib.h>
#include <tf/transform_datatypes.h>
#include <LinearMath/btTransform.h>



#include <iostream>
#include <sstream>


#include <math.h>
#include <tf/transform_datatypes.h>

//#include "local_io.h"
#include "raven/hmatrix.h"
#include "raven/utils.h"
#include "raven/defines.h"

#include "raven/kinematics/kinematics_defines.h"

#include <raven/state/runlevel.h>


using namespace std;

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

/*
 * fwdKin - wrapper function that checks for correct runlevel
 *   and calls fwdMechKin for each mechanism in device
 *
 * inputs: device - pointer to device struct
 *         runlevel - current runlevel
 *
 */


btTransform fwdMechKinNew(struct mechanism* mech) {

	int armId = armIdFromMechType(mech->type);

	btTransform tool = actual_world_to_ik_world(armId)
					* Tw2b
					* Zs(THS_TO_IK(armId,mech->joint[SHOULDER].jpos))
					* Xu
					* Ze(THE_TO_IK(armId,mech->joint[ELBOW].jpos))
					* Xf
					* Zr(THR_TO_IK(armId,mech->joint[TOOL_ROT].jpos))
					* Zi(D_TO_IK(armId,mech->joint[Z_INS].jpos))
					* Xip
					* Zp(THP_TO_IK(armId,mech->joint[WRIST].jpos))
					* Xpy
					* Zy(THY_TO_IK_FROM_FINGERS(armId,mech->joint[GRASP1].jpos,mech->joint[GRASP2].jpos))
					* Tg;

	btMatrix3x3 rot = tool.getBasis() * btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1);
	tool.setBasis(rot);

	return tool;
}

int main(int argc, char **argv) {
	argv = argv+1;
	argc = argc-1;

	for(int i=0; i < argc; i++) {
		cout << "i: " << i << " , " << argv[i] << endl;
	}

	// argv is armId, joints 0-6
	if (argc != 8) {
		cout << "Incorrect input!" << endl;
		return -1;
	}

	string armName = string(argv[0]);
	double shoulder = atof(argv[1]);
	double elbow = atof(argv[2]);
	double insertion = atof(argv[3]);
	double rotation = atof(argv[4]);
	double pitch = atof(argv[5]);
	double finger1 = atof(argv[6]);
	double finger2 = atof(argv[7]);

	mechanism mech;
	if (armName == "L") {
		mech.type = GOLD_ARM;
	} else if (armName == "R") {
		mech.type = GREEN_ARM;
	}
	mech.joint[SHOULDER].jpos = shoulder;
	mech.joint[ELBOW].jpos = elbow;
	mech.joint[Z_INS].jpos = insertion;
	mech.joint[TOOL_ROT].jpos = rotation;
	mech.joint[WRIST].jpos = pitch;
	mech.joint[GRASP1].jpos = finger1;
	mech.joint[GRASP2].jpos = finger2;

	btTransform tool_trans = fwdMechKinNew(&mech);

	btMatrix3x3 ori = tool_trans.getBasis();
	btVector3 pos    = tool_trans.getOrigin();

	cout << "Tool position :" << endl << "(" << pos[0] << "," << pos[1] << "," << pos[2] << ")" << endl;
	cout << "Tool orientation :" << endl << tool_trans << endl;

	return 0;
}




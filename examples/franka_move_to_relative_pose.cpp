#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "robot.h"
#include "utils.h"
#include "lssab_trajectory.h"

using namespace franka_control;
using namespace Eigen;


int main(int argc, char** argv) {

	if (argc != 8) {
		std::cerr << "Usage: ./franka_move_to_relative_pose "
				<< "robot_ip x y z roll pitch yaw" << std::endl;
		return -1;
	}

	try {
		franka_control::Robot robot(argv[1]);
		robot.setDefaultBehavior();

		// Get translation delta from command line params
		Vector3d deltaTrans;
		deltaTrans << atof(argv[2]), atof(argv[3]), atof(argv[4]);

		// Get rotation delta from command line params
		Vector3d deltaRotE;
		deltaRotE << atof(argv[5]), atof(argv[6]), atof(argv[7]);

		// Convert rotation delta from degrees to radians
		deltaRotE *= M_PI / 180;

		// Convert rotation delta to quaternion form
		Quaterniond deltaRotQ = AngleAxisd(deltaRotE(0), Vector3d::UnitX())
			* AngleAxisd(deltaRotE(1), Vector3d::UnitY())
			* AngleAxisd(deltaRotE(2), Vector3d::UnitZ());

    	// Use current/previous target pose as initial pose
		std::tuple<Vector3d, Quaterniond> initPose = qcoeff2quat(robot.getDesiredPose());
		Vector3d initTrans = std::get<0>(initPose);
		Quaterniond initRotQ = std::get<1>(initPose);

		// Compute target pose
		Vector3d finalTrans = initTrans + deltaTrans;
		Quaterniond finalRotQ = deltaRotQ * initRotQ;

		robot.moveLinearPosition(quat2qcoeff(std::make_pair(finalTrans, finalRotQ)));

	} catch (const franka::Exception& e) {
		std::cerr << e.what() << std::endl;
		return -1;
	}
	return 0;
}



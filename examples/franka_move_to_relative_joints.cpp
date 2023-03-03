#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "robot.h"
#include "utils.h"
#include "lssab_trajectory.h"

using namespace franka_control;


int main(int argc, char** argv) {

	if (argc != 9) {
		std::cerr << "Usage: ./franka_move_to_relative_joints "
				<< "robot_ip j1 j2 j3 j4 j5 j6 j7" << std::endl;
		return -1;
	}

	try {
		franka_control::Robot robot(argv[1]);
		robot.setDefaultBehavior();

		// Get joints delta from command line params
		Vector7d deltaJoints;
		deltaJoints << atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]),
				atof(argv[6]), atof(argv[7]), atof(argv[8]);

		// Convert joints delta from degrees to radians
		deltaJoints *= M_PI / 180;

    	// Use current/previous target joints as initial joints
		Vector7d initJoints = robot.getDesiredJointPosition();

		// Compute target joints
		Vector7d finalJoints = initJoints + deltaJoints;

		robot.moveJointPosition(finalJoints);

	} catch (const franka::Exception& e) {
		std::cerr << e.what() << std::endl;
		return -1;
	}
	return 0;
}



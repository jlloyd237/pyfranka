#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "robot.h"
#include "utils.h"

using namespace franka_control;
using namespace Eigen;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds


int main(int argc, char** argv) {

	if (argc != 9) {
		std::cerr << "Usage: ./franka_move_velocity "
				<< "robot_ip dx dy dz droll dpitch dyaw dt" << std::endl;
		return -1;
	}

	try {
		franka_control::Robot robot(argv[1]);
		robot.setDefaultBehavior();

		// Get translation velocity from command line params
		Vector3d transVel;
		transVel << atof(argv[2]), atof(argv[3]), atof(argv[4]);

		// Get rotation velocity from command line params
		Vector3d rotVel;
		rotVel << atof(argv[5]), atof(argv[6]), atof(argv[7]);

		// Convert rotation delta from degrees/s to radians/s
		rotVel *= M_PI / 180;

		Vector6d vel;
		vel << transVel, rotVel;

		// Get time to move for from command line params
		int dt = atof(argv[8]);

		// Do the move for the specified time
		robot.moveLinearVelocity(vel);
	    sleep_for(milliseconds(dt));

//	    vel[1] = vel[0];
//	    vel[0] = 0.0;
//		robot.moveLinearVelocity(vel);
//	    sleep_for(milliseconds(dt));
//	    vel[2] = vel[1];
//	    vel[1] = 0.0;
//		robot.moveLinearVelocity(vel);
//	    sleep_for(milliseconds(dt));

		robot.moveLinearVelocity(Vector6d::Zero());

	} catch (const franka::Exception& e) {
		std::cerr << e.what() << std::endl;
		return -1;
	}
	return 0;
}



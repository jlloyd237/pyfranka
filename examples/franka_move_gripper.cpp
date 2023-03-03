#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "robot.h"
#include "gripper.h"
#include "utils.h"
#include "lssab_trajectory.h"

using namespace franka_control;
using namespace Eigen;


int main(int argc, char** argv) {

	if (argc != 5) {
		std::cerr << "Usage: ./franka_move_gripper "
				<< "robot_ip grasp_width grasp_speed grasp_force" << std::endl;
		return -1;
	}

	try {
		franka_control::Gripper gripper(argv[1]);

		std::cout << "Temperature: " << gripper.temperature() << std::endl;
		std::cout << "Max width: " << gripper.maxWidth() << std::endl;

		std::cout << "Homing ..." << std::endl;
		gripper.homing();

		std::cout << "Width (before grasp): " << gripper.width() << std::endl;
		std::cout << "Is grasped (before grasp): " << gripper.isGrasped() << std::endl;

		std::cout << "Grasping ..." << std::endl;
		gripper.grasp(atof(argv[2]), atof(argv[3]), atof(argv[4]));

		std::cout << "Width (after grasp): " << gripper.width() << std::endl;
		std::cout << "Is grasped (after grasp): " << gripper.isGrasped() << std::endl;

	} catch (const franka::Exception& e) {
		std::cerr << e.what() << std::endl;
		return -1;
	}
	return 0;
}



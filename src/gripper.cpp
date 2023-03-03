#include <franka/gripper.h>

#include "gripper.h"

namespace franka_control {

	Gripper::Gripper(const std::string& ip)
	: gripper_(ip)
	{
	}

	bool Gripper::homing() const {
		return gripper_.homing();
	}

	bool Gripper::grasp(double width, double speed, double force,
			double epsilonInner, double epsilonOuter) const {
		return gripper_.grasp(width, speed, force, epsilonInner, epsilonOuter);
	}

	bool Gripper::move(double width, double speed) const {
		return gripper_.move(width, speed);
	}

	double Gripper::width() const {
		auto state = gripper_.readOnce();
		return state.width;
	}

	double Gripper::maxWidth() const {
		auto state = gripper_.readOnce();
		return state.max_width;
	}

	bool Gripper::isGrasped() const {
		auto state = gripper_.readOnce();
		return state.is_grasped;
	}

	uint16_t Gripper::temperature() const {
		auto state = gripper_.readOnce();
		return state.temperature;
	}

	bool Gripper::stop() const {
		return gripper_.stop();
	}

	unsigned Gripper::serverVersion () const noexcept {
		return gripper_.serverVersion();
	}

}	// namespace franka_control


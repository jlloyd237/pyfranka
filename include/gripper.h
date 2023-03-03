#ifndef FRANKA_CONTROL_GRIPPER_H_
#define FRANKA_CONTROL_GRIPPER_H_

#include <string>
#include <cstdint>

#include <franka/gripper.h>

namespace franka_control {

	class Gripper {

	public:

		explicit Gripper(const std::string& ip);

		// Performs homing of the gripper
		// Returns true if command was successful; false otherwise
		bool homing() const;

		// Grasps an object.
		// An object is considered grasped if the distance between the gripper fingers
		// satisfies width - epsilonInner < distance < width + epsilon_outer
		// width - size of the object to grasp (m)
		// speed - closing speed (m/s)
		// force - grasping force(N)
		// epsilonInner (m)
		// epsilonOuter (m)
		// Returns true if an object has been grasped; false otherwise
		bool grasp(double width, double speed, double force, double epsilon_inner=0.005,
				double epsilon_outer=0.005) const;

		// Moves the gripper fingers to a specified width
		// width - intended opening width (m)
		// speed - closing speed (m/s)
		// Returns true if command was successful; false otherwise
		bool move(double width, double speed) const;

		// Current gripper opening width (m)
		double width() const;

		// Maximum gripper opening width (m)
		double maxWidth() const;

		// Indicates whether an object is currently grasped
		bool isGrasped() const;

		// Current gripper temperature
		uint16_t temperature() const;

		// Stops a currently running gripper move or grasp
		// Returns true if command was successful; false otherwise
		bool stop() const;

		// Returns the software version reported by the connected server
		unsigned serverVersion () const noexcept;

	private:

		franka::Gripper gripper_;

	};

}	// namespace franka_control


#endif	// FRANKA_CONTROL_GRIPPER_H_

#ifndef FRANKA_CONTROL_SSAB_TRAJECTORY_H_
#define FRANKA_CONTROL_SSAB_TRAJECTORY_H_

#include "trajectory.h"


namespace franka_control {

	class SSABTrajectory : public Trajectory {

	public:
		SSABTrajectory();
		SSABTrajectory(double v0, double v1, double amax, double jmax, bool norm = false);

		virtual double operator()(double t);
		virtual double duration() { return tf_; }

	private:
		double v0_;				// start velocity
		double v1_;				// finish velocity
		double amax_;			// max acceleration (magnitude)
		double jmax_;			// max jerk (magnitude)
		bool norm_;				// normalize trajectory

		double tf_;				// Overall trajectory duration
		double amaxSigned_;		// Signed maximum acceleration
	};

}	// namespace franka_control


#endif	// FRANKA_CONTROL_SSAB_TRAJECTORY_H_


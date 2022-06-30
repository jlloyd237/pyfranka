#ifndef FRANKA_CONTROL_LSSAB_TRAJECTORY_H_
#define FRANKA_CONTROL_LSSAB_TRAJECTORY_H_

#include "trajectory.h"


namespace franka_control {

	class LSSABTrajectory : public Trajectory {

	public:
		LSSABTrajectory();
		LSSABTrajectory(double q0, double q1, double vmax, double amax, double jmax, bool norm = false);

		virtual double operator()(double t);
		virtual double duration() { return tf_; }

	private:
		double q0_;				// start position
		double q1_;				// finish position
		double vmax_;			// max velocity (magnitude)
		double amax_;			// max acceleration (magnitude)
		double jmax_;			// max jerk (magnitude)
		bool norm_;				// normalize trajectory

		double tb_;				// Blend time/duration
		double tl_;				// Linear segment time/duration
		double tf_;				// Overall trajectory duration
		double amaxSigned_;		// Signed maximum acceleration
	};

}	// namespace franka_control


#endif	// FRANKA_CONTROL_LSSAB_TRAJECTORY_H_


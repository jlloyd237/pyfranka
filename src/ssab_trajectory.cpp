#include "ssab_trajectory.h"

#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

#include "utils.h"


namespace franka_control {

	using namespace std;

	SSABTrajectory::SSABTrajectory()
		: v0_(0.0)
		, v1_(0.0)
		, amax_(0.0)
		, jmax_(0.0)
		, norm_(false)
		, tf_(0.0)
		, amaxSigned_(0.0)
	{
	}

	SSABTrajectory::SSABTrajectory(double v0, double v1, double amax, double jmax, bool norm)
		: v0_(v0)
		, v1_(v1)
		, amax_(amax)
		, jmax_(jmax)
		, norm_(norm)
		, tf_(0.0)
		, amaxSigned_(0.0)
	{
		if (v1 != v0) {
			// Clip amax so that it is compatible with other constraints
			double amaxClipped = min({
				amax,
				sqrt(2 * jmax * abs(v1 - v0) / M_PI)
			});

			// Overall trajectory duration
			tf_ = 2 * abs(v1 - v0) / amaxClipped;

			// Signed maximum acceleration
			amaxSigned_ = amaxClipped * sgn(v1 - v0);
		}
	}

	double SSABTrajectory::operator()(double t) {
		double v;

		// Restrict t to trajectory time limits
		t = min(max(t, 0.0), tf_);

		// Blend velocities
		v = ((amaxSigned_ * tf_) / (4 * M_PI)) * (2 * M_PI * t / tf_ - sin(2 * M_PI * t / tf_));

		if (norm_) {
			// Normalize value in range [0, 1]
			v /= (v1_ - v0_);
		} else {
			// Add initial offset
			v += v0_;
		}

		return v;
	}

}	// namespace franka_control

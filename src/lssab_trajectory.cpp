#include "lssab_trajectory.h"

#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

#include "utils.h"


namespace franka_control {

	using namespace std;

	LSSABTrajectory::LSSABTrajectory()
		: q0_(0.0)
		, q1_(0.0)
		, vmax_(0.0)
		, amax_(0.0)
		, jmax_(0.0)
		, norm_(false)
		, tb_(0.0)
		, tl_(0.0)
		, tf_(0.0)
		, amaxSigned_(0.0)
	{
	}

	LSSABTrajectory::LSSABTrajectory(double q0, double q1, double vmax, double amax, double jmax, bool norm)
		: q0_(q0)
		, q1_(q1)
		, vmax_(vmax)
		, amax_(amax)
		, jmax_(jmax)
		, norm_(norm)
		, tb_(0.0)
		, tl_(0.0)
		, tf_(0.0)
		, amaxSigned_(0.0)
	{
		if (q1 != q0) {
			// Clip amax so that it is compatible with other constraints
			double amaxClipped = min({
				amax,
				sqrt(2 * vmax * jmax / M_PI),
				cbrt(2 * abs(q1 - q0) * pow(jmax, 2) / pow(M_PI, 2))
			});

			// Jerk constraint on blend time (lower bound, given amax and jmax)
			double tb1 = M_PI * amaxClipped / jmax;

			// Velocity constraint on blend time (upper bound, given vmax and amax)
			double tb2 = 2 * vmax / amaxClipped;

			// Displacement constraint on blend time (upper bound, given amax, q0 and q1)
			double tb3 = sqrt(2 * abs(q1 - q0) / amaxClipped);

			// Blend time/duration
			tb_ = min(tb2, tb3);
//			if (tb_ < tb1) {
//				// This might be redundant after clipping amax?
//				ostringstream errMsg;
//				errMsg << "Jerk constraint incompatible with other constraints - try "
//						"reducing max acceleration or increasing max jerk (q0: "
//						<< q0 << ", q1: " << q1 << ", vmax: " << vmax << ", amax (clipped): "
//						<< amaxClipped << ", jmax: " << jmax << ")";
//				throw runtime_error(errMsg.str());
//			}

			// Linear segment time/duration
			tl_ = 2 * abs(q1 - q0) / (amaxClipped * tb_) - tb_;

			// Overall trajectory duration
			tf_ = 2 * tb_ + tl_;

			// Signed maximum acceleration
			amaxSigned_ = amaxClipped * sgn(q1 - q0);
		}
	}

	double LSSABTrajectory::operator()(double t) {
		double q;

		// Restrict t to trajectory time limits
		t = min(max(t, 0.0), tf_);

		if (t <= tb_) {
			// Initial blend ...
			q = (((amaxSigned_ * tb_) / (4 * M_PI)) * (M_PI * pow(t, 2) / tb_ + tb_ * cos(2 * M_PI * t / tb_) / (2 * M_PI) - tb_ / (2 * M_PI)));
		} else if (t <= (tb_ + tl_)) {
			// Linear segment ...
			q = amaxSigned_ * pow(tb_, 2) / 4 + (t - tb_) * amaxSigned_ * tb_ / 2;
		} else {
			// Final blend ...
			q = amaxSigned_ * pow(tb_, 2) / 4 + tl_ * amaxSigned_ * tb_ / 2 + amaxSigned_ * pow(tb_, 2) / 4 - ((amaxSigned_ * tb_) / (4 * M_PI)) * (M_PI * pow(tf_ - t, 2) / tb_ + tb_ * cos(2 * M_PI * (tf_ - t) / tb_) / (2 * M_PI) - tb_ / (2 * M_PI));
		}

		if (norm_) {
			// Normalize value in range [0, 1]
			q /= (q1_ - q0_);
		} else {
			// Add initial offset
			q += q0_;
		}

		return q;
	}

}	// namespace franka_control

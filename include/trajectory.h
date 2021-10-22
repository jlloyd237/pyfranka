#ifndef FRANKA_CONTROL_TRAJECTORY_H_
#define FRANKA_CONTROL_TRAJECTORY_H_


namespace franka_control {

	class Trajectory {
	public:
		virtual double operator()(double t) = 0;
		virtual double duration() = 0;
	};

}	// namespace franka_control


#endif	// FRANKA_CONTROL_TRAJECTORY_H_

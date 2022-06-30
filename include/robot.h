#ifndef FRANKA_CONTROL_ROBOT_H_
#define FRANKA_CONTROL_ROBOT_H_

#include <array>
#include <string>
#include <thread>
#include <mutex>

#include <Eigen/Dense>
#include <franka/robot.h>

#include "utils.h"
#include "lssab_trajectory.h"
#include "ssab_trajectory.h"

namespace franka_control {

	using namespace Eigen;

	typedef Matrix<double, 2, 1> Vector2d;
	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix<double, 7, 1> Vector7d;

	class Robot {

	public:
//	    // Cartesian kinematic limits
//	    static constexpr double transVelocityLimit {1.7};	// m/s
//	    static constexpr double transAccelLimit {13.0};		// m/s^2
//	    static constexpr double transJerkLimit {6500.0};	// m/s^3
//	    static constexpr double rotVelocityLimit {2.5};		// rad/s
//	    static constexpr double rotAccelLimit {25.0};		// rad/s^2
//	    static constexpr double rotJerkLimit {12500.0};		// rad/s^3
//
//	    // Joint kinematic limits
//	    static constexpr std::array<double, 7> jointVelocityLimits {{2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}};		// rad/s
//	    static constexpr std::array<double, 7> jointAccelLimits {{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0}};					// rad/s^2
//	    static constexpr std::array<double, 7> jointJerkLimits {{7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0}};	// rad/s^3
//
//	    // Other robot parameters
//	    static constexpr size_t degreesOfFreedom {7};
//	    static constexpr double controlRate {0.001};		// s

		explicit Robot(
				const std::string& ip,
				double maxTransVelocity = 1.0,
				double maxTransAccel = 5.0,
				double maxTransJerk = 20.0,
				double maxRotVelocity = 3.0,
				double maxRotAccel = 15.0,
				double maxRotJerk = 75.0,
				double maxJointVelocity = 3.0,
				double maxJointAccel = 15.0,
				double maxJointJerk = 75.0,
				double relVelocity = 0.1,
				double relAccel = 0.1,
				double relJerk = 0.1
				);

		double getMaxTransVelocity() { return maxTransVelocity_; }
		void setMaxTransVelocity(double val) { maxTransVelocity_ = val; }
		double getMaxTransAccel() { return maxTransAccel_; }
		void setMaxTransAccel(double val) { maxTransAccel_ = val; }
		double getMaxTransJerk() { return maxTransJerk_; }
		void setMaxTransJerk(double val) { maxTransJerk_ = val; }
		double getMaxRotVelocity() { return maxRotVelocity_; }
		void setMaxRotVelocity(double val) { maxRotVelocity_ = val; }
		double getMaxRotAccel() { return maxRotAccel_; }
		void setMaxRotAccel(double val) { maxRotAccel_ = val; }
		double getMaxRotJerk() { return maxRotJerk_; }
		void setMaxRotJerk(double val) { maxRotJerk_ = val; }
		double getMaxJointVelocity() { return maxJointVelocity_; }
		void setMaxJointVelocity(double val) { maxJointVelocity_ = val; }
		double getMaxJointAccel() { return maxJointAccel_; }
		void setMaxJointAccel(double val) { maxJointAccel_ = val; }
		double getMaxJointJerk() { return maxJointJerk_; }
		void setMaxJointJerk(double val) { maxJointJerk_ = val; }
		double getRelVelocity() { return relVelocity_; }
		void setRelVelocity(double val) { relVelocity_ = val; }
		double getRelAccel() { return relAccel_; }
		void setRelAccel(double val) { relAccel_ = val; }
		double getRelJerk() { return relJerk_; }
		void setRelJerk(double val) { relJerk_ = val; }

	    void moveJointPosition(const Vector7d& targetJointPos);

	    void moveLinearPosition(const std::tuple<Vector3d, Vector4d>& targetPose);
	    void moveLinearPosition(const std::tuple<Vector3d, Vector4d>& targetPose, const Vector2d& elbow);

	    void moveJointVelocity(const Vector7d& targetJointVel);

	    void moveLinearVelocity(const Vector6d& targetLinearVel);
	    void moveLinearVelocity(const Vector6d& targetLinearVel, const Vector2d& elbow);

	    std::tuple<Vector3d, Vector4d> getCurrentPose();
	    std::tuple<Vector3d, Vector4d> getDesiredPose();
	    std::tuple<Vector3d, Vector4d> getCommandedPose();

	    Vector6d getDesiredLinearVelocity();
	    Vector6d getCommandedLinearVelocity();

	    Vector7d getCurrentJointVelocity();
	    Vector7d getDesiredJointVelocity();
	    Vector7d getCommandedJointVelocity();

	    Vector7d getCurrentJointPositions();
	    Vector7d getDesiredJointPositions();
	    Vector7d getCommandedJointPositions();

	    Vector2d getCurrentElbow();
	    Vector2d getDesiredElbow();
	    Vector2d getCommandedElbow();

	    std::tuple<Vector3d, Vector4d> getEndEffectorFrame();
	    std::tuple<Vector3d, Vector4d> getStiffnessFrame();
	    void setCollisionBehavior(
	    		const Vector7d& lowerTorqueThresholdsAccel,
	    		const Vector7d& upperTorqueThresholdsAccel,
				const Vector7d& lowerTorqueThresholdsNominal,
				const Vector7d& upperTorqueThresholdsNominal,
	    		const Vector6d& lowerForceThresholdsAccel,
	    		const Vector6d& upperForceThresholdsAccel,
				const Vector6d& lowerForceThresholdsNominal,
				const Vector6d& upperForceThresholdsNominal);
		void setJointImpedance(const Vector7d& kq);
		void setCartesianImpedance(const Vector6d& kx);
	    void setEndEffectorFrame(const std::tuple<Vector3d, Vector4d>& eeFrame);
	    void setStiffnessFrame(const std::tuple<Vector3d, Vector4d>& kFrame);
	    void setDefaultBehavior();

	    void stop();
	    bool hasErrors();
	    bool recoverFromErrors();
	    unsigned serverVersion();

	private:
	    void doMoveLinearPosition_(const std::tuple<Vector3d, Vector4d>& targetPose,
	    		const Vector2d& elbow = Vector2d(), bool elbowDefined = false);
	    void doMoveJointPosition_(const Vector7d& targetJointPos);
	    void doMoveLinearVelocity_(const Vector6d& targetLinearVel, const Vector2d& elbow = Vector2d(),
	    		bool elbowDefined = false);
	    void doMoveJointVelocity_(const Vector7d& targetJointVel);

	    void startVelocityControlThread_(void (Robot::*vcThreadFn)());
	    void stopVelocityControlThread_();

	    void doLinearVelocityMotion_();
	    void doJointVelocityMotion_();

	    void updateLinearVelocityParams_(
	    		const Vector6d& initLinearVel,
				const Vector6d& targetLinearVel,
				const Vector2d& elbow,
				bool elbowDefined,
				const SSABTrajectory& trajectory,
				double currTime
		);
	    void updateJointVelocityParams_(
	    		const Vector7d& initJointVel,
				const Vector7d& targetJointVel,
				const SSABTrajectory& trajectory,
				double currTime
		);

	    void checkVelocityControlNotRunning_();

	private:
	    franka::Robot robot_;
	    franka::RobotState robotState_;

		double maxTransVelocity_;
		double maxTransAccel_;
		double maxTransJerk_;
		double maxRotVelocity_;
		double maxRotAccel_;
		double maxRotJerk_;
		double maxJointVelocity_;
		double maxJointAccel_;
		double maxJointJerk_;
		double relVelocity_;
		double relAccel_;
		double relJerk_;

		double currTime_;

		// Elbow parameters
		Vector2d elbow_;
		bool elbowDefined_;

		// Position control trajectory
		LSSABTrajectory posTraj_;

		// Cartesian position control parameters
		Vector3d initTrans_;
		Vector3d targetTrans_;
		Quaterniond initRot_;
		Quaterniond targetRot_;

		// Joint position control parameters
		Vector7d initJointPos_;
		Vector7d targetJointPos_;

		// Velocity control thread
		std::thread vcThread_;
		std::mutex vcMutex_;
		bool vcThreadRunning_;
		void (Robot::*vcThreadFn_)();

		// Velocity control trajectory
		SSABTrajectory velTraj_;

		// Cartesian velocity control parameters
		Vector6d initLinearVel_;
		Vector6d targetLinearVel_;

		// Joint velocity control parameters
		Vector7d initJointVel_;
		Vector7d targetJointVel_;
	};

}	// namespace franka_control


#endif	// FRANKA_CONTROL_ROBOT_H_

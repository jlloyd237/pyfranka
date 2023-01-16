#include <tuple>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>

#include "robot.h"
#include "utils.h"
#include "lssab_trajectory.h"
#include "ssab_trajectory.h"

namespace franka_control {

	using namespace Eigen;

	Robot::Robot(
			const std::string& ip,
			double maxTransVelocity,
			double maxTransAccel,
			double maxTransJerk,
			double maxRotVelocity,
			double maxRotAccel,
			double maxRotJerk,
			double maxJointVelocity,
			double maxJointAccel,
			double maxJointJerk,
			double relVelocity,
			double relAccel,
			double relJerk
			)
	: robot_(ip)
	, maxTransVelocity_(maxTransVelocity)
	, maxTransAccel_(maxTransAccel)
	, maxTransJerk_(maxTransJerk)
	, maxRotVelocity_(maxRotVelocity)
	, maxRotAccel_(maxRotAccel)
	, maxRotJerk_(maxRotJerk)
	, maxJointVelocity_(maxJointVelocity)
	, maxJointAccel_(maxJointAccel)
	, maxJointJerk_(maxJointJerk)
	, relVelocity_(relVelocity)
	, relAccel_(relAccel)
	, relJerk_(relJerk)
	, vcThreadRunning_(false)
	{
		setDefaultBehavior();
	}

    void Robot::moveJointPosition(const Vector7d& targetJointPos) {
    	checkVelocityControlNotRunning_();
    	doMoveJointPosition_(targetJointPos);
    }

    void Robot::moveLinearPosition(const std::tuple<Vector3d, Vector4d>& targetPose) {
    	checkVelocityControlNotRunning_();
    	doMoveLinearPosition_(targetPose);
    }

    void Robot::moveLinearPosition(const std::tuple<Vector3d, Vector4d>& targetPose, const Vector2d& elbow) {
    	checkVelocityControlNotRunning_();
    	doMoveLinearPosition_(targetPose, elbow, true);
    }

	void Robot::moveJointVelocity(const Vector7d& targetJointVel) {
		doMoveJointVelocity_(targetJointVel);
	}

	void Robot::moveLinearVelocity(const Vector6d& targetLinearVel) {
		doMoveLinearVelocity_(targetLinearVel);
	}

	void Robot::moveLinearVelocity(const Vector6d& targetLinearVel, const Vector2d& elbow) {
		doMoveLinearVelocity_(targetLinearVel, elbow, true);
	}

    std::tuple<Vector3d, Vector4d> Robot::getCurrentPose() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE)));
    }

    std::tuple<Vector3d, Vector4d> Robot::getDesiredPose() {
//    	checkVelocityControlNotRunning_();
    	auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE_d)));
    }

    std::tuple<Vector3d, Vector4d> Robot::getCommandedPose() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE_c)));
    }

    Vector2d Robot::getCurrentElbow() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector2d(state.elbow.data());
    }

    Vector2d Robot::getDesiredElbow() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector2d(state.elbow_d.data());
    }

    Vector2d Robot::getCommandedElbow() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector2d(state.elbow_c.data());
    }

    Vector7d Robot::getCurrentJointPositions() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector7d(state.q.data());
    }

    Vector7d Robot::getDesiredJointPositions() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector7d(state.q_d.data());
    }

    Vector7d Robot::getCommandedJointPositions() {
        return getDesiredJointPositions();		// Same as desired joint angles
    }

    Vector6d Robot::getDesiredLinearVelocity() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector6d(state.O_dP_EE_d.data());
    }

    Vector6d Robot::getCommandedLinearVelocity() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector6d(state.O_dP_EE_c.data());
    }

    Vector7d Robot::getCurrentJointVelocity() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector7d(state.dq.data());
    }

    Vector7d Robot::getDesiredJointVelocity() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
        return Vector7d(state.dq_d.data());
    }

    Vector7d Robot::getCommandedJointVelocity() {
        return getDesiredJointVelocity();
    }

    void Robot::setEndEffectorFrame(const std::tuple<Vector3d, Vector4d>& eeFrame) {
    	checkVelocityControlNotRunning_();
    	robot_.setEE(mat2arr(quat2mat(qcoeff2quat(eeFrame))));
    }

    std::tuple<Vector3d, Vector4d> Robot::getEndEffectorFrame() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
    	return quat2qcoeff(mat2quat(arr2mat(state.NE_T_EE)));
    }

	void Robot::setStiffnessFrame(const std::tuple<Vector3d, Vector4d>& kFrame) {
    	checkVelocityControlNotRunning_();
    	robot_.setEE(mat2arr(quat2mat(qcoeff2quat(kFrame))));
    }

    std::tuple<Vector3d, Vector4d> Robot::getStiffnessFrame() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
    	return quat2qcoeff(mat2quat(arr2mat(state.EE_T_K)));
    }

    void Robot::setCollisionBehavior(
    		const Vector7d& lowerTorqueThresholdsAccel,
    		const Vector7d& upperTorqueThresholdsAccel,
			const Vector7d& lowerTorqueThresholdsNominal,
			const Vector7d& upperTorqueThresholdsNominal,
    		const Vector6d& lowerForceThresholdsAccel,
    		const Vector6d& upperForceThresholdsAccel,
			const Vector6d& lowerForceThresholdsNominal,
			const Vector6d& upperForceThresholdsNominal) {

    	checkVelocityControlNotRunning_();
    	robot_.setCollisionBehavior(
    			vec2arr(lowerTorqueThresholdsAccel),
				vec2arr(upperTorqueThresholdsAccel),
				vec2arr(lowerTorqueThresholdsNominal),
				vec2arr(upperTorqueThresholdsNominal),
    			vec2arr(lowerForceThresholdsAccel),
				vec2arr(upperForceThresholdsAccel),
				vec2arr(lowerForceThresholdsNominal),
				vec2arr(upperForceThresholdsNominal));
    }

	void Robot::setDefaultBehavior() {
    	checkVelocityControlNotRunning_();
	    robot_.setCollisionBehavior(
	        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
	        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
	        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
	        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
	        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
	        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
	        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
	        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}}
	    );

//	    robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
//	    robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

	    // libfranka 0.8 split F_T_EE into F_T_NE (set in desk to value below)
	    // and NE_T_EE which defaults to identity - set it again anyway.
//	    Matrix4d init_ee_frame;
//	    init_ee_frame << 1,  0,  0,  0,
//	    				 0,  1,  0,  0,
//						 0,  0,  1,  0,
//						 0,  0,  0,  1;
//	    init_ee_frame << 1,  0,  0,  0,
//	    				 0, -1,  0,  0,
//						 0,  0, -1,  0,
//						 0,  0,  0,  1;
//	    robot_.setEE(mat2arr(init_ee_frame));
	}

	void Robot::setJointImpedance(const Vector7d& kq) {
    	checkVelocityControlNotRunning_();
		robot_.setJointImpedance(vec2arr(kq));
	}

	void Robot::setCartesianImpedance(const Vector6d& kx) {
    	checkVelocityControlNotRunning_();
		robot_.setCartesianImpedance(vec2arr(kx));
	}

    void Robot::stop() {
    	checkVelocityControlNotRunning_();
    	robot_.stop();
    }

	bool Robot::hasErrors() {
//    	checkVelocityControlNotRunning_();
		auto state = vcThreadRunning_ ? robotState_ : robot_.readOnce();
		return bool(state.current_errors);
	}

	bool Robot::recoverFromErrors() {
    	checkVelocityControlNotRunning_();
		robot_.automaticErrorRecovery();
	    return !hasErrors();
	}

	unsigned Robot::serverVersion() {
    	checkVelocityControlNotRunning_();
		return robot_.serverVersion();
	}

    void Robot::doMoveJointPosition_(const Vector7d& targetJointPos) {
//		if (vcThreadRunning_) {
//			throw std::runtime_error("Error: joint position control not possible while "
//				"velocity control is running");
//		}

		targetJointPos_ = targetJointPos;

    	// Use previous desired joints as initial joints
    	initJointPos_ = getDesiredJointPositions();

    	// Compute the distance between initial and target joint angles
    	double jointDist = (targetJointPos_ - initJointPos_).norm();

		posTraj_ = LSSABTrajectory(0, jointDist,
			maxJointVelocity_ * relVelocity_,
			maxJointAccel_ * relAccel_,
			maxJointJerk_ * relJerk_,
			true);

		// If no move to perform then we're done
		if (posTraj_.duration() == 0) return;

		currTime_ = 0.0;

		// Specify motion generator callback function
		robot_.control([this](const franka::RobotState& robotState,
				franka::Duration period) -> franka::JointPositions {
			// Update current time
			currTime_ += period.toSec();
			robotState_ = robotState;

			// Compute trajectory value for the current time
			double trajVal = posTraj_(currTime_);

			// Interpolate between initial and final joint angles
			Vector7d currJoints = lerp(trajVal, initJointPos_, targetJointPos_);

			if (currTime_ >= posTraj_.duration()) {
				return franka::MotionFinished(franka::JointPositions(vec2arr(currJoints)));
			} else {
				return franka::JointPositions(vec2arr(currJoints));
			}
		}, franka::ControllerMode::kJointImpedance, true, 100.0);
    }

    void Robot::doMoveLinearPosition_(const std::tuple<Vector3d, Vector4d>& targetPose, const Vector2d& elbow, bool elbowDefined)
    {
//		if (vcThreadRunning_) {
//			throw std::runtime_error("Error: Cartesian position control not possible while "
//				"velocity control is running");
//		}

    	std::tuple<Vector3d, Quaterniond> targetPoseQuat = qcoeff2quat(targetPose);

    	// Split out translation and rotation (quaternion) components
    	targetTrans_ = std::get<0>(targetPoseQuat);
    	targetRot_ = std::get<1>(targetPoseQuat);

    	// Use previous desired pose as initial pose
    	std::tuple<Vector3d, Quaterniond> initPose = qcoeff2quat(getDesiredPose());

    	initTrans_ = std::get<0>(initPose);
    	initRot_ = std::get<1>(initPose);

    	// Compute translation distance between poses
		double transDist = (targetTrans_ - initTrans_).norm();

		// Compute rotation distance between poses
		double rotDist = targetRot_.angularDistance(initRot_);

		// Use translation or rotation trajectory with the longest duration
		LSSABTrajectory transTraj = LSSABTrajectory(0, transDist,
			maxTransVelocity_ * relVelocity_,
			maxTransAccel_ * relAccel_,
			maxTransJerk_ * relJerk_,
			true);
		LSSABTrajectory rotTraj = LSSABTrajectory(0, rotDist,
			maxRotVelocity_ * relVelocity_,
			maxRotAccel_ * relAccel_,
			maxRotJerk_ * relJerk_,
			true);
		posTraj_ = (transTraj.duration() > rotTraj.duration()) ? transTraj : rotTraj;

		// If no move to perform then we're done
		if (posTraj_.duration() == 0) return;

		elbow_ = elbow;
		elbowDefined_ = elbowDefined;
		currTime_ = 0.0;

		// Specify motion generator callback function
		robot_.control([this](const franka::RobotState& robotState,
				franka::Duration period) -> franka::CartesianPose {
			// Update current time
			currTime_ += period.toSec();
			robotState_ = robotState;

			// Interpolate between initial and final poses
			double trajVal = posTraj_(currTime_);
			Vector3d currPos = lerp(trajVal, initTrans_, targetTrans_);
			Quaterniond currRot = initRot_.slerp(trajVal, targetRot_);

			// Convert interpolated pose to column-major vector representation of 4x4 homogeneous matrix
			std::array<double, 16> currPose = mat2arr(quat2mat(std::make_tuple(currPos, currRot)));

			// Returned interpolated pose depends on whether elbow has been specified
			if (currTime_ >= posTraj_.duration()) {
				if (elbowDefined_) {
					return franka::MotionFinished(franka::CartesianPose(currPose, vec2arr(elbow_)));
				} else {
					return franka::MotionFinished(franka::CartesianPose(currPose));
				}
			} else {
				if (elbowDefined_) {
					return franka::CartesianPose(currPose, vec2arr(elbow_));
				} else {
					return franka::CartesianPose(currPose);
				}
			}
		}, franka::ControllerMode::kJointImpedance, true, 100.0);
    }

	void Robot::startVelocityControlThread_(void (Robot::*vcThreadFn)()) {
		if (vcThreadRunning_) {
			throw std::runtime_error("Error: cannot start velocity control thread - already running");
		}
		vcThread_ = std::thread([this, vcThreadFn] { (this->*vcThreadFn)(); });
		vcThreadFn_ = vcThreadFn;
		vcThreadRunning_ = true;
	}

	void Robot::stopVelocityControlThread_() {
		if (!vcThreadRunning_) {
			throw std::runtime_error("Error: cannot stop velocity control thread - not running");
		}
		vcThreadRunning_ = false;
		vcThread_.join();
	}

	void Robot::doLinearVelocityMotion_() {
		robot_.control([this](const franka::RobotState& robotState, franka::Duration period) -> franka::CartesianVelocities {

			Vector6d currVel;
			{
				std::lock_guard<std::mutex> guard(vcMutex_);
				currTime_ += period.toSec();
				robotState_ = robotState;
				currVel = lerp(velTraj_(currTime_), initLinearVel_, targetLinearVel_);
			}

			if (!vcThreadRunning_) {
				if (elbowDefined_){
					return franka::MotionFinished(franka::CartesianVelocities(vec2arr(currVel), vec2arr(elbow_)));
				} else {
					return franka::MotionFinished(franka::CartesianVelocities(vec2arr(currVel)));
				}
			}
			if (elbowDefined_) {
				return franka::CartesianVelocities(vec2arr(currVel), vec2arr(elbow_));
			} else {
				return franka::CartesianVelocities(vec2arr(currVel));
			}
		}, franka::ControllerMode::kJointImpedance, true, 100.0);
	}

	void Robot::doJointVelocityMotion_() {
		robot_.control([this](const franka::RobotState& robotState, franka::Duration period) -> franka::JointVelocities {

			Vector7d currJointVel;
			{
				std::lock_guard<std::mutex> guard(vcMutex_);
				currTime_ += period.toSec();
				robotState_ = robotState;
				currJointVel = lerp(velTraj_(currTime_), initJointVel_, targetJointVel_);
			}

			if (!vcThreadRunning_) {
				return franka::MotionFinished(franka::JointVelocities(vec2arr(currJointVel)));
			}
			return franka::JointVelocities(vec2arr(currJointVel));

		}, franka::ControllerMode::kJointImpedance, true, 100.0);
	}

	void Robot::doMoveLinearVelocity_(const Vector6d& targetLinearVel, const Vector2d& elbow, bool elbowDefined) {

		if (vcThreadRunning_ && vcThreadFn_ == &Robot::doJointVelocityMotion_) {
			throw std::runtime_error("Error: Cartesian velocity control not possible while "
				"joint velocity control is running");
		}

    	// Use previous desired velocity as initial velocity
    	Vector6d initLinearVel = vcThreadRunning_ ? targetLinearVel_ : Vector6d::Zero();

    	Vector3d initTransVel = initLinearVel.head(3);
    	Vector3d initRotVel = initLinearVel.tail(3);

    	Vector3d targetTransVel = targetLinearVel.head(3);
    	Vector3d targetRotVel = targetLinearVel.tail(3);

    	// Compute relative distance between translation velocities
		double transDist = (targetTransVel - initTransVel).norm();

    	// Compute relative distance between rotational velocities
		double rotDist = (targetRotVel - initRotVel).norm();

		// Use translation or rotation trajectory with the longest duration
		SSABTrajectory transTraj = SSABTrajectory(0, transDist,
			maxTransAccel_ * relAccel_,
			maxTransJerk_ * relJerk_,
			true);
		SSABTrajectory rotTraj = SSABTrajectory(0, rotDist,
			maxRotAccel_ * relAccel_,
			maxRotJerk_ * relJerk_,
			true);
		SSABTrajectory& newTraj = (transTraj.duration() > rotTraj.duration()) ? transTraj : rotTraj;

		// If no move to perform then we're done
		if (newTraj.duration() == 0) return;

		// If velocity control thread already running ...
		if (vcThreadRunning_) {
			// Wait until current trajectory finished
			while (currTime_ < velTraj_.duration()) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));
			}

			// Update/initialise velocity control parameters
			updateLinearVelocityParams_(targetLinearVel_, targetLinearVel, elbow, elbowDefined, newTraj, 0.0);
		}
		else {
			// Update/initialise velocity control parameters
			updateLinearVelocityParams_(Vector6d::Zero(), targetLinearVel, elbow, elbowDefined, newTraj, 0.0);

			// Start velocity control thread
			startVelocityControlThread_(&Robot::doLinearVelocityMotion_);
		}

		// If target velocity is zero
		if (vcThreadRunning_ && targetLinearVel_.isZero()) {
			// Wait until current trajectory finished
			while (currTime_ < velTraj_.duration()) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));
			}

			// Stop velocity control thread
			stopVelocityControlThread_();
		}
	}

	void Robot::doMoveJointVelocity_(const Vector7d& targetJointVel) {

		if (vcThreadRunning_ && vcThreadFn_ == &Robot::doLinearVelocityMotion_) {
			throw std::runtime_error("Error: joint velocity control not possible while "
				"Cartesian velocity control is running");
		}

    	// Use previous desired velocity as initial velocity
    	Vector7d initJointVel = vcThreadRunning_ ? targetJointVel_ : Vector7d::Zero();

    	// Compute relative distance between joint velocities
    	double jointDist = (targetJointVel - initJointVel).norm();

		SSABTrajectory newTraj = SSABTrajectory(0, jointDist,
			maxJointAccel_ * relAccel_,
			maxJointJerk_ * relJerk_,
			true);

		// If no move to perform then we're done
		if (newTraj.duration() == 0) return;

		// If velocity control thread already running ...
		if (vcThreadRunning_) {
			// Wait until current trajectory finished
			while (currTime_ < velTraj_.duration()) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));
			}

			// Update/initialise velocity control parameters
			updateJointVelocityParams_(targetJointVel_, targetJointVel, newTraj, 0.0);
		}
		else {
			// Update/initialise velocity control parameters
			updateJointVelocityParams_(Vector7d::Zero(), targetJointVel, newTraj, 0.0);

			// Start velocity control thread
			startVelocityControlThread_(&Robot::doJointVelocityMotion_);
		}

		// If target velocity is zero
		if (vcThreadRunning_ && targetJointVel_.isZero()) {
			// Wait until trajectory finished
			while (currTime_ < velTraj_.duration()) {
				std::this_thread::sleep_for(std::chrono::microseconds(100));
			}

			// Stop velocity control thread
			stopVelocityControlThread_();
		}
	}

    void Robot::updateLinearVelocityParams_(
    		const Vector6d& initLinearVel,
			const Vector6d& targetLinearVel,
			const Vector2d& elbow,
			bool elbowDefined,
			const SSABTrajectory& trajectory,
			double currTime
			)
    {
		std::lock_guard<std::mutex> guard(vcMutex_);

		initLinearVel_ = initLinearVel;
		targetLinearVel_ = targetLinearVel;
		elbow_ = elbow;
		elbowDefined_ = elbowDefined;
		velTraj_ = trajectory;
		currTime_ = currTime;
    }

	void Robot::updateJointVelocityParams_(
			const Vector7d& initJointVel,
			const Vector7d& targetJointVel,
			const SSABTrajectory& trajectory,
			double currTime
			)
	{
		std::lock_guard<std::mutex> guard(vcMutex_);

		initJointVel_ = initJointVel;
		targetJointVel_ = targetJointVel;
		velTraj_ = trajectory;
		currTime_ = currTime;
    }

	void Robot::checkVelocityControlNotRunning_() {
		if (vcThreadRunning_) {
			throw std::runtime_error("Error: operation not possible while "
				"velocity control is running");
		}
	}

}	// namespace franka_control





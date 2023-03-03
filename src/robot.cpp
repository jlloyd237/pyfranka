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
	, bgThreadRunning_(false)
	, bgThreadFunc_(nullptr)
	{
		setDefaultBehavior();
		robotState_ = robot_.readOnce();
	}

    void Robot::moveJointPosition(const Vector7d& targetJointPos) {
    	doMoveJointPosition_(targetJointPos);
    }

    void Robot::moveLinearPosition(const std::tuple<Vector3d, Vector4d>& targetPose) {
    	doMoveLinearPosition_(targetPose);
    }

    void Robot::moveLinearPosition(const std::tuple<Vector3d, Vector4d>& targetPose, const Vector2d& elbow) {
    	doMoveLinearPosition_(targetPose, elbow, true);
    }

	void Robot::moveJointVelocity(const Vector7d& targetJointVel) {
		if ((bgThreadFunc_ != nullptr) && (bgThreadFunc_ != &Robot::doJointVelocityMotion_)) {
			throw std::runtime_error("Error: Joint velocity control not possible while "
				"another type of robot control function is running");
		}
		doMoveJointVelocity_(targetJointVel);
	}

	void Robot::moveLinearVelocity(const Vector6d& targetLinearVel) {
		if ((bgThreadFunc_ != nullptr) && (bgThreadFunc_ != &Robot::doLinearVelocityMotion_)) {
			throw std::runtime_error("Error: Linear velocity control not possible while "
				"another type of robot control function is running");
		}
		doMoveLinearVelocity_(targetLinearVel);
	}

	void Robot::moveLinearVelocity(const Vector6d& targetLinearVel, const Vector2d& elbow) {
		if ((bgThreadFunc_ != nullptr) && (bgThreadFunc_ != &Robot::doLinearVelocityMotion_)) {
			throw std::runtime_error("Error: Linear velocity control not possible while "
				"another type of robot control function is running");
		}
		doMoveLinearVelocity_(targetLinearVel, elbow, true);
	}

    Vector7d Robot::getCurrentJointPosition() {
    	auto state = getRobotState();
        return Vector7d(state.q.data());
    }

    Vector7d Robot::getDesiredJointPosition() {
    	auto state = getRobotState();
        return Vector7d(state.q_d.data());
    }

    Vector7d Robot::getCommandedJointPosition() {
        return getDesiredJointPosition();		// Same as desired joint angles
    }

    std::tuple<Vector3d, Vector4d> Robot::getCurrentPose() {
    	auto state = getRobotState();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE)));
    }

    std::tuple<Vector3d, Vector4d> Robot::getDesiredPose() {
    	auto state = getRobotState();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE_d)));
    }

    std::tuple<Vector3d, Vector4d> Robot::getCommandedPose() {
    	auto state = getRobotState();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE_c)));
    }

    Vector7d Robot::getCurrentJointVelocity() {
    	auto state = getRobotState();
        return Vector7d(state.dq.data());
    }

    Vector7d Robot::getDesiredJointVelocity() {
    	auto state = getRobotState();
        return Vector7d(state.dq_d.data());
    }

    Vector7d Robot::getCommandedJointVelocity() {
        return getDesiredJointVelocity();
    }

    Vector6d Robot::getDesiredLinearVelocity() {
    	auto state = getRobotState();
        return Vector6d(state.O_dP_EE_d.data());
    }

    Vector6d Robot::getCommandedLinearVelocity() {
    	auto state = getRobotState();
        return Vector6d(state.O_dP_EE_c.data());
    }

    Vector2d Robot::getCurrentElbow() {
    	auto state = getRobotState();
        return Vector2d(state.elbow.data());
    }

    Vector2d Robot::getDesiredElbow() {
    	auto state = getRobotState();
        return Vector2d(state.elbow_d.data());
    }

    Vector2d Robot::getCommandedElbow() {
    	auto state = getRobotState();
        return Vector2d(state.elbow_c.data());
    }

    std::tuple<Vector3d, Vector4d> Robot::getEndEffectorFrame() {
    	auto state = getRobotState();
    	return quat2qcoeff(mat2quat(arr2mat(state.NE_T_EE)));
    }

    void Robot::setEndEffectorFrame(const std::tuple<Vector3d, Vector4d>& eeFrame) {
    	robot_.setEE(mat2arr(quat2mat(qcoeff2quat(eeFrame))));
    }

    std::tuple<Vector3d, Vector4d> Robot::getStiffnessFrame() {
    	auto state = getRobotState();
    	return quat2qcoeff(mat2quat(arr2mat(state.EE_T_K)));
    }

	void Robot::setStiffnessFrame(const std::tuple<Vector3d, Vector4d>& kFrame) {
    	robot_.setEE(mat2arr(quat2mat(qcoeff2quat(kFrame))));
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

	void Robot::setJointImpedance(const Vector7d& kq) {
		robot_.setJointImpedance(vec2arr(kq));
	}

	void Robot::setCartesianImpedance(const Vector6d& kx) {
		robot_.setCartesianImpedance(vec2arr(kx));
	}

	void Robot::setDefaultBehavior() {
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

    void Robot::stop() {
    	robot_.stop();
    }

	bool Robot::hasErrors() {
    	auto state = getRobotState();
		return bool(state.current_errors);
	}

	bool Robot::recoverFromErrors() {
		robot_.automaticErrorRecovery();
	    return !hasErrors();
	}

	unsigned Robot::serverVersion() {
		return robot_.serverVersion();
	}

    void Robot::doMoveJointPosition_(const Vector7d& targetJointPos) {

    	// Use previous desired joint angles as initial joint angles
		initJointPos_ = getDesiredJointPosition();
		targetJointPos_ = targetJointPos;

		// Compute Euclidean distance between initial and target joint angles
		double jointDist = (targetJointPos_ - initJointPos_).norm();

		// Compute joint angle trajectory
		posTraj_ = LSSABTrajectory(0, jointDist,
			maxJointVelocity_ * relVelocity_,
			maxJointAccel_ * relAccel_,
			maxJointJerk_ * relJerk_,
			true);

		// If there is any trajectory to follow ...
		if (posTraj_.duration() > 0) {

			trajTime_ = 0.0;

			// Control robot using joint motion generator callback function
			robot_.control([this](const franka::RobotState& robotState,
					franka::Duration period) -> franka::JointPositions {

				// Synch robot state
				{
					std::lock_guard<std::mutex> guard(robotStateMutex_);
					robotState_ = franka::RobotState(robotState);
					trajTime_ += period.toSec();
				}

				// Interpolate between initial and final joint angles
				Vector7d currJoints = lerp(posTraj_(trajTime_), initJointPos_, targetJointPos_);

				// If we've reached the end of the trajectory, signal completion;
				// otherwise return the joint angles
				if (trajTime_ >= posTraj_.duration()) {
					return franka::MotionFinished(franka::JointPositions(vec2arr(currJoints)));
				} else {
					return franka::JointPositions(vec2arr(currJoints));
				}
			});
		}
    }

    void Robot::doMoveLinearPosition_(const std::tuple<Vector3d, Vector4d>& targetPose,
    		const Vector2d& elbow, bool elbowDefined) {

		// Use previous desired pose as initial pose; extract translation and
    	// rotation (quaternion) components
		std::tuple<Vector3d, Quaterniond> initPose = qcoeff2quat(getDesiredPose());
		initTrans_ = std::get<0>(initPose);
		initRot_ = std::get<1>(initPose);

		std::tuple<Vector3d, Quaterniond> targetPoseQuat = qcoeff2quat(targetPose);
		targetTrans_ = std::get<0>(targetPoseQuat);
		targetRot_ = std::get<1>(targetPoseQuat);

		// Compute Euclidean distance between translation components
		double transDist = (targetTrans_ - initTrans_).norm();

		// Compute angular distance between rotation components
		double rotDist = targetRot_.angularDistance(initRot_);

		// Compute translation and rotation trajectories; use trajectory
		// with the longest duration
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

		// If there is any trajectory to follow ...
		if (posTraj_.duration() > 0) {

			elbow_ = elbow;
			elbowDefined_ = elbowDefined;
			trajTime_ = 0.0;

			// Control robot using Cartesian position motion generator callback function
			robot_.control([this](const franka::RobotState& robotState,
					franka::Duration period) -> franka::CartesianPose {

				// Synch robot state
				{
					std::lock_guard<std::mutex> guard(robotStateMutex_);
					robotState_ = franka::RobotState(robotState);
					trajTime_ += period.toSec();
				}

				// Interpolate between initial and final translation and rotation
				double trajVal = posTraj_(trajTime_);
				Vector3d currPos = lerp(trajVal, initTrans_, targetTrans_);
				Quaterniond currRot = initRot_.slerp(trajVal, targetRot_);

				// Convert interpolated translation and rotation to column-major
				// vector representation of 4x4 homogeneous matrix
				std::array<double, 16> currPose = mat2arr(quat2mat(std::make_tuple(currPos, currRot)));

				// If we've reached the end of the trajectory, signal completion;
				// otherwise return the Cartesian pose (format depends on whether
				// elbow has been defined)
				if (trajTime_ >= posTraj_.duration()) {
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
			});
		}
    }

	void Robot::doMoveJointVelocity_(const Vector7d& targetJointVel) {

		// Use previous target joint velocity as initial joint velocity if velocity
		// control thread is already running; otherwise assume it is zero
		Vector7d initJointVel = bgThreadRunning_ ? targetJointVel_ : Vector7d::Zero();

		// Compute Euclidean distance between joint velocities
		double jointDist = (targetJointVel - initJointVel).norm();

		// Compute joint velocity trajectory
		SSABTrajectory newTraj = SSABTrajectory(0, jointDist,
			maxJointAccel_ * relAccel_,
			maxJointJerk_ * relJerk_,
			true);

		// If there is any trajectory to follow ...
		if (newTraj.duration() > 0) {

			// If background control thread is already running ...
			if (bgThreadRunning_) {
				// Wait until current trajectory has completed
				while (trajTime_ < velTraj_.duration()) {
					std::this_thread::sleep_for(std::chrono::microseconds(100));
				}

				// Update velocity control parameters
				{
					std::lock_guard<std::mutex> guard(robotStateMutex_);
					initJointVel_ = targetJointVel_;
					targetJointVel_ = targetJointVel;
					velTraj_ = newTraj;
					trajTime_ = 0.0;
				}
			}
			else {
				// Initialise velocity control parameters
				initJointVel_ = Vector7d::Zero();
				targetJointVel_ = targetJointVel;
				velTraj_ = newTraj;
				trajTime_ = 0.0;

				// Start background velocity control thread
				startBackgroundThread_(&Robot::doJointVelocityMotion_);
			}

			// If target joint velocity is zero
			if (bgThreadRunning_ && targetJointVel_.isZero()) {
				// Wait until trajectory has finished
				while (trajTime_ < velTraj_.duration()) {
					std::this_thread::sleep_for(std::chrono::microseconds(100));
				}

				// Stop background velocity control thread
				stopBackgroundThread_();
			}
		}
	}

	void Robot::doMoveLinearVelocity_(const Vector6d& targetLinearVel, const Vector2d& elbow, bool elbowDefined) {

		// Use previous target Cartesian velocity as initial Cartesian velocity if
		// velocity control thread is already running; otherwise assume it is zero
		Vector6d initLinearVel = bgThreadRunning_ ? targetLinearVel_ : Vector6d::Zero();

		// Extract translation and rotation velocity components
		Vector3d initTransVel = initLinearVel.head(3);
		Vector3d initRotVel = initLinearVel.tail(3);

		Vector3d targetTransVel = targetLinearVel.head(3);
		Vector3d targetRotVel = targetLinearVel.tail(3);

		// Compute Euclidean distance between translation velocities
		double transDist = (targetTransVel - initTransVel).norm();

		// Compute Euclidean distance between rotation velocities
		double rotDist = (targetRotVel - initRotVel).norm();

		// Compute translation and rotation velocity trajectories;
		// use trajectory with the longest duration
		SSABTrajectory transTraj = SSABTrajectory(0, transDist,
			maxTransAccel_ * relAccel_,
			maxTransJerk_ * relJerk_,
			true);
		SSABTrajectory rotTraj = SSABTrajectory(0, rotDist,
			maxRotAccel_ * relAccel_,
			maxRotJerk_ * relJerk_,
			true);
		SSABTrajectory& newTraj = (transTraj.duration() > rotTraj.duration()) ? transTraj : rotTraj;

		// If there is any trajectory to follow ...
		if (newTraj.duration() > 0) {

			// If background velocity control thread is already running ...
			if (bgThreadRunning_) {

				// Wait until current trajectory has completed
				while (trajTime_ < velTraj_.duration()) {
					std::this_thread::sleep_for(std::chrono::microseconds(100));
				}

				// Update velocity control parameters
				{
					std::lock_guard<std::mutex> guard(robotStateMutex_);
					initLinearVel_ = targetLinearVel_;
					targetLinearVel_ = targetLinearVel;
					elbow_ = elbow;
					elbowDefined_ = elbowDefined;
					velTraj_ = newTraj;
					trajTime_ = 0.0;
				}
			}
			else {
				// Initialise velocity control parameters
				initLinearVel_ = Vector6d::Zero();
				targetLinearVel_ = targetLinearVel;
				elbow_ = elbow;
				elbowDefined_ = elbowDefined;
				velTraj_ = newTraj;
				trajTime_ = 0.0;

				// Start background velocity control thread
				startBackgroundThread_(&Robot::doLinearVelocityMotion_);
			}

			// If target Cartesian velocity is zero ...
			if (bgThreadRunning_ && targetLinearVel_.isZero()) {
				// Wait until current velocity trajectory has finished
				while (trajTime_ < velTraj_.duration()) {
					std::this_thread::sleep_for(std::chrono::microseconds(100));
				}

				// Stop background control thread
				stopBackgroundThread_();
			}
		}
	}

	void Robot::doJointVelocityMotion_() {

		robot_.control([this](const franka::RobotState& robotState, franka::Duration period) -> franka::JointVelocities {

			// Synch robot state
			Vector7d initJointVel;
			Vector7d targetJointVel;
			SSABTrajectory velTraj;
			{
				std::lock_guard<std::mutex> guard(robotStateMutex_);
				initJointVel = initJointVel_;
				targetJointVel = targetJointVel_;
				velTraj = velTraj_;
				trajTime_ += period.toSec();
				robotState_ = franka::RobotState(robotState);
			}

			// Interpolate between initial and final joint velocities
			auto currJointVel = lerp(velTraj(trajTime_), initJointVel, targetJointVel);

			// If thread status flag has been reset, signal completion (and terminate thread);
			// otherwise return the joint velocities (format depends on whether
			// elbow has been defined)
			if (!bgThreadRunning_) {
				return franka::MotionFinished(franka::JointVelocities(vec2arr(currJointVel)));
			}
			return franka::JointVelocities(vec2arr(currJointVel));
		});
	}

	void Robot::doLinearVelocityMotion_() {

		robot_.control([this](const franka::RobotState& robotState, franka::Duration period) -> franka::CartesianVelocities {

			// Synch robot state
			Vector6d initLinearVel;
			Vector6d targetLinearVel;
			SSABTrajectory velTraj;
			Vector2d elbow;
			bool elbowDefined;
			{
				std::lock_guard<std::mutex> guard(robotStateMutex_);
				initLinearVel = initLinearVel_;
				targetLinearVel = targetLinearVel_;
				velTraj = velTraj_;
				elbow = elbow_;
				elbowDefined = elbowDefined_;
				trajTime_ += period.toSec();
				robotState_ = franka::RobotState(robotState);
			}

			// Interpolate between initial and final Cartesian velocities
			auto currVel = lerp(velTraj(trajTime_), initLinearVel, targetLinearVel);

			// If thread status flag has been reset, signal completion (and terminate thread);
			// otherwise return the Cartesian velocity (format depends on whether elbow has
			// been defined)
			if (!bgThreadRunning_) {
				if (elbowDefined){
					return franka::MotionFinished(franka::CartesianVelocities(vec2arr(currVel), vec2arr(elbow)));
				} else {
					return franka::MotionFinished(franka::CartesianVelocities(vec2arr(currVel)));
				}
			}
			if (elbowDefined) {
				return franka::CartesianVelocities(vec2arr(currVel), vec2arr(elbow));
			} else {
				return franka::CartesianVelocities(vec2arr(currVel));
			}
		});
	}

	franka::RobotState Robot::getRobotState() {
		// If a motion generator control function is running, use the state that is
		// saved by the control function (calling readOnce() in this state will throw
		// an exception

		try {
			return robot_.readOnce();
		} catch (franka::InvalidOperationException) {
			std::lock_guard<std::mutex> guard(robotStateMutex_);
			return franka::RobotState(robotState_);
		}
	}

	void Robot::startBackgroundThread_(void (Robot::*bgThreadFunc)()) {
		if (bgThreadRunning_) {
			throw std::runtime_error("Error: background thread already running");
		}
		bgThreadRunning_ = true;
		bgThread_ = std::thread([this, bgThreadFunc] { (this->*bgThreadFunc)(); });
		bgThreadFunc_ = bgThreadFunc;
	}

	void Robot::stopBackgroundThread_() {
		if (!bgThreadRunning_) {
			throw std::runtime_error("Error: background thread not running");
		}
		// Use thread status flag to terminate thread
		bgThreadRunning_ = false;
		bgThread_.join();
		bgThreadFunc_ = nullptr;
	}

}	// namespace franka_control







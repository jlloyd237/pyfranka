#include "robot.h"

#include <tuple>
#include <iostream>
#include <thread>

#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include "utils.h"
#include "lssab_trajectory.h"


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
	{
		setDefaultBehavior();
	}

    void Robot::doMoveLinear_(const std::tuple<Vector3d, Vector4d>& targetPose, const Vector2d* pElbow)
    {
    	std::tuple<Vector3d, Quaterniond> targetPoseQuat = qcoeff2quat(targetPose);

    	// Split out translation and rotation (quaternion) components
    	Vector3d targetTrans = std::get<0>(targetPoseQuat);
    	Quaterniond targetRot = std::get<1>(targetPoseQuat);

    	// Use previous target pose as initial pose
    	std::tuple<Vector3d, Quaterniond> initPose = qcoeff2quat(getTargetPose());

    	Vector3d initTrans = std::get<0>(initPose);
    	Quaterniond initRot = std::get<1>(initPose);

    	// Compute translation distance between poses
		double transDist = (targetTrans - initTrans).norm();

		// Compute rotation distance between poses
		if (initRot.coeffs().dot(targetRot.coeffs()) < 0.0) {
			targetRot.coeffs() << -targetRot.coeffs();
		}
		double rotDist = targetRot.angularDistance(initRot);

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
		LSSABTrajectory& traj = (transTraj.duration() > rotTraj.duration()) ? transTraj : rotTraj;

		// If no move to perform then we're done
		if (traj.duration() == 0) return;

		// Specify motion generator callback function
		double currTime = 0.0;
		robot_.control([=, &currTime, &traj](const franka::RobotState& robotState,
				franka::Duration period) -> franka::CartesianPose {
			// Update current time
			currTime += period.toSec();

			// Compute trajectory value for the current time
			double currTrajVal = traj(currTime);

			// Interpolate between initial and final poses
			Vector3d currPos = lerp(currTrajVal, initTrans, targetTrans);
			Quaterniond currRot = initRot.slerp(currTrajVal, targetRot);

			// Convert interpolated pose to column-major vector representation of 4x4 homogeneous matrix
			std::array<double, 16> currPose = mat2arr(quat2mat(std::make_tuple(currPos, currRot)));

			// Returned interpolated pose depends on whether elbow has been specified
			if (pElbow) {
				std::array<double, 2> currElbow;
				Map<Vector2d, Unaligned>(currElbow.data()) = *pElbow;
				franka::CartesianPose currFrankaPose(currPose, currElbow);
				if (currTime >= traj.duration()) {
					return franka::MotionFinished(currFrankaPose);
				}
				return currFrankaPose;
			} else {
				franka::CartesianPose currFrankaPose(currPose);
				if (currTime >= traj.duration()) {
					return franka::MotionFinished(currFrankaPose);
				}
				return currFrankaPose;
			}
		}, franka::ControllerMode::kJointImpedance, true, 100.0);
    }

    void Robot::moveLinear(const std::tuple<Vector3d, Vector4d>& targetPose) {
    	doMoveLinear_(targetPose, nullptr);

//    	std::thread worker(&Robot::doMoveLinear_, this, targetPose, nullptr);
//    	worker.join();
    }

    void Robot::moveLinear(const std::tuple<Vector3d, Vector4d>& targetPose, const Vector2d& elbow) {
    	doMoveLinear_(targetPose, &elbow);

//    	std::thread worker(&Robot::doMoveLinear_, this, targetPose, &elbow);
//    	worker.join();
    }

    void Robot::doMoveJoints_(const Vector7d& targetJoints) {
    	// Use previous target joints as initial joints
    	Vector7d initJoints = getTargetJoints();

    	// Compute the maximum distance between initial and target joint angles
    	double maxJointDist = (targetJoints - initJoints).cwiseAbs().maxCoeff();

		LSSABTrajectory traj = LSSABTrajectory(0, maxJointDist,
			maxJointVelocity_ * relVelocity_,
			maxJointAccel_ * relAccel_,
			maxJointJerk_ * relJerk_,
			true);

		// If no move to perform then we're done
		if (traj.duration() == 0) return;

		// Specify motion generator callback function
		double currTime = 0.0;
		robot_.control([=, &currTime, &traj](const franka::RobotState& robotState,
				franka::Duration period) -> franka::JointPositions {
			// Update current time
			currTime += period.toSec();

			// Compute trajectory value for the current time
			double currTrajVal = traj(currTime);

			// Interpolate between initial and final joint angles
			Vector7d currJointsVec = lerp(currTrajVal, initJoints, targetJoints);

			// Convert interpolated joint angles to std::array
			std::array<double, 7> currJoints;
			Map<Vector7d, Unaligned>(currJoints.data()) = currJointsVec;

			if (currTime >= traj.duration()) {
				return franka::MotionFinished(franka::JointPositions(currJoints));
			}

			return franka::JointPositions(currJoints);
		}, franka::ControllerMode::kJointImpedance, true, 100.0);
    }

    void Robot::moveJoints(const Vector7d& targetJoints) {
    	doMoveJoints_(targetJoints);

//     	std::thread worker(&Robot::doMoveJoints_, this, targetJoints);
//     	worker.join();
    }

    std::tuple<Vector3d, Vector4d> Robot::getCurrentPose() {
        auto state = robot_.readOnce();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE)));
    }

    Vector7d Robot::getCurrentJoints() {
        auto state = robot_.readOnce();
        return Vector7d(state.q.data());
    }

    Vector2d Robot::getCurrentElbow() {
        auto state = robot_.readOnce();
        return Vector2d(state.elbow.data());
    }

    std::tuple<Vector3d, Vector4d> Robot::getTargetPose() {
        auto state = robot_.readOnce();
        return quat2qcoeff(mat2quat(arr2mat(state.O_T_EE_d)));
    }

    Vector7d Robot::getTargetJoints() {
        auto state = robot_.readOnce();
        return Vector7d(state.q_d.data());
    }

    Vector2d Robot::getTargetElbow() {
        auto state = robot_.readOnce();
        return Vector2d(state.elbow_d.data());
    }

    void Robot::setEndEffectorFrame(const std::tuple<Vector3d, Vector4d>& eeFrame) {
    	robot_.setEE(mat2arr(quat2mat(qcoeff2quat(eeFrame))));
    }

    std::tuple<Vector3d, Vector4d> Robot::getEndEffectorFrame() {
    	auto state = robot_.readOnce();
    	return quat2qcoeff(mat2quat(arr2mat(state.NE_T_EE)));
    }

	void Robot::setStiffnessFrame(const std::tuple<Vector3d, Vector4d>& kFrame) {
    	robot_.setEE(mat2arr(quat2mat(qcoeff2quat(kFrame))));
    }

    std::tuple<Vector3d, Vector4d> Robot::getStiffnessFrame() {
    	auto state = robot_.readOnce();
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

		std::array<double, 7> lowerTorqueThresholdsAccelArr;
		Map<Vector7d, Unaligned>(lowerTorqueThresholdsAccelArr.data()) = lowerTorqueThresholdsAccel;
		std::array<double, 7> upperTorqueThresholdsAccelArr;
		Map<Vector7d, Unaligned>(upperTorqueThresholdsAccelArr.data()) = upperTorqueThresholdsAccel;
		std::array<double, 7> lowerTorqueThresholdsNominalArr;
		Map<Vector7d, Unaligned>(lowerTorqueThresholdsNominalArr.data()) = lowerTorqueThresholdsNominal;
		std::array<double, 7> upperTorqueThresholdsNominalArr;
		Map<Vector7d, Unaligned>(upperTorqueThresholdsNominalArr.data()) = upperTorqueThresholdsNominal;
		std::array<double, 6> lowerForceThresholdsAccelArr;
		Map<Vector6d, Unaligned>(lowerForceThresholdsAccelArr.data()) = lowerForceThresholdsAccel;
		std::array<double, 6> upperForceThresholdsAccelArr;
		Map<Vector6d, Unaligned>(upperForceThresholdsAccelArr.data()) = upperForceThresholdsAccel;
		std::array<double, 6> lowerForceThresholdsNominalArr;
		Map<Vector6d, Unaligned>(lowerForceThresholdsNominalArr.data()) = lowerForceThresholdsNominal;
		std::array<double, 6> upperForceThresholdsNominalArr;
		Map<Vector6d, Unaligned>(upperForceThresholdsNominalArr.data()) = upperForceThresholdsNominal;

    	robot_.setCollisionBehavior(
    			lowerTorqueThresholdsAccelArr,
				upperTorqueThresholdsAccelArr,
				lowerTorqueThresholdsNominalArr,
				upperTorqueThresholdsNominalArr,
    			lowerForceThresholdsAccelArr,
				upperForceThresholdsAccelArr,
				lowerForceThresholdsNominalArr,
				upperForceThresholdsNominalArr);
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

	void Robot::setJointImpedance(const Vector7d& kq) {
		std::array<double, 7> kqArr;
		Map<Vector7d, Unaligned>(kqArr.data()) = kq;
		robot_.setJointImpedance(kqArr);
	}

	void Robot::setCartesianImpedance(const Vector6d& kx) {
		std::array<double, 6> kxArr;
		Map<Vector6d, Unaligned>(kxArr.data()) = kx;
		robot_.setCartesianImpedance(kxArr);
	}

    void Robot::stop() {
    	robot_.stop();
    }

	bool Robot::hasErrors() {
		return bool(robot_.readOnce().current_errors);
	}

	bool Robot::recoverFromErrors() {
		robot_.automaticErrorRecovery();
	    return !hasErrors();
	}

	unsigned Robot::serverVersion() {
		return robot_.serverVersion();
	}

}	// namespace pyfranka





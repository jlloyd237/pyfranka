#include "utils.h"


namespace franka_control {

	Matrix4d arr2mat(const std::array<double, 16>& pose) {
		Matrix4d convPose = Map<const Matrix4d, ColMajor>(pose.data(), 4, 4);
		return convPose;
	}

	std::array<double, 16> mat2arr(const Matrix4d& pose) {
		std::array<double, 16> convPose;
		Map<Matrix4d, ColMajor>(convPose.data()) = pose;
		return convPose;
	}

	std::tuple<Vector3d, Quaterniond> mat2quat(const Matrix4d& pose) {
		Vector3d transVec = pose.block<3, 1>(0, 3);
		Matrix3d rotMat = pose.block<3, 3>(0, 0);
		Quaterniond rotQuat(rotMat);
		return std::make_tuple(transVec, rotQuat);
	}

	Matrix4d quat2mat(const std::tuple<Vector3d, Quaterniond>& pose) {
		Vector3d transVec = std::get<0>(pose);
		Quaterniond rotQuat = std::get<1>(pose);
		Matrix3d rotMat = rotQuat.toRotationMatrix();
		Matrix4d convPose;
		convPose.setIdentity();
		convPose.block<3, 3>(0, 0) = rotMat;
		convPose.block<3, 1>(0, 3) = transVec;
		return convPose;
	}

	std::tuple<Vector3d, Vector4d> quat2qcoeff(const std::tuple<Vector3d, Quaterniond>& pose) {
		Vector3d trans = std::get<0>(pose);
		Vector4d rot = std::get<1>(pose).coeffs();	// x, y, z, w
		Vector3d xyz = rot.head(3);
		double w = rot[3];
		rot[0] = w;
		rot.tail(3) = xyz;
		return std::make_tuple(trans, rot);
//		return std::make_tuple(std::get<0>(pose), std::get<1>(pose).coeffs());
	}

	std::tuple<Vector3d, Quaterniond> qcoeff2quat(const std::tuple<Vector3d, Vector4d>& pose) {
		Vector3d trans = std::get<0>(pose);
		Vector4d rot = std::get<1>(pose);		// w, x, y, z
		double w = rot[0];
		Vector3d xyz = rot.tail(3);
		rot.head(3) = xyz;
		rot[3] = w;
		return std::make_tuple(trans, Quaterniond(rot));
//		return std::make_tuple(std::get<0>(pose), Quaterniond(std::get<1>(pose)));
	}
}	// namespace franka_control

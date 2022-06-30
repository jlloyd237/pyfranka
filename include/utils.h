#ifndef FRANKA_CONTROL_UTILS_H_
#define FRANKA_CONTROL_UTILS_H_

#include <array>
#include <tuple>

#include <Eigen/Dense>


namespace franka_control {

	using namespace Eigen;

	typedef Matrix<double, 2, 1> Vector2d;
	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix<double, 7, 1> Vector7d;

	template <typename T> int sgn(T val) {
	    return (T(0) < val) - (val < T(0));
	}

	template <typename T> T lerp(double t, const T& start, const T& finish) {
		return (1 - t) * start + t * finish;
	}

	template <typename T, int K>
	std::array<T, K> vec2arr(const Matrix<T, K, 1>& vec) {
		std::array<T, K> arr;
		Map<Matrix<T, K, 1>, Unaligned>(arr.data()) = vec;
		return arr;
	}

	Matrix4d arr2mat(const std::array<double, 16>& pose);

	std::array<double, 16> mat2arr(const Matrix4d& pose);

	std::tuple<Vector3d, Quaterniond> mat2quat(const Matrix4d& pose);

	Matrix4d quat2mat(const std::tuple<Vector3d, Quaterniond>& pose);

	std::tuple<Vector3d, Vector4d> quat2qcoeff(const std::tuple<Vector3d, Quaterniond>& pose);

	std::tuple<Vector3d, Quaterniond> qcoeff2quat(const std::tuple<Vector3d, Vector4d>& pose);

}	// namespace franka_control


#endif	// FRANKA_CONTROL_UTILS_H_


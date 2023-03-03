#include <array>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <franka/exception.h>

#include "robot.h"
#include "gripper.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace franka_control;


PYBIND11_MODULE(_pyfranka, m) {
    m.doc() = "Python 3 control interface for Franka Panda robot";

    // Python Robot class
    py::class_<Robot>(m, "Robot")
        .def(py::init<const std::string &, double, double, double, double, double, double,
        		double, double, double, double, double, double>(),
        			"ip"_a,
					"max_trans_velocity"_a = 1.0,
					"max_trans_accel"_a = 5.0,
					"max_trans_jerk"_a = 20.0,
					"max_rot_velocity"_a = 3.0,
					"max_rot_accel"_a = 15.0,
					"max_rot_jerk"_a = 75.0,
					"max_joint_velocity"_a = 3.0,
					"max_joint_accel"_a = 15.0,
					"max_joint_jerk"_a = 75.0,
					"rel_velocity"_a = 0.1,
					"rel_accel"_a = 0.1,
					"rel_jerk"_a = 0.1)

//		.def_property_readonly_static("trans_velocity_limit", [](py::object) { return Robot::transVelocityLimit; })
//		.def_property_readonly_static("trans_accel_limit", [](py::object) { return Robot::transAccelLimit; })
//		.def_property_readonly_static("trans_jerk_limit", [](py::object) { return Robot::transJerkLimit; })
//		.def_property_readonly_static("rot_velocity_limit", [](py::object) { return Robot::rotVelocityLimit; })
//		.def_property_readonly_static("rot_accel_limit", [](py::object) { return Robot::rotAccelLimit; })
//		.def_property_readonly_static("rot_jerk_limit", [](py::object) { return Robot::rotJerkLimit; })
//		.def_property_readonly_static("joint_velocity_limits", [](py::object) { return Robot::jointVelocityLimits; })
//		.def_property_readonly_static("joint_accel_limits", [](py::object) { return Robot::jointAccelLimits; })
//		.def_property_readonly_static("joint_jerk_limits", [](py::object) { return Robot::jointJerkLimits; })
//		.def_property_readonly_static("degrees_of_freedom", [](py::object) { return Robot::degreesOfFreedom; })
//		.def_property_readonly_static("control_rate", [](py::object) { return Robot::controlRate; })

		.def_property("max_trans_velocity", &Robot::getMaxTransVelocity, &Robot::setMaxTransVelocity)
		.def_property("max_trans_accel", &Robot::getMaxTransAccel, &Robot::setMaxTransAccel)
		.def_property("max_trans_jerk", &Robot::getMaxTransJerk, &Robot::setMaxTransJerk)
		.def_property("max_rot_velocity", &Robot::getMaxRotVelocity, &Robot::setMaxRotVelocity)
		.def_property("max_rot_accel", &Robot::getMaxRotAccel, &Robot::setMaxRotAccel)
		.def_property("max_rot_jerk", &Robot::getMaxRotJerk, &Robot::setMaxRotJerk)
		.def_property("max_joint_velocity", &Robot::getMaxJointVelocity, &Robot::setMaxJointVelocity)
		.def_property("max_joint_accel", &Robot::getMaxJointAccel, &Robot::setMaxJointAccel)
		.def_property("max_joint_jerk", &Robot::getMaxJointJerk, &Robot::setMaxJointJerk)
		.def_property("rel_velocity", &Robot::getRelVelocity, &Robot::setRelVelocity)
		.def_property("rel_accel", &Robot::getRelAccel, &Robot::setRelAccel)
		.def_property("rel_jerk", &Robot::getRelJerk, &Robot::setRelJerk)

		.def_property_readonly("server_version", &Robot::serverVersion)
		.def_property_readonly("has_errors", &Robot::hasErrors)

		.def_property_readonly("current_joints", &Robot::getCurrentJointPosition)
		.def_property_readonly("desired_joints", &Robot::getDesiredJointPosition)
		.def_property_readonly("commanded_joints", &Robot::getCommandedJointPosition)
		.def_property_readonly("current_pose", &Robot::getCurrentPose)
		.def_property_readonly("desired_pose", &Robot::getDesiredPose)
		.def_property_readonly("commanded_pose", &Robot::getCommandedPose)
		.def_property_readonly("current_elbow", &Robot::getCurrentElbow)
		.def_property_readonly("desired_elbow", &Robot::getDesiredElbow)
		.def_property_readonly("commanded_elbow", &Robot::getCommandedElbow)

		.def_property_readonly("current_joints_velocity", &Robot::getCurrentJointVelocity)
		.def_property_readonly("desired_joints_velocity", &Robot::getDesiredJointVelocity)
		.def_property_readonly("commanded_joints_velocity", &Robot::getCommandedJointVelocity)
		.def_property_readonly("desired_linear_velocity", &Robot::getDesiredLinearVelocity)
		.def_property_readonly("commanded_linear_velocity", &Robot::getCommandedLinearVelocity)

		.def_property("ee_frame", &Robot::getEndEffectorFrame, &Robot::setEndEffectorFrame)
		.def_property("stiffness_frame", &Robot::getStiffnessFrame, &Robot::setStiffnessFrame)

		.def("move_joints", &Robot::moveJointPosition, py::call_guard<py::gil_scoped_release>(), "target_joint_pos"_a)
        .def("move_linear", (void (Robot::*)(const std::tuple<Vector3d, Vector4d>&)) &Robot::moveLinearPosition,
        		py::call_guard<py::gil_scoped_release>(),"target_pose"_a)
        .def("move_linear", (void (Robot::*)(const std::tuple<Vector3d, Vector4d>&, const Vector2d&)) &Robot::moveLinearPosition,
        		py::call_guard<py::gil_scoped_release>(), "target_pose"_a, "elbow"_a)

		.def("move_joints_velocity", &Robot::moveJointVelocity, py::call_guard<py::gil_scoped_release>(),
				"target_joint_vel"_a)
		.def("move_linear_velocity", (void (Robot::*)(const Vector6d&)) &Robot::moveLinearVelocity,
				py::call_guard<py::gil_scoped_release>(), "target_linear_vel"_a)
		.def("move_linear_velocity", (void (Robot::*)(const Vector6d&, const Vector2d&)) &Robot::moveLinearVelocity,
				py::call_guard<py::gil_scoped_release>(), "target_linear_vel"_a, "elbow"_a)

		.def("set_collision_behavior", &Robot::setCollisionBehavior,
				"lower_torque_thresholds_accel"_a,
				"upper_torque_thresholds_accel"_a,
				"lower_torque_thresholds_nominal"_a,
				"upper_torque_thresholds_nominal"_a,
				"lower_force_thresholds_accel"_a,
				"upper_force_thresholds_accel"_a,
				"lower_force_thresholds_nominal"_a,
				"upper_force_thresholds_nominal"_a)
		.def("set_joint_impedance", &Robot::setJointImpedance, "k_q"_a)
		.def("set_cartesian_impedance", &Robot::setCartesianImpedance, "k_x"_a)
        .def("set_default_behavior", &Robot::setDefaultBehavior)

		.def("stop", &Robot::stop)
        .def("recover_from_errors", &Robot::recoverFromErrors)
		;

    // Python Gripper class
    py::class_<Gripper>(m, "Gripper")
        .def(py::init<const std::string &>(), "ip"_a)

		.def_property_readonly("server_version", &Gripper::serverVersion)
		.def_property_readonly("temperature", &Gripper::temperature)
		.def_property_readonly("max_width", &Gripper::maxWidth)
		.def_property_readonly("width", &Gripper::width)
		.def_property_readonly("is_grasped", &Gripper::isGrasped)

		.def("homing", &Gripper::homing, py::call_guard<py::gil_scoped_release>())
        .def("grasp", (bool (Gripper::*)(double, double, double)) &Gripper::grasp,
        		py::call_guard<py::gil_scoped_release>(),"width"_a, "speed"_a, "force"_a)
		.def("grasp", (bool (Gripper::*)(double, double, double, double)) &Gripper::grasp,
				py::call_guard<py::gil_scoped_release>(),"width"_a, "speed"_a, "force"_a,
				"epsilon_inner"_a)
        .def("grasp", (bool (Gripper::*)(double, double, double, double, double)) &Gripper::grasp,
        		py::call_guard<py::gil_scoped_release>(),"width"_a, "speed"_a, "force"_a,
				"epsilon_inner"_a, "epsilon_outer"_a)
        .def("move", &Gripper::move, py::call_guard<py::gil_scoped_release>(), "width"_a, "speed"_a)
		.def("stop", &Gripper::stop)
		;

	py::register_exception<franka::Exception>(m, "Exception");
    py::register_exception<franka::CommandException>(m, "CommandException");
    py::register_exception<franka::ControlException>(m, "ControlException");
    py::register_exception<franka::IncompatibleVersionException>(m, "IncompatibleVersionException");
    py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException");
    py::register_exception<franka::ModelException>(m, "ModelException");
    py::register_exception<franka::NetworkException>(m, "NetworkException");
    py::register_exception<franka::ProtocolException>(m, "ProtocolException");
    py::register_exception<franka::RealtimeException>(m, "RealtimeException");
}

/*
 * ecp_g_get_position.cpp
 *
 *  Created on: Apr 29, 2010
 *      Author: rtulwin
 */

#include "ecp_g_get_position.h"

#include "lib/exception.h"
#include <boost/throw_exception.hpp>

#define THROW_NONFATAL_ERROR(x) \
	BOOST_THROW_EXCEPTION(\
		lib::exception::NonFatal_error() << \
		lib::exception::error_code(x) \
	)

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {


get_position::get_position(common::task::task& _ecp_task, bool _is_synchronised, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
        generator (_ecp_task) {
	position = vector<double>();
	this->axes_num = axes_num;
	this->pose_spec = pose_spec;
}

get_position::~get_position() {

}

bool get_position::first_step() {
	switch (pose_spec) {
		case lib::ECP_XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
			break;
		case lib::ECP_XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
			break;
		case lib::ECP_MOTOR:
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::ECP_JOINT:
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
		default:
			THROW_NONFATAL_ERROR(INVALID_POSE_SPECIFICATION);
	}
	return true;
}

bool get_position::next_step() {
	if (pose_spec == lib::ECP_XYZ_ANGLE_AXIS || pose_spec == lib::ECP_XYZ_EULER_ZYZ) {

		lib::Homog_matrix actual_position;
		actual_position.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

		if (pose_spec == lib::ECP_XYZ_ANGLE_AXIS) {
			lib::Xyz_Angle_Axis_vector angle_axis_vector;
			actual_position.get_xyz_angle_axis(angle_axis_vector);
			angle_axis_vector.to_vector(position);

		} else if (pose_spec == lib::ECP_XYZ_EULER_ZYZ) {
			lib::Xyz_Euler_Zyz_vector euler_vector;
			actual_position.get_xyz_euler_zyz(euler_vector);
			euler_vector.to_vector(position);

		} else {
			THROW_NONFATAL_ERROR(INVALID_POSE_SPECIFICATION);
		}

	} else if (pose_spec == lib::ECP_JOINT || pose_spec == lib::ECP_MOTOR) {
		for (int i = 0; i < axes_num; i++) {
			position.push_back(the_robot->reply_package.arm.pf_def.arm_coordinates[i]);
		}
	} else {
		THROW_NONFATAL_ERROR(INVALID_POSE_SPECIFICATION);
	}
	return true;
}

vector<double> get_position::get_position_vector() {
	return position;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

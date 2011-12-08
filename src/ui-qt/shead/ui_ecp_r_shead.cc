#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_shead.h"

namespace mrrocpp {
namespace ui {
namespace shead {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		EcpRobotDataPort(_ui_robot)
{
	//	the_robot = (boost::shared_ptr <robot_t>) new ecp::shead1::robot(*(ui_robot.interface.config), *(ui_robot.msg));
}

void EcpRobot::move_motors(const double final_position[lib::shead::NUM_OF_SERVOS], lib::epos::EPOS_MOTION_VARIANT motion_variant)
{
	the_robot->epos_motor_command_data_port.data.motion_variant = motion_variant;

	for (int i = 0; i < lib::shead::NUM_OF_SERVOS; ++i) {
		the_robot->epos_motor_command_data_port.data.desired_position[i] = final_position[i];
	}

	the_robot->epos_motor_command_data_port.set();

	execute_motion();
}

void EcpRobot::move_joints(const double final_position[lib::shead::NUM_OF_SERVOS], lib::epos::EPOS_MOTION_VARIANT motion_variant)
{
	the_robot->epos_joint_command_data_port.data.motion_variant = motion_variant;

	for (int i = 0; i < lib::shead::NUM_OF_SERVOS; ++i) {
		the_robot->epos_joint_command_data_port.data.desired_position[i] = final_position[i];
	}

	the_robot->epos_joint_command_data_port.set();
	execute_motion();
}

void EcpRobot::clear_fault()
{

	the_robot->epos_clear_fault_data_port.set();

	execute_motion();
}

void EcpRobot::stop_motors()
{
	the_robot->epos_brake_command_data_port.set();

	execute_motion();
}

}
} //namespace ui
} //namespace mrrocpp

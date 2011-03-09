// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <fcntl.h>
#include <cerrno>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_single_motor.h"

#include "robot/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"
#include "robot/sarkofag/ecp_r_sarkofag.h"
#include "robot/conveyor/ecp_r_conv.h"
#include "robot/shead/ecp_r_shead.h"
#include "robot/polycrank/ecp_r_polycrank.h"

namespace mrrocpp {
namespace ui {
namespace single_motor {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
	common::EcpRobot(_interface, _robot_name)
{

	if (_robot_name == lib::irp6ot_tfg::ROBOT_NAME) {
		/* TR
		 ecp = new ecp::irp6ot_tfg::robot(*(_interface.config), *(_interface.all_ecp_msg));
		 */
		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.04;
			JOINT_STEP[j] = 0.00001;
		}

	} else if (_robot_name == lib::irp6p_tfg::ROBOT_NAME) {

		ecp = new ecp::irp6p_tfg::robot(*(_interface.config), *(_interface.all_ecp_msg));

		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.04;
			JOINT_STEP[j] = 0.00001;
		}

	} else if (_robot_name == lib::sarkofag::ROBOT_NAME) {

		ecp = new ecp::sarkofag::robot(*(_interface.config), *(_interface.all_ecp_msg));

		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.4;
			JOINT_STEP[j] = 0.0001;
		}
	} else if (_robot_name == lib::conveyor::ROBOT_NAME) {

		ecp = new ecp::conveyor::robot(*(_interface.config), *(_interface.all_ecp_msg));

		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.04;
			JOINT_STEP[j] = 0.00004;
		}
	} else if (_robot_name == lib::polycrank::ROBOT_NAME) {

		ecp = new ecp::polycrank::robot(*(_interface.config), *(_interface.all_ecp_msg));
		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.04;
			JOINT_STEP[j] = 0.00004;
		}
	}

	assert(ecp);

	// Konstruktor klasy
	ecp->ecp_command.robot_model.kinematic_model.kinematic_model_no = 0;
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::MOTOR;
	ecp->ecp_command.motion_steps = 0;
	ecp->ecp_command.value_in_step_no = 0;

	ecp->synchronised = false;
}


}
} //namespace ui
} //namespace mrrocpp



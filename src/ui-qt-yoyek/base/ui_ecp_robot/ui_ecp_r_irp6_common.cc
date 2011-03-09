// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <cfloat>
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <cmath>
#include <fcntl.h>
#include <cerrno>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../interface.h"
#include "ui_ecp_r_irp6_common.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace ui {
namespace irp6 {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
	common::EcpRobot(_interface, _robot_name)
{

	if (_robot_name == lib::irp6ot_m::ROBOT_NAME) {

		ecp = new ecp::irp6ot_m::robot(*(_interface.config), *(_interface.all_ecp_msg));

		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.05;
		}

		JOINT_STEP[0] = 0.00004;
		for (int j = 1; j < ecp->number_of_servos; j++) {
			JOINT_STEP[j] = 0.0004;
		}

		for (int j = 0; j < 3; j++) {
			END_EFFECTOR_STEP[j] = 0.00002;
		}

		for (int j = 3; j < 6; j++) {
			END_EFFECTOR_STEP[j] = 0.0002;
		}

	} else if (_robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp = new ecp::irp6p_m::robot(*(_interface.config), *(_interface.all_ecp_msg));

		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.05;
			JOINT_STEP[j] = 0.0004;
		}

		for (int j = 0; j < 3; j++) {
			END_EFFECTOR_STEP[j] = 0.00002;
		}

		for (int j = 3; j < 6; j++) {
			END_EFFECTOR_STEP[j] = 0.0002;
		}

	}

	init();

}


}
} //namespace ui
} //namespace mrrocpp

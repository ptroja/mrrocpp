#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_irp6p_tfg.h"

namespace mrrocpp {
namespace ui {
namespace irp6p_tfg {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		common012::EcpRobot(_ui_robot)
{
	ecp = new ecp::irp6p_tfg::robot(*(ui_robot.interface.config), *(ui_robot.msg));

	for (int j = 0; j < ecp->number_of_servos; j++) {
		MOTOR_STEP[j] = 0.4;
		JOINT_STEP[j] = 0.00001;
	}

	init();
}

}
} //namespace ui
} //namespace mrrocpp

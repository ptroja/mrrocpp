#ifndef _UI_ECP_R_BASE_H
#define _UI_ECP_R_BASE_H

#include "../ui.h"
#include "../ui_robot.h"

#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {
class UiRobot;

// ---------------------------------------------------------------
class EcpRobot
{
public:
	common::UiRobot& ui_robot;

	ecp::common::robot::common_buffers_ecp_robot *ecp;

	EcpRobot(common::UiRobot& _ui_robot); // Konstruktor

	// by Y - do odczytu stanu poczatkowego robota
	void get_controller_state(lib::controller_state_t & robot_controller_initial_state_l);

	virtual void execute_motion(void);

	virtual ~EcpRobot();
};

}
} //namespace ui
} //namespace mrrocpp
#endif

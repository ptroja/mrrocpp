#ifndef _UI_ECP_R_BASE_H
#define _UI_ECP_R_BASE_H

#include "../ui.h"
#include "../ui_robot.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"

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

	virtual ~EcpRobot();

};

}
} //namespace ui
} //namespace mrrocpp
#endif

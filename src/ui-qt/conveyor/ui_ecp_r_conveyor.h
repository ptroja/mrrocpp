#ifndef _UI_ECP_R_CONVEYOR_H
#define _UI_ECP_R_CONVEYOR_H

#include "../base/ui.h"
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/conveyor/ecp_r_conv.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common012.h"
namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace conveyor {

// ---------------------------------------------------------------
class EcpRobot : public common012::EcpRobot
{

public:

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(common::UiRobot& _ui_robot); // Konstruktor

};

}
} //namespace ui
} //namespace mrrocpp

#endif


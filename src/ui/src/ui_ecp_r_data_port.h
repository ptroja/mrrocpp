// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_DATA_PORT_H
#define _UI_ECP_R_DATA_PORT_H

#include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
// ---------------------------------------------------------------
class EcpRobotDataPort
{

public:
	Interface& interface;
	ecp::common::robot::ecp_robot *the_robot;

	// by Y - do odczytu stanu poczatkowego robota
	void get_controller_state(lib::controller_state_t & robot_controller_initial_state_l);
	virtual void execute_motion(void);

	// ecp_buffer ui_edp_package; // by Y
	EcpRobotDataPort(Interface& _interface); // Konstruktor

	virtual ~EcpRobotDataPort();

};

}
} //namespace ui
} //namespace mrrocpp

#endif


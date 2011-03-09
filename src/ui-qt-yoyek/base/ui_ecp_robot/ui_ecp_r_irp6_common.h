// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_MANIP_H
#define _UI_ECP_R_MANIP_H

#include "../ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "ui_ecp_r_common.h"

#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace irp6 {

// ---------------------------------------------------------------
class EcpRobot : public common::EcpRobot
{
public:

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(common::Interface& _interface, lib::robot_name_t _robot_name); // Konstruktor


};

}
} //namespace ui
} //namespace mrrocpp

#endif

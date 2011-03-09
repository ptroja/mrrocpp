// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_SMB_H
#define _UI_ECP_R_SMB_H

#include "../base/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "robot/smb/ecp_r_smb.h"
#include "../base/ui_ecp_robot/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace smb {

// ---------------------------------------------------------------
class EcpRobot : public common::EcpRobotDataPort
{

public:

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(common::Interface& _interface); // Konstruktor

};

}
} //namespace ui
} //namespace mrrocpp

#endif


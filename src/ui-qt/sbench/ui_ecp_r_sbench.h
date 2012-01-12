#ifndef _UI_ECP_R_SMB_H
#define _UI_ECP_R_SMB_H

/*!
 * @file
 * @brief File contains ui EcpRobot class declaration for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */


#include "../base/ui.h"

#include "base/lib/configurator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_robot.h"
#include "robot/sbench/ecp_r_sbench.h"
#include "../base/ui_ecp_robot/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace sbench {

// ---------------------------------------------------------------
class EcpRobot : public common::_EcpRobotDataPort <ecp::sbench::robot>
{

public:

	// ecp_buffer ui_edp_package; // by Y
	EcpRobot(common::UiRobot& _ui_robot); // Konstruktor

};

}
} //namespace ui
} //namespace mrrocpp

#endif


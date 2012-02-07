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

#include "base/ecp/ecp_robot.h"
#include "robot/sbench/ecp_r_sbench.h"
#include "../base/ui_ecp_robot/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace sbench {

/*!
 * @class
 * @brief sbench EcpRobot class
 * it specializes and derives from _EcpRobotDataPort
 * @author yoyek
 *
 *  @ingroup sbench
 */
class EcpRobot : public common::_EcpRobotDataPort <ecp::sbench::robot>
{
public:
	/**
	 * @brief constructor
	 * @param _ui_robot UiRobot object pointer
	 */
	EcpRobot(common::UiRobot& _ui_robot);

};

}
} //namespace ui
} //namespace mrrocpp

#endif


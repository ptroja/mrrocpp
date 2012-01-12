/*!
 * @file
 * @brief File contains ui EcpRobot class definition for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_sbench.h"

namespace mrrocpp {
namespace ui {
namespace sbench {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		EcpRobotDataPort(_ui_robot)
{
	the_robot = (boost::shared_ptr <robot_t>) new ecp::sbench::robot(*(ui_robot.interface.config), *(ui_robot.msg));
	common::EcpRobot::ecp = (ecp::common::robot::ecp_robot*) (the_robot.get());
}

}
} //namespace ui
} //namespace mrrocpp

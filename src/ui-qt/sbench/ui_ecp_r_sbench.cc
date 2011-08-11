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
}

}
} //namespace ui
} //namespace mrrocpp

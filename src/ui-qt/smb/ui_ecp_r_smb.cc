#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "ui_ecp_r_smb.h"

namespace mrrocpp {
namespace ui {
namespace smb {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
	EcpRobotDataPort(_ui_robot)
{
	the_robot = (boost::shared_ptr <robot_t>) new ecp::smb::robot(*(ui_robot.interface.config), *(ui_robot.msg));
}

}
} //namespace ui
} //namespace mrrocpp

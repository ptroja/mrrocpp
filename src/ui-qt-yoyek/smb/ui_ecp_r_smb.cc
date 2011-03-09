#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "ui_ecp_r_smb.h"

namespace mrrocpp {
namespace ui {
namespace smb {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface) :
	EcpRobotDataPort(_interface)
{
	the_robot = (boost::shared_ptr<robot_t>) new ecp::smb::robot(*(_interface.config), *(_interface.all_ecp_msg));
}

}
} //namespace ui
} //namespace mrrocpp

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_shead.h"

namespace mrrocpp {
namespace ui {
namespace shead {



// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface) :
	EcpRobotDataPort(_interface)
{
	the_robot = (boost::shared_ptr<robot_t>) new ecp::shead::robot(*(_interface.config), *(_interface.all_ecp_msg));
}

}
} //namespace ui
} //namespace mrrocpp

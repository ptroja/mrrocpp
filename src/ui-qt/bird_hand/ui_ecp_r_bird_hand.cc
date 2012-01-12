#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_bird_hand.h"

namespace mrrocpp {
namespace ui {
namespace bird_hand {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		EcpRobotDataPort(_ui_robot)
{

	the_robot = (boost::shared_ptr <robot_t>) new ecp::bird_hand::robot(*(ui_robot.interface.config), *(ui_robot.msg));
	common::EcpRobot::ecp = (ecp::common::robot::common_buffers_ecp_robot*) (the_robot.get());
}

}
} //namespace ui
} //namespace mrrocpp

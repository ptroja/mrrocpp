// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <fcntl.h>
#include <cerrno>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "ui/src/ui_class.h"

#include "base/lib/sr/srlib.h"

#include "ui/src/shead/ui_ecp_r_shead.h"

namespace mrrocpp {
namespace ui {
namespace shead {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface) :
	common::EcpRobotDataPort(_interface)
{

	the_robot = new ecp::shead::robot(*(_interface.config), *(_interface.all_ecp_msg));

	assert(the_robot);

}

}
} //namespace ui
} //namespace mrrocpp

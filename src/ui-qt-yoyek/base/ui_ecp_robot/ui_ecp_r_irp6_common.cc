// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <cfloat>
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <cmath>
#include <fcntl.h>
#include <cerrno>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../interface.h"
#include "ui_ecp_r_irp6_common.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"



namespace mrrocpp {
namespace ui {
namespace irp6 {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
	common::EcpRobot(_interface, _robot_name)
{


}


}
} //namespace ui
} //namespace mrrocpp

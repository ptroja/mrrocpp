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
#include "../interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_single_motor.h"

#include "robot/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"
#include "robot/sarkofag/ecp_r_sarkofag.h"
#include "robot/conveyor/ecp_r_conv.h"
#include "robot/shead/ecp_r_shead.h"
#include "robot/polycrank/ecp_r_polycrank.h"

namespace mrrocpp {
namespace ui {
namespace single_motor {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
	common::EcpRobot(_interface, _robot_name)
{

}

}
} //namespace ui
} //namespace mrrocpp



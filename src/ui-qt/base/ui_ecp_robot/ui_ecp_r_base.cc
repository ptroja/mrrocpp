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
#include <fcntl.h>
#include <cerrno>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_base.h"

namespace mrrocpp {
namespace ui {
namespace common {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
		ui_robot(_ui_robot), ecp(NULL)
{

}
// ---------------------------------------------------------------

EcpRobot::~EcpRobot()
{

}

}
} //namespace ui
} //namespace mrrocpp


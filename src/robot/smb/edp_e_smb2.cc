#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp_e_smb2.h"
#include "const_smb2.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/smb/kinematic_model_smb.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace smb2 {

// Konstruktor.
effector::effector(common::shell &_shell) :
	smb::effector(_shell, lib::smb2::ROBOT_NAME)
{

}

}
// namespace smb


namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new smb2::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


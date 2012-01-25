#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_smb2.h"
#include "const_smb2.h"
#include "base/edp/reader.h"
#include "robot/smb/kinematic_model_smb.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace smb2 {

#include "base/lib/debug.hpp"

effector::effector(common::shell &_shell) :
		smb::effector(_shell, lib::smb2::ROBOT_NAME)
{
	DEBUG_METHOD;

	// Set synchronization values, different for both SMBs.
	pkm_zero_position_voltage = 2500;
	pkm_zero_position_offset = 9850;
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


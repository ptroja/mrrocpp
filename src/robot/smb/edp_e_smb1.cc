#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_smb1.h"
#include "const_smb1.h"
#include "base/edp/reader.h"
#include "robot/smb/kinematic_model_smb.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace smb1 {

#include "base/lib/debug.hpp"

effector::effector(common::shell &_shell) :
		smb::effector(_shell, lib::smb1::ROBOT_NAME)
{
	DEBUG_METHOD;

	// Set synchronization values, different for both SMBs.
	pkm_zero_position_voltage = 2300;
	pkm_zero_position_offset = 9850;

}

}
// namespace smb

namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new smb1::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


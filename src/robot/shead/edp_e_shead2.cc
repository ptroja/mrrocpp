#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_shead2.h"
#include "const_shead2.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/shead/kinematic_model_shead.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace shead2 {

// Konstruktor.
effector::effector(common::shell &_shell) :
	shead::effector(_shell, lib::shead2::ROBOT_NAME)
{

}

}// namespace shead2


namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new shead2::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


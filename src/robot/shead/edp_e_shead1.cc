#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_shead1.h"
#include "const_shead1.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/shead/kinematic_model_shead.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace shead1 {

// Konstruktor.
effector::effector(common::shell &_shell) :
	shead::effector(_shell, lib::shead1::ROBOT_NAME)
{

}

}// namespace shead1


namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new shead1::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


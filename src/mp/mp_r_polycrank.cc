#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp_r_polycrank.h"

namespace mrrocpp {
namespace mp {
namespace robot {

polycrank::polycrank (task::task &mp_object_l) :
		manip_and_conv (lib::ROBOT_POLYCRANK, "[ecp_polycrank]", mp_object_l)
{}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


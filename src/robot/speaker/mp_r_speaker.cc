#include <cstring>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "base/mp/mp_task.h"
#include "robot/speaker/mp_r_speaker.h"

namespace mrrocpp {
namespace mp {
namespace robot {

speaker::speaker(task::task &mp_object_l) :
	robot(lib::speaker::ROBOT_NAME, lib::speaker::ECP_SECTION, mp_object_l, 0)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


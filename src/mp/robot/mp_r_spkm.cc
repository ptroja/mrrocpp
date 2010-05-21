#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/robot/mp_r_spkm.h"

namespace mrrocpp {
namespace mp {
namespace robot {

spkm::spkm(task::task &mp_object_l) :
	manip_and_conv(lib::ROBOT_SPKM, ECP_SPKM_SECTION, mp_object_l,
			SPKM_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

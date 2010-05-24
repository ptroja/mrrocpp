#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/robot/mp_r_speaker.h"

namespace mrrocpp {
namespace mp {
namespace robot {

speaker::speaker(task::task &mp_object_l) :
	robot(lib::ROBOT_SPEAKER, ECP_SPEAKER_SECTION, mp_object_l)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


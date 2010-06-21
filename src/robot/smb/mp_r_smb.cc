#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "robot/smb/mp_r_smb.h"

namespace mrrocpp {
namespace mp {
namespace robot {

smb::smb(task::task &mp_object_l) :
	motor_driven(lib::ROBOT_SMB, ECP_SMB_SECTION, mp_object_l, SMB_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


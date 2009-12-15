#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/robot/mp_r_manip_and_conv.h"

namespace mrrocpp {
namespace mp {
namespace robot {

manip_and_conv::manip_and_conv(lib::robot_name_t l_robot_name, const char* _section_name, task::task &mp_object_l) :
	robot(l_robot_name, _section_name, mp_object_l), servos_number(0), has_gripper(false)
{
	switch (l_robot_name) {
		case lib::ROBOT_IRP6_ON_TRACK:
			servos_number = IRP6_ON_TRACK_NUM_OF_SERVOS;
			has_gripper = true;
			break;
		case lib::ROBOT_IRP6_POSTUMENT:
			servos_number = IRP6_POSTUMENT_NUM_OF_SERVOS;
			has_gripper = true;
			break;
		case lib::ROBOT_CONVEYOR:
			servos_number = CONVEYOR_NUM_OF_SERVOS;
			break;
		case lib::ROBOT_IRP6_MECHATRONIKA:
			servos_number = IRP6_MECHATRONIKA_NUM_OF_SERVOS;
			break;
		default: // error
			break;
	}
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

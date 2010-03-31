#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/robot/mp_r_manip_and_conv.h"

namespace mrrocpp {
namespace mp {
namespace robot {

manip_and_conv::manip_and_conv(lib::robot_name_t l_robot_name,
		const char* _section_name, task::task &mp_object_l,
		int _number_of_servos) :
	robot(l_robot_name, _section_name, mp_object_l), servos_number(
			_number_of_servos) {
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

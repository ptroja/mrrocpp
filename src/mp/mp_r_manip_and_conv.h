#ifndef MP_R_MANIP_AND_CONV_H_
#define MP_R_MANIP_AND_CONV_H_

#include "mp/mp.h"
#include "mp/mp_robot.h"
#include "lib/conveyor_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class manip_and_conv : public robot
{
	private:
		int servos_number;
		bool has_gripper;

	public:
		manip_and_conv (lib::robot_name_t l_robot_name, const char* _section_name, task::task &mp_object_l);


};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_MANIP_AND_CONV_H_*/

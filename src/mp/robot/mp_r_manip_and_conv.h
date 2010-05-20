#ifndef MP_R_MANIP_AND_CONV_H_
#define MP_R_MANIP_AND_CONV_H_

#include "lib/impconst.h"
#include "mp/mp.h"
#include "mp/robot/mp_robot.h"
#include "lib/robot_consts/all_robots_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class manip_and_conv : public robot
{
private:
	const int servos_number;

public:
	manip_and_conv(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l, int _number_of_servos);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_MANIP_AND_CONV_H_*/

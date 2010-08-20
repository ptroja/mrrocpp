#ifndef MP_R_motor_driven_H_
#define MP_R_motor_driven_H_

#include "lib/impconst.h"

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_robot.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class motor_driven : public robot
{
private:
	const int servos_number;

public:
	motor_driven(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l, int _number_of_servos);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_motor_driven_H_*/

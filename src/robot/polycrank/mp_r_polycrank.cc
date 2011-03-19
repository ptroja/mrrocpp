
#include "base/mp/MP_main_error.h"
#include "robot/polycrank/mp_r_polycrank.h"

namespace mrrocpp {
namespace mp {
namespace robot {

polycrank::polycrank(task::task &mp_object_l) :
			robot(lib::polycrank::ROBOT_NAME, lib::polycrank::ECP_SECTION, mp_object_l, lib::polycrank::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

/*
#include "base/mp/MP_main_error.h"
#include "robot/conveyor/mp_r_conveyor.h"

namespace mrrocpp {
namespace mp {
namespace robot {

conveyor::conveyor(task::task &mp_object_l) :
	robot(lib::conveyor::ROBOT_NAME, lib::conveyor::ECP_SECTION, mp_object_l, lib::conveyor::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp
*/

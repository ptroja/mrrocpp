/*!
 * @file
 * @brief File contains mp motor_driven robot definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {

motor_driven::motor_driven(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l, int _number_of_servos) :
	robot(l_robot_name, _section_name, mp_object_l), servos_number(_number_of_servos)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

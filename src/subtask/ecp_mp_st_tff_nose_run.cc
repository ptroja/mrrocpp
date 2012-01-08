/*!
 * @file
 * @brief File contains bias_edp_force definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "ecp_mp_st_tff_nose_run.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sub_task {

void behaviour_specification_data_type::set_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z, lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az)
{
	behaviour[0] = x;
	behaviour[1] = y;
	behaviour[2] = z;
	behaviour[3] = ax;
	behaviour[4] = ay;
	behaviour[5] = az;
}

behaviour_specification_data_type::behaviour_specification_data_type(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z, lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az)
{
	set_behaviour(x, y, z, ax, ay, az);
}

behaviour_specification_data_type::behaviour_specification_data_type()
{
	set_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT);
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp

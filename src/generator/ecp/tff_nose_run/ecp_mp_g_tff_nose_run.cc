/*!
 * @file
 * @brief File contains ecp mp tff nose run definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "ecp_mp_g_tff_nose_run.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {
namespace tff_nose_run {

void behaviour_specification_data_type::set_compliance(bool x, bool y, bool z, bool ax, bool ay, bool az)
{
	if (x) {
		behaviour[0] = lib::CONTACT;
	} else {
		behaviour[0] = lib::UNGUARDED_MOTION;
	}

	if (y) {
		behaviour[1] = lib::CONTACT;
	} else {
		behaviour[1] = lib::UNGUARDED_MOTION;
	}

	if (z) {
		behaviour[2] = lib::CONTACT;
	} else {
		behaviour[2] = lib::UNGUARDED_MOTION;
	}

	if (ax) {
		behaviour[3] = lib::CONTACT;
	} else {
		behaviour[3] = lib::UNGUARDED_MOTION;
	}

	if (ay) {
		behaviour[4] = lib::CONTACT;
	} else {
		behaviour[4] = lib::UNGUARDED_MOTION;
	}

	if (az) {
		behaviour[5] = lib::CONTACT;
	} else {
		behaviour[5] = lib::UNGUARDED_MOTION;
	}

}

behaviour_specification_data_type::behaviour_specification_data_type(bool x, bool y, bool z, bool ax, bool ay, bool az)
{
	set_compliance(x, y, z, ax, ay, az);
}

behaviour_specification_data_type::behaviour_specification_data_type()
{
	set_compliance(true, true, true, true, true, true);
}

}
} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

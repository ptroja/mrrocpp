/*!
 * @file
 * @brief File contains ecp_mp base generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include "base/ecp_mp/ecp_mp_generator.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {

generator::generator(lib::sr_ecp& _sr_ecp_msg) :
	sr_ecp_msg(_sr_ecp_msg), trigger(false), node_counter(0)
{
}

bool generator::check_and_null_trigger()
{
	bool returned_value = false;
	if (trigger) {
		trigger = false;
		returned_value = true;
	}

	return returned_value;
}

generator::~generator()
{
}

} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

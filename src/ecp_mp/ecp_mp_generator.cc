#include "ecp_mp/ecp_mp_generator.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {

base::base(lib::sr_ecp& _sr_ecp_msg) :
	sr_ecp_msg(_sr_ecp_msg),
	trigger(false),
	node_counter(0)
	{}


bool base::check_and_null_trigger()
{
	bool returned_value = false;
	if (trigger)
	{
		trigger = false;
		returned_value = true;
	}

	return returned_value;
}


base::~base()
{}

} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp
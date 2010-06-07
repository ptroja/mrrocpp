#include "ecp_mp/ecp_mp_generator.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {

generator::generator(lib::sr_ecp& _sr_ecp_msg) :
	sr_ecp_msg(_sr_ecp_msg),
	node_counter(0)
{
}

bool generator::check_and_null_trigger()
{
	// this require ui_command_buffer at this level of hierarchy;
	// do agree with it?
	return false;
}

generator::~generator()
{
}

} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

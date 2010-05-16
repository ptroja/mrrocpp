#include "ecp_mp/transmitter/transmitter.h"
#include "ecp_mp/task/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

transmitter_base::transmitter_base(TRANSMITTER_ENUM _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object)
		: transmitter_name(_transmitter_name), sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg)
{
}

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp

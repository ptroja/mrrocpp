/*!
 * @file
 * @brief File contains ecp_mp transmitter base class definition
 * @author ptroja, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include "base/ecp_mp/transmitter.h"
#include "base/ecp_mp/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

transmitter_base::transmitter_base(lib::TRANSMITTER_t _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object) :
	transmitter_name(_transmitter_name), sr_ecp_msg(*_ecp_mp_object.sr_ecp_msg)
{
}

transmitter_base::~transmitter_base()
{
}

bool transmitter_base::t_read(bool wait)
{
	return true;
}

bool transmitter_base::t_write(void)
{
	return true;
}

transmitter_error::transmitter_error(lib::error_class_t err_cl) :
	error_class(err_cl)
{
}

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp

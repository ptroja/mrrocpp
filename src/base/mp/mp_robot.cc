/*!
 * @file
 * @brief File contains mp base robot definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstring>
#include <unistd.h>
#include <cerrno>
#include <sys/types.h>
#include <sys/wait.h>

#include <boost/foreach.hpp>

#include "base/lib/datastr.h"

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"

#include "base/lib/agent/RemoteAgent.h"
#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace mp {
namespace robot {

robot::robot(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l, int _number_of_servos) :
	ecp_mp::robot(l_robot_name),
	number_of_servos(_number_of_servos),
	ECP_pid(mp_object_l.config, _section_name),
	ecp(_section_name),
	command(ecp, "command"),
	mp_object(mp_object_l),
	sr_ecp_msg(*(mp_object_l.sr_ecp_msg)),
	continuous_coordination(false),
	reply(_section_name),
	communicate_with_ecp(true)
{
	mp_object.registerBuffer(reply);
}

robot::~robot()
{
	fprintf(stderr, "robot::~robot()\n");
}

void robot::start_ecp(void)
{
	mp_command.command = lib::START_TASK;

	command.Set(mp_command);
}

void robot::execute_motion(void)
{
	command.Set(mp_command);
}

void robot::terminate_ecp(void)
{
	// zlecenie STOP zakonczenia ruchu
	mp_command.command = lib::STOP;

	command.Set(mp_command);
}

MP_error::MP_error(lib::error_class_t err0, uint64_t err1) :
	error_class(err0), error_no(err1)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

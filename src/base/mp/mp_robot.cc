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
#include "base/lib/mis_fun.h"

#include "base/lib/agent/RemoteAgent.h"
#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace mp {
namespace robot {

robot::robot(lib::robot_name_t l_robot_name, task::task &mp_object_l, int _number_of_servos) :
	ecp_mp::robot(l_robot_name), number_of_servos(_number_of_servos),
			ECP_pid(mp_object_l.config, mp_object_l.config.get_ecp_section(robot_name)),
			ecp(mp_object_l.config.get_ecp_section(robot_name)), command(ecp, "command"), mp_object(mp_object_l),
			sr_ecp_msg(*(mp_object_l.sr_ecp_msg)), reply(mp_object_l.config.get_ecp_section(robot_name)),
			ecp_reply_package(reply.access), communicate_with_ecp(true)
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

	command.Send(mp_command);

}

void robot::pause_ecp(void)
{
	mp_command.command = lib::PAUSE_TASK;

	command.Send(mp_command);

}

void robot::resume_ecp(void)
{
	mp_command.command = lib::RESUME_TASK;

	command.Send(mp_command);
}

void robot::execute_motion(void)
{
	command.Send(mp_command);
}

void robot::terminate_ecp(void)
{
	// zlecenie STOP zakonczenia ruchu
	mp_command.command = lib::STOP;
	sr_ecp_msg.message(lib::NON_FATAL_ERROR, "terminate_ecp");

	command.Send(mp_command);
}

void robot::ecp_errors_handler()
{

	if (reply.Get().reply == lib::ERROR_IN_ECP) {
		// Odebrano od ECP informacje o bledzie
		throw MP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);
	}
}

MP_error::MP_error(lib::error_class_t err0, uint64_t err1) :
	error_class(err0), error_no(err1)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

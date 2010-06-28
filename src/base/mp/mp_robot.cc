// -------------------------------------------------------------------------
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/datastr.h"

#include "lib/srlib.h"
#include "base/mp/mp.h"
#include "base/mp/mp_task.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

#include "lib/exception.h"
#include <boost/throw_exception.hpp>
#include <boost/exception/errinfo_errno.hpp>

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace mp {
namespace robot {

// -------------------------------------------------------------------
robot::robot(lib::robot_name_t l_robot_name, const std::string & _section_name, task::task &mp_object_l) :
	new_data_flag(false),
	ecp_mp::robot(l_robot_name),
	mp_object(mp_object_l),

	ecp_reply_package_buffer(mp_object, std::string("mp:") + _section_name),
	ECP_pid(mp_object.config.process_spawn(_section_name)),
	ecp_agent(_section_name),
	mp_command_buffer(ecp_agent, "command"),

	communicate(true), // domyslnie robot jest aktywny
	sr_ecp_msg(*(mp_object_l.sr_ecp_msg)),
	continuous_coordination(false)
{
#if !defined(PROCESS_SPAWN_RSH)
	const std::string node_name(mp_object.config.value <std::string> ("node_name", _section_name));

	nd = mp_object.config.return_node_number(node_name);
#endif
}
// -------------------------------------------------------------------

robot::~robot()
{
	fprintf(stderr, "robot::~robot()\n");

#if defined(PROCESS_SPAWN_RSH)
	if (kill(ECP_pid, SIGTERM) == -1) {
		perror("kill()");
		fprintf(stderr, "kill failed for robot %s pid %d\n", lib::toString(robot_name).c_str(), ECP_pid);
	} else {
		if (waitpid(ECP_pid, NULL, 0) == -1) {
			perror("waitpid()");
		}
	}
#else
	SignalKill(nd, ECP_pid, 0, SIGTERM, 0, 0);
#endif
}

bool robot::is_new_data()
{
	const bool new_data_status = new_data_flag;
	new_data_flag = false;
	return new_data_status;
}

// ------------------------------------------------------------------------
void robot::start_ecp(void)
{
	mp_command.command = lib::START_TASK;
	fprintf(stderr, "robot::start_ecp()\n");
	mp_command_buffer.Set(mp_command);
}
// ------------------------------------------------------------------------


// -------------------------------------------------------------------
void robot::execute_motion(void)
{
	mp_command_buffer.Set(mp_command);
}
// ---------------------------------------------------------------


// -------------------------------------------------------------------
void robot::terminate_ecp(void) { // zlecenie STOP zakonczenia ruchu
	mp_command.command = lib::STOP;

	mp_command_buffer.Set(mp_command);
}
// ---------------------------------------------------------------


} // namespace robot
} // namespace mp
} // namespace mrrocpp


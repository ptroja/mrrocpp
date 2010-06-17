#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <stdio.h>

#include "lib/mis_fun.h"
#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_generator.h"

#include "lib/exception.h"
#include <boost/throw_exception.hpp>
#include <boost/exception/errinfo_errno.hpp>

#include "lib/agent/OrDataCondition.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

task::task(lib::configurator &_config) :
	ecp_mp::task::task(_config),
	mp_agent(MP_SECTION),
	ecp_reply_buffer(mp_agent, std::string("mp:") + _config.section_name),
	mp_command_buffer(*this, "command"),
	ui_trigger_buffer(*this, "trigger"),
	ecp_m_robot(NULL),
	continuous_coordination(false)
{
	initialize_communication();
}

void task::initialize_communication()
{
	const std::string ecp_attach_point =
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "ecp_attach_point");
	const std::string sr_net_attach_point =
			config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION);

	// Obiekt do komuniacji z SR
	sr_ecp_msg = new lib::sr_ecp(lib::ECP, ecp_attach_point, sr_net_attach_point, true);
	sh_msg = new lib::sr_ecp(lib::ECP, ecp_attach_point, sr_net_attach_point, false);
}

// Badanie typu polecenia z MP
lib::MP_COMMAND task::mp_command_type(void) const
{
	return mp_command.command;
}

// Ustawienie typu odpowiedzi z ECP do MP
void task::set_ecp_reply(lib::ECP_REPLY ecp_r)
{
	ecp_reply.reply = ecp_r;
}

// Informacja dla MP o zakonczeniu zadania uzytkownika
void task::ecp_termination_notice(void)
{
	if (mp_command_type() != lib::END_MOTION) {

		set_ecp_reply(lib::TASK_TERMINATED);
		ecp_reply_buffer.Set(ecp_reply);
	}
}

// Petla odbierania wiadomosci.
void task::wait_for_stop(void)
{
	while(mp_command_buffer.Get().command != lib::STOP) {
		Wait(mp_command_buffer);
	}
}

// Oczekiwanie na polecenie START od MP
void task::wait_for_start(void)
{
	while(mp_command_buffer.Get().command != lib::START_TASK) {
		Wait(mp_command_buffer);
	}
	fprintf(stderr, "%s: GOT START COMMAND!\n", getName().c_str());
}

// Oczekiwanie na kolejne zlecenie od MP
void task::get_next_state(void)
{
	fprintf(stderr, "%s: get_next_state()\n", getName().c_str());

	while(!mp_command_buffer.Get(mp_command)) {
		Wait(mp_command_buffer);

		if (mp_command.command == lib::NEXT_STATE)
			break;
	}

	mp_2_ecp_next_state_string = mp_command.ecp_next_state.mp_2_ecp_next_state;

	// acknowledge the command
	set_ecp_reply(lib::ECP_ACKNOWLEDGE);
	ecp_reply_buffer.Set(ecp_reply);
}

ecp_sub_task::ecp_sub_task(task &_ecp_t) :
	ecp_t(_ecp_t)
{
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

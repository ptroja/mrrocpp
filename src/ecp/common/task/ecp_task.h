#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

#include "ecp_mp/task/ecp_mp_task.h"
#include "ecp/common/ecp_robot.h"
#include "lib/agent/RemoteAgent.h"
#include "lib/agent/RemoteBuffer.h"
#include "lib/agent/DataBuffer.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// klasa globalna dla calego procesu MP
class task : public ecp_mp::task::task
{
private:
	// Badanie typu polecenia z MP
	lib::MP_COMMAND mp_command_type(void) const;

	void initialize_communication(void);
protected:
	// Oczekiwanie na nowy stan od MP
	void get_next_state(void);

	//! Coordinator agent
	RemoteAgent mp_agent;
public:
	//! Output buffer for reply to a coordinator
	RemoteBuffer<lib::ECP_REPLY_PACKAGE> ecp_reply_buffer;

	//! Input buffer for command from a coordinator
	DataBuffer<lib::MP_COMMAND_PACKAGE> mp_command_buffer;

	//! Input buffer for a trigger from a UI
	DataBuffer<char> ui_trigger_buffer;

public: // TODO: following packages should be 'protected'
	// Odpowiedz ECP do MP, pola do ew. wypelnienia przez generatory
	lib::ECP_REPLY_PACKAGE ecp_reply;

	std::string mp_2_ecp_next_state_string;

	// Polecenie od MP dla TASKa
	lib::MP_COMMAND_PACKAGE mp_command;

		//ew. koordynacja ciagla domyslnie wylaczona ma wplyw na instrukcje move
		bool continuous_coordination;

	// sprawdza czy przeszedl puls do ECP lub MP
	bool pulse_check();

	// KONSTRUKTOR
	task(lib::configurator &_config);

	// Informacja dla MP o zakonczeniu zadania uzytkownika
	void ecp_termination_notice(void);

	// Oczekiwanie na polecenie START od MP
	void wait_for_start(void);

	// Oczekiwanie na STOP
	void wait_for_stop(void);

	// Oczekiwanie na STOP
	void ecp_wait_for_stop(void);

	// Oczekiwanie na polecenie od MP
	bool mp_buffer_receive_and_send(void);

	// Ustawienie typu odpowiedzi z ECP do MP
	void set_ecp_reply(lib::ECP_REPLY ecp_r);
};

task* return_created_ecp_task(lib::configurator &_config);

// klasa podzadania
class ecp_sub_task
{
protected:
	task &ecp_t;

public:
	ecp_sub_task(task &_ecp_t);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */

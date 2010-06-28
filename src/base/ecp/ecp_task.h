#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

#include "base/ecp_mp/ecp_mp_task.h"
#include "base/ecp/ecp_robot.h"
#include "lib/agent/RemoteAgent.h"
#include "lib/agent/RemoteBuffer.h"
#include "lib/agent/DataBuffer.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class ecp_sub_task;

typedef std::map <std::string, ecp_sub_task *> subtasks_t;
typedef subtasks_t::value_type subtask_pair_t;

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

	subtasks_t subtask_m;

	std::string mp_2_ecp_next_state_string;

	// Polecenie od MP dla TASKa
	lib::MP_COMMAND_PACKAGE mp_command;

	//ew. koordynacja ciagla domyslnie wylaczona ma wplyw na instrukcje move
	bool continuous_coordination;

	ecp_robot* ecp_m_robot;

	// sprawdza czy przeszedl puls do ECP lub MP
	bool pulse_check();

	// KONSTRUKTOR
	task(lib::configurator &_config);

	// dla gcc: `'class Foo' has virtual functions but non-virtual destructor` warning.
	virtual ~task();

	// methods for ECP template to redefine in concrete classes
	virtual void main_task_algorithm(void);

	virtual void mp_2_ecp_next_state_string_handler(void);
	// Informacja dla MP o zakonczeniu zadania uzytkownika
	void ecp_termination_notice(void);

	// Oczekiwanie na polecenie START od MP
	void wait_for_start(void);

	// Oczekiwanie na STOP
	void wait_for_stop(void);

	void subtasks_conditional_execution();

public:
	// TODO: what follows should be private method

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

	/*
	 * executed on the MP demand
	 */

	virtual void conditional_execution() = 0;
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */

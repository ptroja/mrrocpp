#if !defined(_ECP_TASK_H)
#define _ECP_TASK_H

#include "ecp_mp/task/ecp_mp_task.h"
#include "ecp/common/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// klasa globalna dla calego procesu MP
class task : public ecp_mp::task::task
{
	private:
#if !defined(USE_MESSIP_SRR)
		name_attach_t *ecp_attach, *trigger_attach; // by Y
		int MP_fd;
#else
		messip_channel_t *ecp_attach, *trigger_attach, *MP_fd;
#endif
		// Wysyla puls do Mp przed oczekiwaniem na spotkanie
		void send_pulse_to_mp(int pulse_code, int pulse_value = 1);

		// Receive of mp message
		int receive_mp_message(bool block);

		// Badanie typu polecenia z MP
		lib::MP_COMMAND mp_command_type(void) const;

		void initialize_communication(void);
	protected:
		// Oczekiwanie na nowy stan od MP
		void get_next_state(void);

	public: // TODO: following packages should be 'protected'
		// Odpowiedz ECP do MP, pola do ew. wypelnienia przez generatory
		lib::ECP_REPLY_PACKAGE ecp_reply;

		// Polecenie od MP dla TASKa
		lib::MP_COMMAND_PACKAGE mp_command;

		ecp_robot* ecp_m_robot;

		//ew. koordynacja ciagla domyslnie wylaczona ma wplyw na instrukcje move
			bool continuous_coordination;

		// sprawdza czy przeszedl puls do ECP lub MP
		bool pulse_check();

		// KONSTRUKTOR
		task(lib::configurator &_config);

		// dla gcc: `'class Foo' has virtual functions but non-virtual destructor` warning.
		virtual ~task();

		// methods for ECP template to redefine in concrete classes
		virtual void main_task_algorithm(void) = 0;

		// Informacja dla MP o zakonczeniu zadania uzytkownika
		void ecp_termination_notice(void);

		// Oczekiwanie na polecenie START od MP
		bool ecp_wait_for_start(void);

		// Oczekiwanie na STOP
		void ecp_wait_for_stop(void);

	public: // TODO: what follows should be private method

		// Oczekiwanie na polecenie od MP
		bool mp_buffer_receive_and_send(void);

		// Ustawienie typu odpowiedzi z ECP do MP
		void set_ecp_reply(lib::ECP_REPLY ecp_r);
};

task* return_created_ecp_task (lib::configurator &_config);


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

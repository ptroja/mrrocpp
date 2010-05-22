#ifndef MP_TASK_H_
#define MP_TASK_H_

#include "mp/generator/mp_generator.h"
#include "mp/mp.h"
#include "ecp_mp/task/ecp_mp_task.h"

#if !defined(USE_MESSIP_SRR)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#else
#include <messip.h>
#endif

namespace mrrocpp {
namespace mp {

// forward delcaration
namespace generator {
class generator;
}

namespace task {

// klasa globalna dla calego procesu MP
class task : public ecp_mp::task::task
{
private:
	void initialize_communication(void);

	/// utworzenie robotow
	virtual void create_robots(void);

public:
#if !defined(USE_MESSIP_SRR)
	static name_attach_t *mp_pulse_attach;
#else
	static messip_channel_t *mp_pulse_attach;
#endif
	/// KONSTRUKTORY
	task(lib::configurator &_config);
	virtual ~task(void);

	void stop_and_terminate(void);

	// oczekiwanie na puls z ECP
	typedef enum _MP_RECEIVE_PULSE_ENUM
	{
		NONBLOCK, BLOCK
	} RECEIVE_PULSE_MODE;

	typedef enum _WAIT_FOR_NEW_PULSE_ENUM
	{
		NEW_ECP_PULSE, NEW_UI_PULSE, NEW_UI_OR_ECP_PULSE
	} WAIT_FOR_NEW_PULSE_MODE;

	void set_next_playerpos_goal(lib::robot_name_t robot_l, const lib::playerpos_goal_t &goal);

	void set_next_ecps_state(int l_state, int l_variant, const char* l_string, int str_len, int number_of_robots, ...);

	void send_end_motion_to_ecps(int number_of_robots, ...);
	void send_end_motion_to_ecps(int number_of_robots, lib::robot_name_t *properRobotsSet);

	void run_extended_empty_gen(bool activate_trigger, int number_of_robots, ...);
	void
			run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ...);
	void
			run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, lib::robot_name_t *robotsToMove, lib::robot_name_t *robotsWaitingForTaskTermination);

	void wait_ms(int _ms_delay); // zamiast delay

	// Oczekiwanie na zlecenie START od UI
	void wait_for_start(void);// by Y&W

	// Oczekiwanie na zlecenie STOP od UI
	void wait_for_stop(void);// by Y&W dodany tryb

	// Wystartowanie wszystkich ECP
	void start_all(const common::robots_t & _robot_m);

	// Zatrzymanie wszystkich ECP
	void terminate_all(const common::robots_t & _robot_m);

	// warunkowe wyslanie pulsu zadania komunikacji do ECP
	void request_communication_with_robots(const common::robots_t & _robot_m);

	// Wyslanie rozkazu do wszystkich ECP
	void execute_all(const common::robots_t & _robot_m);

	/// method redefine in concrete classes
	virtual void main_task_algorithm(void) = 0;

	/// mapa wszystkich robotow
	common::robots_t robot_m;

	// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
	void mp_receive_ui_or_ecp_pulse(common::robots_t & _robot_m, generator::generator& the_generator);

private:
	friend class robot::robot;

	//! wait until ECP/UI calls name_open() to pulse channel;
	//! \return {identifier (scoid/QNET or socket/messip) of the next connected process}
	int wait_for_name_open(void);

	//! A server connection ID identifying UI
	int ui_scoid;

	//! flag indicating opened pulse connection from UI
	bool ui_opened;

	char ui_pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
	bool ui_new_pulse; // okresla czy jest nowy puls

	bool
			check_and_optional_wait_for_new_pulse(WAIT_FOR_NEW_PULSE_MODE process_type, const RECEIVE_PULSE_MODE desired_wait_mode);
};

task* return_created_mp_task(lib::configurator &_config);

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /*MP_TASK_H_*/

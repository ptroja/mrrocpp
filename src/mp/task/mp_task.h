#ifndef MP_TASK_H_
#define MP_TASK_H_

#include "mp/generator/mp_generator.h"
#include "mp/mp.h"
#include "ecp_mp/task/ecp_mp_task.h"
#include "lib/agent/DataBuffer.h"

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

const std::string UI_COMMAND_BUFFER = "UI command";

// klasa globalna dla calego procesu MP
class task : public ecp_mp::task::task
{
	// robot class needs an access to the the Agent buffers
	friend class mp::robot::robot;

	// generator class needs an access to the UI command buffer
	friend class mp::generator::generator;

private:
	//! Start/Stop data buffer
	DataBuffer<char> ui_command_buffer;

	void initialize_communication(void);

	/// utworzenie robotow
	virtual void create_robots(void);

public:
	/// KONSTRUKTORY
	task(lib::configurator &_config);
	virtual ~task(void);

	void stop_and_terminate(void);

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

	// Wyslanie rozkazu do wszystkich ECP
	void execute_all(const common::robots_t & _robot_m);

	/// mapa wszystkich robotow
	common::robots_t robot_m;
};

task* return_created_mp_task(lib::configurator &_config);

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif /*MP_TASK_H_*/

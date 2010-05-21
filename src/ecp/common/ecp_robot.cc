#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

#ifdef __gnu_linux__
#include <execinfo.h>
#include <exception>
#include <iostream>
#endif /* __gnu_linux__ */

#include "ecp/common/ecp_robot.h"
#include "ecp/common/task/ecp_task.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

#include "lib/mis_fun.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// konstruktor wywolywany z UI
ecp_robot::ecp_robot(lib::robot_name_t _robot_name, int _number_of_servos,
		const std::string &_edp_section, lib::configurator &_config,
		lib::sr_ecp &_sr_ecp_msg) :
	robot(_robot_name),
	spawn_and_kill(true),
	communicate_with_edp(true),
	sr_ecp_msg(_sr_ecp_msg),
	number_of_servos(_number_of_servos),
	edp_section(_edp_section)
{
	connect_to_edp(_config);
}

// konstruktor wywolywany z ECP
ecp_robot::ecp_robot(lib::robot_name_t _robot_name, int _number_of_servos,
		const std::string &_edp_section, common::task::task& _ecp_object) :
	robot(_robot_name),
	spawn_and_kill(false),
	communicate_with_edp(true),
	sr_ecp_msg(*_ecp_object.sr_ecp_msg),
	number_of_servos(_number_of_servos),
	edp_section(_edp_section)
{
	connect_to_edp(_ecp_object.config);
}

// -------------------------------------------------------------------
ecp_robot::~ecp_robot(void) {
#if !defined(USE_MESSIP_SRR)
	if (EDP_fd > 0) {
		name_close(EDP_fd);
	}
#else /* USE_MESSIP_SRR */
	if (EDP_fd)
	{
		messip::port_disconnect(EDP_fd);
	}
#endif /* USE_MESSIP_SRR */

	if (spawn_and_kill) {
		if (kill(EDP_MASTER_Pid, SIGTERM) == -1) {
			perror("kill()");
		} else {
			//    		int status;
			//    		if (waitpid(EDP_MASTER_Pid, &status, 0) == -1) {
			//    			perror("waitpid()");
			//    		}
		}
	}
}

pid_t ecp_robot::get_EDP_pid(void) const {
	return EDP_MASTER_Pid;
}

ecp_robot::ECP_error::ECP_error(lib::error_class_t err_cl, uint64_t err_no,
		uint64_t err0, uint64_t err1) :
	error_class(err_cl), error_no(err_no) {
	error.error0 = err0;
	error.error1 = err1;
#ifdef __gnu_linux__
	void * array[25];
	int nSize = backtrace(array, 25);
	char ** symbols = backtrace_symbols(array, nSize);

	for (int i = 0; i < nSize; i++)
	{
		std::cerr << symbols[i] << std::endl;
	}

	free(symbols);
#endif /* __gnu_linux__ */
}

ecp_robot::ECP_main_error::ECP_main_error(lib::error_class_t err_cl,
		uint64_t err_no) :
	error_class(err_cl), error_no(err_no) {
}

bool ecp_robot::is_synchronised(void) const {
	// Czy robot zsynchronizowany?
	return synchronised;
}

// Kopiowanie bufora przesylanego z MP do bufora wysylanego do EDP
void ecp_robot::copy_mp_to_edp_buffer(lib::c_buffer& mp_buffer) {
	ecp_command.instruction = mp_buffer;
}

// by Y - o dziwo tego nie bylo !!!
// Kopiowanie bufora przesylanego z EDP do bufora wysylanego do MP
void ecp_robot::copy_edp_to_mp_buffer(lib::r_buffer& mp_buffer) {
	mp_buffer = reply_package;
}

// ---------------------------------------------------------------
void ecp_robot::connect_to_edp(lib::configurator &config) {
	EDP_MASTER_Pid = (spawn_and_kill) ? config.process_spawn(edp_section) : -1;

	std::string edp_net_attach_point = config.return_attach_point_name(
			lib::configurator::CONFIG_SERVER, "resourceman_attach_point",
			edp_section);

	printf("connect_to_edp");
	fflush(stdout);

	short tmp = 0;
#if !defined(USE_MESSIP_SRR)
	while ((EDP_fd = name_open(edp_net_attach_point.c_str(),
			NAME_FLAG_ATTACH_GLOBAL)) < 0)
#else
	while ((EDP_fd = messip::port_connect(edp_net_attach_point)) == NULL )
#endif
	{
		if ((tmp++) < CONNECT_RETRY) {
			usleep(1000* CONNECT_DELAY );
			printf(".");
			fflush(stdout);
		} else {
			int e = errno; // kod bledu systemowego
			fprintf(
					stderr,
					"Unable to locate EDP_MASTER process at channel \"%s\": %s\n",
					edp_net_attach_point.c_str(), strerror(errno));
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e,
					"Unable to locate EDP_MASTER process");
			throw ecp_robot::ECP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
		}
	}
	printf(".done\n");
}

void ecp_robot::synchronise(void) {
	// Zlecenie synchronizacji robota

	/*
	 // maskowanie sygnalu SIGTERM
	 // w celu zapobierzenia przerwania komunikacji ECP z EDP pomiedzy SET a QUERY - usuniete

	 sigset_t set;

	 sigemptyset( &set );
	 sigaddset( &set, SIGTERM );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
	// komunikacja wlasciwa
	ecp_command.instruction.instruction_type = lib::SYNCHRO;

	send(); // Wyslanie zlecenia synchronizacji
	query(); // Odebranie wyniku zlecenia

	synchronised = (reply_package.reply_type == lib::SYNCHRO_OK);

	/*
	 // odmaskowanie sygnalu SIGTERM
	 sigemptyset( &set );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
}

void ecp_robot::send()

{
	// Wyslanie do EDP polecenia
	// int command_size;  // rozmiar przesylanej przesylki
	// printf("\n a w send fd=%d",fd);  // debug

	// printf("w ECP send instruction.instruction_type: %d\n", ecp_command.instruction.instruction_type);

	switch (ecp_command.instruction.instruction_type) {
	case lib::SET:
	case lib::SET_GET:
	case lib::GET:
	case lib::SYNCHRO:
	case lib::QUERY:
	case lib::INVALID:
		// command_size = ((uint8_t*) (&instruction.address_byte)) - ((uint8_t*) (&instruction.instruction_type));
		// by Y bylo command_size zamiast sizeof(in..)
		// by Y&W doszlo dodatkowe pole w instruction zwiazane z obsluga resource managera

#if !defined(USE_MESSIP_SRR)
		if (MsgSend(EDP_fd, &ecp_command, sizeof(ecp_command), &reply_package,
				sizeof(lib::r_buffer)) == -1)
#else
		if ( messip::port_send(EDP_fd, 0, 0, ecp_command, reply_package) == -1 )
#endif
		{
			int e = errno; // kod bledu systemowego
			perror("ECP: Send to EDP_MASTER error");
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e,
					"ECP: Send to EDP_MASTER error");
			throw ecp_robot::ECP_error(lib::SYSTEM_ERROR, 0);
		}
		//printf("sizeof(lib::r_buffer) = %d\n", sizeof(lib::r_buffer));

		break;
	default: // blad: nieprawidlowe polecenie
		perror("ECP: INVALID COMMAND TO EDP");
		sr_ecp_msg.message(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
		throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
	}

	// TODO: this is called much too often (?!)
	lib::set_thread_priority(pthread_self(), MAX_PRIORITY - 2);
}

void ecp_robot::create_command() {
}

void ecp_robot::get_reply() {
}

void ecp_robot::clear_data_ports() {
}

void ecp_robot::query()

{
	ecp_command.instruction.instruction_type = lib::QUERY;
	send(); // czyli wywolanie funkcji ecp_buffer::send, ktora jest powyzej :)
}

void ecp_robot::execute_motion(void) {
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	/*
	 // maskowanie sygnalu SIGTERM
	 // w celu zapobierzenia przerwania komunikacji ECP z EDP pomiedzy SET a QUERY - usuniete

	 sigset_t set;

	 sigemptyset( &set );
	 sigaddset( &set, SIGTERM );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
	// komunikacja wlasciwa
	send();
	if (reply_package.reply_type == lib::ERROR) {
		query();
		throw ECP_error(lib::NON_FATAL_ERROR, EDP_ERROR);
	}
	query();

	/*
	 // odmaskowanie sygnalu SIGTERM

	 sigemptyset( &set );

	 if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1)
	 {
	 printf ("blad w ECP procmask signal\n");
	 }
	 */
	if (reply_package.reply_type == lib::ERROR) {
		throw ECP_error(lib::NON_FATAL_ERROR, EDP_ERROR);
	}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


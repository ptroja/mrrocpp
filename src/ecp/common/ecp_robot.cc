#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>


#include <stdint.h>



#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

#ifdef __LINUX__
#include <execinfo.h>
#include <exception>
#include <iostream>
#endif /* __LINUX__ */

#include "ecp/common/ecp_robot.h"
#include "ecp/common/ecp_task.h"

#include "lib/mis_fun.h"

// konstruktor wywolywany z UI
ecp_robot::ecp_robot(ROBOT_ENUM _robot_name, configurator &_config, sr_ecp *_sr_ecp_msg) :
	ecp_mp_robot(_robot_name), spawn_and_kill(true)
{
	sr_ecp_msg = _sr_ecp_msg;

	connect_to_edp(_config);
}

// konstruktor wywolywany z ECP
ecp_robot::ecp_robot(ROBOT_ENUM _robot_name, ecp_task& _ecp_object) :
	ecp_mp_robot(_robot_name), spawn_and_kill(false)
{
	sr_ecp_msg = _ecp_object.sr_ecp_msg;

	connect_to_edp(_ecp_object.config);
}

// -------------------------------------------------------------------
ecp_robot::~ecp_robot(void)
{
#if !defined(USE_MESSIP_SRR)
    if (EDP_fd > 0)
    {
        name_close(EDP_fd);
    }
#else /* USE_MESSIP_SRR */
    if (EDP_fd)
    {
        messip_channel_disconnect(EDP_fd, MESSIP_NOTIMEOUT);
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

pid_t ecp_robot::get_EDP_pid(void) const
{
    return EDP_MASTER_Pid;
}

ecp_robot::ECP_error::ECP_error ( uint64_t err_cl, uint64_t err_no,
                                  uint64_t err0, uint64_t err1 )
        : error_class(err_cl), error_no(err_no)
{
    error.error0 = err0;
    error.error1 = err1;
#ifdef __LINUX__
    void * array[25];
    int nSize = backtrace(array, 25);
    char ** symbols = backtrace_symbols(array, nSize);

    for (int i = 0; i < nSize; i++)
    {
        std::cout << symbols[i] << std::endl;
    }

    free(symbols);
#endif /* __LINUX__ */
}

ecp_robot::ECP_main_error::ECP_main_error ( uint64_t err_cl, uint64_t err_no)
        : error_class(err_cl), error_no(err_no)
{}

bool ecp_robot::is_synchronised ( void ) const
{
    // Czy robot zsynchronizowany?
    return synchronised;
}


// Kopiowanie bufora przesylanego z MP do bufora wysylanego do EDP
void ecp_robot::copy_mp_to_edp_buffer(c_buffer& mp_buffer)
{
	memcpy( &ecp_command.instruction, &mp_buffer, sizeof(c_buffer));
}

// by Y - o dziwo tego nie bylo !!!
// Kopiowanie bufora przesylanego z EDP do bufora wysylanego do MP
void ecp_robot::copy_edp_to_mp_buffer(r_buffer& mp_buffer)
{
	memcpy( &mp_buffer, &reply_package, sizeof(r_buffer));
}


// ---------------------------------------------------------------
void ecp_robot::connect_to_edp(configurator &config)
{
	const char *edp_section;

    // name of the edp_section depends on _robot_name
	switch (robot_name) {
		case ROBOT_IRP6_ON_TRACK:
			edp_section = "[edp_irp6_on_track]";
			number_of_servos = IRP6_ON_TRACK_NUM_OF_SERVOS;
			break;
		case ROBOT_IRP6_POSTUMENT:
			edp_section = "[edp_irp6_postument]";
			number_of_servos = IRP6_POSTUMENT_NUM_OF_SERVOS;
			break;
		case ROBOT_CONVEYOR:
			edp_section = "[edp_conveyor]";
			number_of_servos = CONVEYOR_NUM_OF_SERVOS;
			break;
		case ROBOT_SPEAKER:
			edp_section = "[edp_speaker]";
			break;
		case ROBOT_IRP6_MECHATRONIKA:
			edp_section = "[edp_irp6_mechatronika]";
			number_of_servos = IRP6_MECHATRONIKA_NUM_OF_SERVOS;
			break;
		default:
			fprintf(stderr, "ERROR: unknown robot name in ecp_robot constructor\n");
			edp_section = NULL;
			break;
	}

	EDP_MASTER_Pid = (spawn_and_kill) ? config.process_spawn(edp_section) : -1;

	const char* edp_net_attach_point =
			config.return_attach_point_name(configurator::CONFIG_SERVER, "resourceman_attach_point", edp_section);

	printf("connect_to_edp");
	fflush(stdout);

	short tmp = 0;
#if !defined(USE_MESSIP_SRR)
	while ((EDP_fd = name_open(edp_net_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0)
#else
	while ((EDP_fd = messip_channel_connect(NULL, edp_net_attach_point, MESSIP_NOTIMEOUT)) == NULL )
#endif
	{
		if ((tmp++)<CONNECT_RETRY) {
			usleep(1000*CONNECT_DELAY);
			printf(".");
			fflush(stdout);
		} else {
			int e = errno; // kod bledu systemowego
			fprintf(stderr, "Unable to locate EDP_MASTER process at channel \"%s\": %s\n", edp_net_attach_point, sys_errlist[errno]);
			sr_ecp_msg->message(SYSTEM_ERROR, e, "Unable to locate EDP_MASTER process");
			throw ecp_robot::ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
		}
	}
	printf(".done\n");

	delete [] edp_net_attach_point;
}

void ecp_robot::synchronise(void)
{
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
	ecp_command.instruction.instruction_type = SYNCHRO;

	send(); // Wyslanie zlecenia synchronizacji
	query(); // Odebranie wyniku zlecenia

	synchronised = (reply_package.reply_type == SYNCHRO_OK);

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
		case SET:
		case SET_GET:
		case GET:
		case SYNCHRO:
		case QUERY:
		case INVALID:
			// command_size = ((BYTE*) (&instruction.address_byte)) - ((BYTE*) (&instruction.instruction_type));
			// by Y bylo command_size zamiast sizeof(in..)
			// by Y&W doszlo  dodatkowe pole w instruction zwiazane z obsluga resource managera

#if !defined(USE_MESSIP_SRR)
			if (MsgSend(EDP_fd, &ecp_command, sizeof(ecp_command), &reply_package, sizeof(r_buffer)) == -1)
#else
			int32_t answer;
			if ( messip_send(EDP_fd, 0, 0,
							&ecp_command, sizeof(ecp_command),
							&answer, &reply_package, sizeof(r_buffer),
							MESSIP_NOTIMEOUT) == -1 )
#endif
			{
				uint64_t e= errno; // kod bledu systemowego
				perror("Send error to EDP_MASTER");
				sr_ecp_msg->message(SYSTEM_ERROR, e, "Send error to EDP_MASTER");
				throw ecp_robot::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
			}
			//printf("sizeof(r_buffer) = %d\n", sizeof(r_buffer));

			break;
		default: // blad: nieprawidlowe polecenie
			perror("ECP: INVALID COMMAND TO EDP\n");
			sr_ecp_msg->message(NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
			throw ecp_robot::ECP_error(NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
	}

	set_thread_priority(pthread_self(), MAX_PRIORITY-2);

}


void ecp_robot::query()

{
	ecp_command.instruction.instruction_type = QUERY;
	send(); // czyli wywolanie funkcji ecp_buffer::send, ktora jest powyzej :)
}



void ecp_robot::execute_motion(void)
{
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
	if (reply_package.reply_type == ERROR) {
		query();
		throw ECP_error (NON_FATAL_ERROR, EDP_ERROR);
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
	if (reply_package.reply_type == ERROR) {
		throw ECP_error (NON_FATAL_ERROR, EDP_ERROR);
	}
}

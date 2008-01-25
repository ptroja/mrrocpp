#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#include "ecp/common/ecp_robot.h"
#include "ecp/common/ecp_task.h"

robot_EDP_transmission_data::robot_EDP_transmission_data (void)
{
	instruction_type = INVALID;
	reply_type = ACKNOWLEDGE;
} // konstruktor

// -------------------------------------------------------------------
// Konstruktor klasy robot
// -------------------------------------------------------------------

// konstruktor wywolywany z UI
ecp_robot::ecp_robot(ROBOT_ENUM _robot_name, configurator &_config, sr_ecp *_sr_ecp_msg)
	:
	robot_name(_robot_name)
{
	char *edp_section;

	communicate = true; // domyslnie robot jest aktywny
	EDP_command_and_reply_buffer.sr_ecp_msg = _sr_ecp_msg;

	// name of the edp_section depends on _robot_name
	switch (_robot_name) {
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
			break;
		case ROBOT_SPEAKER:
			edp_section = "[edp_speaker]";
			break;
		case ROBOT_IRP6_MECHATRONIKA:
			edp_section = "[edp_irp6_mechatronika]";
			number_of_servos = IRP6_MECHATRONIKA_NUM_OF_SERVOS;
			break;
		default:
			printf("ERROR: unknown robot name in ecp_robot constructor\n");
			edp_section = NULL;
			break;
	}

	EDP_MASTER_Pid = _config.process_spawn(edp_section);

	if (edp_section) connect_to_edp (_config.return_attach_point_name
	                (configurator::CONFIG_SERVER, 	"resourceman_attach_point",	edp_section));

}

// konstruktor wywolywany z ECP
ecp_robot::ecp_robot(ROBOT_ENUM _robot_name, ecp_task& _ecp_object)
	:
	robot_name(_robot_name)
{
	char *edp_section;

	communicate = true; // domyslnie robot jest aktywny
	EDP_command_and_reply_buffer.sr_ecp_msg = _ecp_object.sr_ecp_msg;

	// name of the edp_section depends on _robot_name
	switch (_robot_name) {
		case ROBOT_IRP6_ON_TRACK:
			edp_section = "[edp_irp6_on_track]";
			break;
		case ROBOT_IRP6_POSTUMENT:
			edp_section = "[edp_irp6_postument]";
			break;
		case ROBOT_CONVEYOR:
			edp_section = "[edp_conveyor]";
			break;
		case ROBOT_SPEAKER:
			edp_section = "[edp_speaker]";
			break;
		case ROBOT_IRP6_MECHATRONIKA:
			edp_section = "[edp_irp6_mechatronika]";
			break;
		default:
			printf("ERROR: unknown robot name in ecp_robot constructor\n");
			edp_section = NULL;
			break;
	}

	if (edp_section) connect_to_edp (_ecp_object.config->return_attach_point_name
	                (configurator::CONFIG_SERVER, 	"resourceman_attach_point",	edp_section));

}
 
// -------------------------------------------------------------------
ecp_robot::~ecp_robot(void)
{
#if !defined(USE_MESSIP_SRR)
	if (EDP_fd > 0) {
		name_close(EDP_fd);
	}
#else /* USE_MESSIP_SRR */
	if (EDP_fd) {
		messip_channel_disconnect(EDP_fd, MESSIP_NOTIMEOUT);
	}
#endif /* USE_MESSIP_SRR */
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
void ecp_robot::copy_mp_to_edp_buffer (c_buffer& mp_buffer)
{
	memcpy( &EDP_command_and_reply_buffer.instruction, &mp_buffer, sizeof(c_buffer));
}

// by Y - o dziwo tego nie bylo !!!
// Kopiowanie bufora przesylanego z EDP do bufora wysylanego do MP
void ecp_robot::copy_edp_to_mp_buffer (r_buffer& mp_buffer)
{
	memcpy( &mp_buffer, &EDP_command_and_reply_buffer.reply_package, sizeof(r_buffer));
}


// ---------------------------------------------------------------
void ecp_robot::connect_to_edp (const char* edp_net_attach_point)
{
	uint64_t e;     // kod bledu systemowego
	short tmp = 0;

	printf("connect_to_edp..."); fflush (stdout);

	// kilka sekund  (~5) na otworzenie urzadzenia
#if !defined(USE_MESSIP_SRR)
	while ((EDP_fd = name_open( edp_net_attach_point, NAME_FLAG_ATTACH_GLOBAL)) <0 ) {
#else
	while ((EDP_fd = messip_channel_connect(NULL, edp_net_attach_point, MESSIP_NOTIMEOUT)) == NULL ) {
#endif
		if ((tmp++)<CONNECT_RETRY) {
			usleep(1000*CONNECT_DELAY);
			printf(","); fflush(stdout);
		} else {
			e = errno;
			perror("Unable to locate EDP_MASTER process\n");
			EDP_command_and_reply_buffer.sr_ecp_msg->message (SYSTEM_ERROR, e, "Unable to locate EDP_MASTER process");
			throw ecp_robot::ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
		}
	}
	printf("done\n");

}
// -------------------------------------------------------------------


// ---------------------------------------------------------------
void ecp_robot::synchronise ( void )
{
// Zlecenie synchronizacji robota
//  printf("\n synchronizacja fd=%d",fd);  // debug

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
	EDP_command_and_reply_buffer.instruction.instruction_type = SYNCHRO;
	EDP_command_and_reply_buffer.send(EDP_fd);  // Wyslanie zlecenia synchronizacji
	EDP_command_and_reply_buffer.query(EDP_fd); // Odebranie wyniku zlecenia
	if (EDP_command_and_reply_buffer.reply_package.reply_type == SYNCHRO_OK) {
		synchronised = true;
	} else {
		synchronised = false;
	}
	/*
	// odmaskowanie sygnalu SIGTERM 
	  	sigemptyset( &set );

	if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1) 
	{
	printf ("blad w ECP procmask signal\n");
	}
	*/
} // end: irp6_on_track_robot::synchronise ()
// ---------------------------------------------------------------



// ---------------------------------------------------------------
// virtual // wywalam Y&W - zmiana w kompilatorze
void ecp_robot::execute_motion (void)
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
	EDP_command_and_reply_buffer.send(EDP_fd);
	if (EDP_command_and_reply_buffer.reply_package.reply_type == ERROR) {
		EDP_command_and_reply_buffer.query(EDP_fd);
		throw ECP_error (NON_FATAL_ERROR, EDP_ERROR);
	}
	EDP_command_and_reply_buffer.query(EDP_fd);

	/*
	// odmaskowanie sygnalu SIGTERM 

	sigemptyset( &set );

	if  (sigprocmask( SIG_SETMASK, &set, NULL)==-1) 
	{
	printf ("blad w ECP procmask signal\n");
	}
	*/
	if (EDP_command_and_reply_buffer.reply_package.reply_type == ERROR) {
		throw ECP_error (NON_FATAL_ERROR, EDP_ERROR);
	}

} // end: irp6_on_track_robot::execute_motion (void)
/*---------------------------------------------------------------------*/

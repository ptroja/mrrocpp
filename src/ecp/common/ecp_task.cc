#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <stdio.h>

#include "lib/mis_fun.h"
#include "ecp/common/ecp_task.h"
#include "ecp/common/ECP_main_error.h"
#include "ecp/common/ecp_teach_in_generator.h"

ecp_task::ecp_task(configurator &_config)
	: ecp_mp_task(_config)
{
    sensor_m.clear();
}

ecp_task::~ecp_task()
{
    delete[] mrrocpp_network_path;
}

bool ecp_task::pulse_check()
{
    struct sigevent stop_event;

    _pulse_msg ui_msg; // wiadomosc z ui

    stop_event.sigev_notify = SIGEV_UNBLOCK;// by Y zamiast creceive
    TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE,  &stop_event, NULL, NULL ); // czekamy na odbior pulsu stopu
    int rcvid = MsgReceive(trigger_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

    if (rcvid == -1)
    {/* Error condition, exit */
        // perror("blad receive w reader\n");

    }

    if (rcvid == 0)
    {/* Pulse received */
        switch (ui_msg.hdr.code)
        {
        case _PULSE_CODE_DISCONNECT:
            /*
             * A client disconnected all its connections (called
             * name_close() for each name_open() of our name) or
             * terminated
             */
            ConnectDetach(ui_msg.hdr.scoid);
            break;
        case _PULSE_CODE_UNBLOCK:
            /*
             * REPLY blocked client wants to unblock (was hit by a signal or timed out).  It's up to you if you
             * reply now or later.
             */
            break;
        default:
            if (ui_msg.hdr.code== ECP_TRIGGER )
            { // odebrano puls ECP_TRIGGER
                return true;
            }
            /*
            * A pulse sent by one of your processes or a _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
            * from the kernel?
            */
        }
    }

    if (rcvid > 0)
    {
        /* A QNX IO message received, reject */
        if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX)
        {
            MsgReply(rcvid, EOK, 0, 0);
        }
        else
        {
            /* A message (presumable ours) received, handle */
            printf("ECP trigger server receive strange message of type: %d\n", ui_msg.data);
            MsgReply(rcvid, EOK, 0, 0);
        }
    }

    return false;
}
// ---------------------------------------------------------------



void ecp_task::catch_signal_in_ecp_task(int sig)
{
	switch (sig)
	{
		case SIGTERM :
			kill_all_VSP (sensor_m);
			sr_ecp_msg->message ("ECP terminated");
			_exit (EXIT_SUCCESS);
			break;
		case SIGSEGV:
			fprintf(stderr, "\n\nSegmentation fault in ECP process\n\n");
			signal(SIGSEGV, SIG_DFL);
			break;
	}
}


void ecp_task::terminate ( )
{}


// ---------------------------------------------------------------
void ecp_task::initialize_communication ()
{
    uint64_t e;     // kod bledu systemowego

    char* sr_net_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
    char* ecp_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "ecp_attach_point");
    char* trigger_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "trigger_attach_point");
    char* mp_pulse_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "mp_pulse_attach_point", "[mp]");

    if (( sr_ecp_msg = new sr_ecp(ECP, ecp_attach_point, sr_net_attach_point)) == NULL)
    { // Obiekt do komuniacji z SR
        perror ( "Unable to locate SR\n");
        throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
    }

    // Lokalizacja procesu MP - okreslenie identyfikatora (pid)
    if ( (MP_fd = name_open(mp_pulse_attach_point, NAME_FLAG_ATTACH_GLOBAL))  < 0 )
    {
        e = errno;
        perror("ECP: Unable to locate MP_MASTER process\n");
        throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
    }

    // Rejstracja procesu ECP
    if ((ecp_attach = name_attach(NULL, ecp_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL)
    {
        e = errno;
        perror("Failed to attach Effector Control Process\n");
        sr_ecp_msg->message (SYSTEM_ERROR, e, "Failed to attach Effector Control Process");
        throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
    }

    if ((trigger_attach = name_attach(NULL, trigger_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL)
    {
        e = errno;
        perror("Failed to attach TRIGGER pulse chanel for ecp\n");
        sr_ecp_msg->message (SYSTEM_ERROR, e, "Failed  Failed to name attach (trigger pulse)");
        throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
    }

    delete [] ecp_attach_point;
    delete [] sr_net_attach_point;
    delete [] trigger_attach_point;
    delete [] mp_pulse_attach_point;

}
// -------------------------------------------------------------------


// methods for ECP template to redefine in concrete classes
void ecp_task::task_initialization(void)
{}

void ecp_task::main_task_algorithm(void)
{}

void ecp_task::Move (ecp_generator& the_generator)
{
	// Funkcja ruchu dla ECP

	// generacja pierwszego kroku ruchu
	the_generator.node_counter = 0;
	set_ecp_reply (ECP_ACKNOWLEDGE);

	if (!mp_buffer_receive_and_send() || !the_generator.first_step())
	{
		return; // Warunek koncowy spelniony w pierwszym kroku
	}

	do
	{ // realizacja ruchu

		// zadanie przygotowania danych od czujnikow
		all_sensors_initiate_reading (the_generator.sensor_m);

		// wykonanie kroku ruchu
		if ((the_generator.the_robot) && the_generator.the_robot->communicate)
		{
			the_generator.the_robot->create_command ();
			// zlecenie ruchu SET oraz odczyt stanu robota GET
			the_generator.the_robot->execute_motion();
			the_generator.the_robot->get_reply();
		}

		// odczytanie danych z wszystkich czujnikow
		all_sensors_get_reading(the_generator.sensor_m);
		the_generator.node_counter++;
	}
	while (mp_buffer_receive_and_send() && the_generator.next_step());
}

// Przekazanie identyfikatora procesu MP
void ecp_task::set_mp_pid (pid_t mp_pid)
{
    MP_pid = mp_pid;
}

// Badanie typu polecenia z MP
MP_COMMAND ecp_task::mp_command_type (void) const
{
    return mp_command.command;
}

// Ustawienie typu odpowiedzi z ECP do MP
void ecp_task::set_ecp_reply ( ECP_REPLY ecp_r)
{
    ecp_reply.reply = ecp_r;
}

// Informacja dla MP o zakonczeniu zadania uzytkownika
void ecp_task::ecp_termination_notice (void)
{
    set_ecp_reply (TASK_TERMINATED);
    mp_buffer_receive_and_send ();
}

// Wysyla puls do Mp przed oczekiwaniem na spotkanie
void ecp_task::send_pulse_to_mp (char pulse_code, long pulse_value)
{
    if (MsgSendPulse (MP_fd , sched_get_priority_min(SCHED_FIFO),  pulse_code,  pulse_value)==-1)
    {
        perror("Blad w wysylaniu pulsu do mp\n");
    }
}

// Petla odbierania wiadomosci.
void ecp_task::ecp_wait_for_stop (void)
{
    // Wyslanie pulsu do MP
    send_pulse_to_mp ( ECP_WAIT_FOR_STOP, 1);

    // Oczekiwanie na wiadomosc.
    int caller = receive_mp_message();

    if (mp_command_type() == STOP)
    {
    	set_ecp_reply (ECP_ACKNOWLEDGE);
    }
    else
    {
    	set_ecp_reply (ERROR_IN_ECP);
    }

    // Wyslanie odpowiedzi.
    if ( MsgReply(caller, EOK, &ecp_reply, sizeof(ecp_reply)) ==-1)
    {// by Y&W
        uint64_t e = errno; // kod bledu systemowego
        perror("ECP: Reply to MP failed\n");
        sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Reply to MP failed");
        throw ecp_generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
    }

    if (mp_command_type() != STOP)
    {
        fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
                __FILE__, __LINE__);
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
    }
}

// Oczekiwanie na polecenie START od MP
bool ecp_task::ecp_wait_for_start (void)
{
    bool ecp_stop = false;

    // Wyslanie pulsu do MP
    send_pulse_to_mp ( ECP_WAIT_FOR_START, 1);

    int caller = receive_mp_message();

    switch (mp_command_type() )
    {
    	case START_TASK:
    		// by Y - ECP_ACKNOWLEDGE zamienione na TASK_TERMINATED w celu uproszczenia oprogramowania zadan wielorobotowych
    		set_ecp_reply (TASK_TERMINATED);
    		break;
    	case STOP:
    		set_ecp_reply (TASK_TERMINATED);
    		ecp_stop = true;
    		break;
    	default:
    		set_ecp_reply (INCORRECT_MP_COMMAND);
    		break;
    }

    // if (Reply (caller, &ecp_reply, sizeof(ECP_REPLY_PACKAGE)) == -1 ) {
    if ( MsgReply(caller, EOK, &ecp_reply, sizeof(ECP_REPLY_PACKAGE)) ==-1)
    {// by Y&W
        uint64_t e = errno; // kod bledu systemowego
        perror("ECP: Reply to MP failed\n");
        sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Reply to MP failed");
        throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
    }
    if (ecp_stop)
        throw ecp_generator::ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

    if (ecp_reply.reply == INCORRECT_MP_COMMAND)
    {
        fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
                __FILE__, __LINE__);
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
    }

    sr_ecp_msg->message("ECP user program is running");
    return false;

}

// Oczekiwanie na polecenie START od MP
bool ecp_task::get_next_state (void)
{
    bool ecp_stop = false;

    // Wyslanie pulsu do MP
    send_pulse_to_mp ( ECP_WAIT_FOR_NEXT_STATE, 1);

    int caller = receive_mp_message();

    switch (mp_command_type() )
    {
    	case NEXT_STATE:
    		set_ecp_reply (ECP_ACKNOWLEDGE);
    		break;
    	case STOP:
    		set_ecp_reply (ECP_ACKNOWLEDGE);
    		ecp_stop = true;
    		break;
    	default:
    		set_ecp_reply (INCORRECT_MP_COMMAND);
    		break;
    }

    if ( MsgReply(caller, EOK, &ecp_reply, sizeof(ECP_REPLY_PACKAGE)) ==-1)
    {// by Y&W
        uint64_t e = errno; // kod bledu systemowego
        perror("ECP: Reply to MP failed\n");
        sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Reply to MP failed");
        throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
    }

    if (ecp_stop)
        throw ecp_generator::ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

    if (ecp_reply.reply == INCORRECT_MP_COMMAND)
    {
        fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
                __FILE__, __LINE__);
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
    }

    return false;

}

// Oczekiwanie na polecenie od MP
bool ecp_task::mp_buffer_receive_and_send (void)
{
	
//	printf("mp_buffer_receive_and_send\n");
	
    bool returned_value = true;
    bool ecp_stop = false;
    // Wyslanie pulsu do MP
    send_pulse_to_mp ( ECP_WAIT_FOR_COMMAND, 1);

    int caller = receive_mp_message();

    switch (mp_command_type() )
    {
    	case NEXT_POSE:
    		if ((ecp_reply.reply != TASK_TERMINATED)&&(ecp_reply.reply != ERROR_IN_ECP))
    			set_ecp_reply (ECP_ACKNOWLEDGE);
    		break;
    	case STOP:
    		set_ecp_reply (ECP_ACKNOWLEDGE);
    		ecp_stop = true;
    		break;
    	case END_MOTION:
    		// dla ulatwienia programowania apliakcji wielorobotowych
    		if (ecp_reply.reply != ERROR_IN_ECP)
    			set_ecp_reply (TASK_TERMINATED);
    		returned_value = false;
    		break;
    	default:
    		set_ecp_reply (INCORRECT_MP_COMMAND);
    		break;
    }

    if ( MsgReply(caller, EOK, &ecp_reply, sizeof(ecp_reply)) ==-1)
    {// by Y&W
        uint64_t e = errno; // kod bledu systemowego
        perror("ECP: Reply to MP failed\n");
        sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Reply to MP failed");
        throw ecp_robot::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
    }


    if (ecp_stop)
        throw ecp_generator::ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

    if (ecp_reply.reply == INCORRECT_MP_COMMAND)
    {
        fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
                __FILE__, __LINE__);
        throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
    }

    return returned_value;

}



// Receive of mp message
int ecp_task::receive_mp_message (void)
{
    while (1)
    {

        int caller = MsgReceive(ecp_attach->chid, &mp_command, sizeof(mp_command), NULL);

        if (caller == -1)
        {/* Error condition, exit */
            uint64_t e = errno; // kod bledu systemowego
            perror("ECP: Receive from MP failed\n");
            sr_ecp_msg->message (SYSTEM_ERROR, e, "ECP: Receive from MP failed");
            throw ecp_robot::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
        }

        if (caller == 0)
        {/* Pulse received */
            switch (mp_command.hdr.code)
            {
            case _PULSE_CODE_DISCONNECT:
                /*
                 * A client disconnected all its connections (called
                 * name_close() for each name_open() of our name) or
                 * terminated
                 */
                ConnectDetach(mp_command.hdr.scoid);
                break;
            case _PULSE_CODE_UNBLOCK:
                /*
                 * REPLY blocked client wants to unblock (was hit by
                 * a signal or timed out).  It's up to you if you
                 * reply now or later.
                 */
                break;
            default:
                /*
                * A pulse sent by one of your processes or a
                * _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
                * from the kernel?
                */
                break;
            }
            continue;
        }

        /* A QNX IO message received, reject */
        if (mp_command.hdr.type >= _IO_BASE && mp_command.hdr.type <= _IO_MAX)
        {
            MsgReply(caller, EOK, 0, 0);
            continue;
        }

        return caller;
    }
}

BYTE ecp_task::convert (POSE_SPECIFICATION ps)
{
	switch (ps)
	{
		case MOTOR:
			return C_MOTOR;
		case JOINT:
			return C_JOINT;
		case XYZ_ANGLE_AXIS:
			return C_XYZ_ANGLE_AXIS;
		case XYZ_EULER_ZYZ:
			return C_XYZ_EULER_ZYZ;
		default:
			return C_MOTOR;
	}
	return C_MOTOR;
}

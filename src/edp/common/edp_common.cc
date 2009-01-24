// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>

#include "lib/mis_fun.h"
#include "edp/common/edp.h"

#include "messip/messip.h"



/*--------------------------------------------------------------------------*/
edp_effector::edp_effector(configurator &_config, ROBOT_ENUM l_robot_name) :
	config(_config), robot_name(l_robot_name)
{

	/* Lokalizacja procesu wywietlania komunikatow SR */

	if ((msg = new sr_edp(EDP, config.return_string_value("resourceman_attach_point"),
			config.return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]")))
			== NULL) {
		perror("Unable to locate SR ");
		throw System_error();
	}

	if (config.exists("test_mode"))
		test_mode = config.return_int_value("test_mode");
	else
		test_mode = 0;
	mrrocpp_network_path = config.return_mrrocpp_network_path();

}

void edp_effector::check_config(const char* string, uint8_t* input)
{
	if ((config.exists(string))&&(config.return_int_value(string)))
		*input=1;
	else
		*input=0;
}

/*--------------------------------------------------------------------------*/
bool edp_effector::initialize_communication()
{
	char* server_attach_point;
	server_attach_point
			= config.return_attach_point_name(configurator::CONFIG_SERVER, "resourceman_attach_point");

#if !defined(USE_MESSIP_SRR)
	// obsluga mechanizmu sygnalizacji zajetosci sprzetu
	if (!(test_mode)) {
		char* hardware_busy_attach_point;
		char full_path_to_hardware_busy_attach_point[100];
		name_attach_t *tmp_attach;

		hardware_busy_attach_point
				= config.return_string_value("hardware_busy_attach_point");
		sprintf(full_path_to_hardware_busy_attach_point, "/dev/name/global/%s", hardware_busy_attach_point);

		// sprawdzenie czy nie jakis proces EDP nie zajmuje juz sprzetu
		if (access(full_path_to_hardware_busy_attach_point, R_OK)== 0) {
			fprintf( stderr, "EDP: hardware busy\n");

			delete[] hardware_busy_attach_point;

			return false;
		}

		tmp_attach
				= name_attach(NULL, hardware_busy_attach_point, NAME_FLAG_ATTACH_GLOBAL);

		if (tmp_attach == NULL) {
			msg->message(SYSTEM_ERROR, errno, "EDP: hardware_busy_attach_point failed to attach");
			fprintf( stderr, "hardware_busy_attach_point name_attach() failed: %s\n", strerror( errno ));

			delete[] hardware_busy_attach_point;

			return false;
		}

		delete[] hardware_busy_attach_point;
	}
#endif /* !defined(USE_MESSIP_SRR */

	char full_path_to_server_attach_point[100];
	sprintf(full_path_to_server_attach_point, "/dev/name/global/%s", server_attach_point);

	// sprawdzenie czy nie jest juz zarejestrowany server EDP
	if (access(full_path_to_server_attach_point, R_OK)== 0) {
		fprintf( stderr, "edp already exists() failed: %s\n", strerror( errno ));
		return false;
	}

	/* Ustawienie priorytetu procesu */

	set_thread_priority(pthread_self() , MAX_PRIORITY-2);

	attach =
#if !defined(USE_MESSIP_SRR)
		name_attach(NULL, server_attach_point, NAME_FLAG_ATTACH_GLOBAL);
#else /* USE_MESSIP_SRR */
		messip_channel_create(NULL, server_attach_point, MESSIP_NOTIMEOUT, 0);
#endif /* USE_MESSIP_SRR */

	delete [] server_attach_point;

	if (attach == NULL) {
		msg->message(SYSTEM_ERROR, errno, "EDP: resmg failed to attach");
		fprintf( stderr, "name_attach() failed: %s\n", strerror( errno ));
		return false;
	}

	msg->message("EDP loaded");

	return true;
}


void edp_effector::insert_reply_type(REPLY_TYPE rt)
{
	reply.reply_type = rt;
}

bool edp_effector::is_reply_type_ERROR() const
{
	return (reply.reply_type==ERROR);
}

void edp_effector::main_loop()
{
}

void edp_effector::establish_error(uint64_t err0, uint64_t err1)
{
	reply.reply_type = ERROR;
	reply.error_no.error0 = err0;
	reply.error_no.error1 = err1;
}

// r_buffer
REPLY_TYPE edp_effector::is_reply_type(void) const
{
	return reply.reply_type;
}

uint64_t edp_effector::is_error_no_0(void) const
{
	return reply.error_no.error0;
}

uint64_t edp_effector::is_error_no_1(void) const
{
	return reply.error_no.error1;
}


INSTRUCTION_TYPE edp_effector::receive_instruction(void)
{
	// oczekuje na polecenie od ECP, wczytuje je oraz zwraca jego typ
	int rcvid;
	/* Oczekiwanie na polecenie od ECP */

	/* Do your MsgReceive's here now with the chid */
	while (1) {
#if !defined(USE_MESSIP_SRR)
		rcvid
				= MsgReceive(attach->chid, &new_instruction, sizeof(c_buffer), NULL);

		if (rcvid == -1) {/* Error condition, exit */
			perror("MsgReceive()");
			break;
		}

		if (rcvid == 0) {/* Pulse received */
			switch (new_instruction.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					/*
					 * A client disconnected all its connections (called
					 * name_close() for each name_open() of our name) or
					 * terminated
					 */
					ConnectDetach(new_instruction.hdr.scoid);
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

		/* name_open() sends a connect message, must EOK this */
		if (new_instruction.hdr.type == _IO_CONNECT) {
			MsgReply(rcvid, EOK, NULL, 0);
			continue;
		}

		/* Some other QNX IO message was received; reject it */
		if (new_instruction.hdr.type > _IO_BASE && new_instruction.hdr.type
				<= _IO_MAX) {
			MsgError(rcvid, ENOSYS);
			continue;
		}
#else /* USE_MESSIP_SRR */
		int32_t type, subtype;
		rcvid = messip_receive(attach, &type, &subtype, &new_instruction, sizeof(c_buffer), MESSIP_NOTIMEOUT);

		if (rcvid == -1)
		{/* Error condition, exit */
			perror("messip_receive()");
			break;
		}
		else if (rcvid < -1)
		{
			fprintf(stderr, "ie. MESSIP_MSG_DISCONNECT\n");
			continue;
		}
#endif /* USE_MESSIP_SRR */

		/* A message (presumable ours) received, handle */
		break;
	}

	//memcpy( &new_instruction, msg_cb, sizeof(*msg_cb) );
	caller = rcvid;

	return new_instruction.instruction_type;
}

void edp_effector::reply_to_instruction(void)
{
	// Wyslanie potwierdzenia przyjecia polecenia do wykonania,
	// adekwatnej odpowiedzi na zapytanie lub
	// informacji o tym, ze przyslane polecenie nie moze byc przyjte
	// do wykonania w aktualnym stanie EDP
	// int reply_size;     // liczba bajtw wysyanej odpowiedzi
	if ( !( (reply.reply_type == ERROR) || (reply.reply_type == SYNCHRO_OK) ))
		reply.reply_type = real_reply_type;
#if !defined(USE_MESSIP_SRR)
	if (MsgReply(caller, 0, &reply, sizeof(reply)) == -1) { // Odpowiedz dla procesu ECP badz UI by Y
#else /* USE_MESSIP_SRR */
	if (messip_reply(attach, caller, 0, &reply, sizeof(reply), MESSIP_NOTIMEOUT) == -1) {
#endif /* USE_MESSIP_SRR */
		uint64_t e= errno;
		perror("Reply() to ECP failed");
		msg->message(SYSTEM_ERROR, e, "Reply() to ECP failed");
		throw System_error();
	}
	real_reply_type = ACKNOWLEDGE;
}

reader_buffer::reader_buffer()
{
	pthread_mutex_init(&reader_mutex, NULL);

	sem_init(&reader_sem, 0, 0);
}

int reader_buffer::set_new_step() // podniesienie semafora
{
	sem_trywait(&(reader_sem));
	return sem_post(&reader_sem);// odwieszenie watku edp_master
}

int reader_buffer::reader_wait_for_new_step() // oczekiwanie na semafor
{
	return sem_wait(&reader_sem);
}

int reader_buffer::lock_mutex() // zajecie mutex'a
{
	return pthread_mutex_lock( &reader_mutex);
}

int reader_buffer::unlock_mutex() // zwolnienie mutex'a
{
	return pthread_mutex_unlock( &reader_mutex);
}

master_trans_t_buffer::master_trans_t_buffer()
{

	// semafory do komunikacji miedzy EDP_MASTER a EDP_TRANS
	sem_init( &(master_to_trans_t_sem), 0, 0);
	sem_init( &(trans_t_to_master_sem), 0, 0);

}

int master_trans_t_buffer::master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb)
{ // zlecenie z watku master dla trans_t
	trans_t_task = nm_task; // force, arm etc.
	trans_t_tryb = nm_tryb; // tryb dla zadania

	// odwieszenie watku transformation
	sem_trywait(&(master_to_trans_t_sem));
	sem_post(&(master_to_trans_t_sem));

	// oczekiwanie na zwolniene samafora przez watek trans_t
	sem_wait(&(trans_t_to_master_sem)); // oczekiwanie na zezwolenie ruchu od edp_master

	// sekcja sprawdzajaca czy byl blad w watku transforamation i ew. rzucajaca go w watku master

	switch (error) {
		case NonFatal_erroR_1:
			throw *(NonFatal_error_1*)(error_pointer);
			break;
		case NonFatal_erroR_2:
			throw *(NonFatal_error_2*)(error_pointer);
			break;
		case NonFatal_erroR_3:
			throw *(NonFatal_error_3*)(error_pointer);
			break;
		case NonFatal_erroR_4:
			throw *(NonFatal_error_4*)(error_pointer);
			break;
		case Fatal_erroR:
			throw *(Fatal_error*)(error_pointer);
			break;
		case System_erroR:
			throw *(System_error*)(error_pointer);
			break;
		default:
			break;
	}

	return 1;
}

// oczekiwanie na semafor statusu polecenia z trans_t
int master_trans_t_buffer::master_wait_for_trans_t_order_status()
{

	// oczekiwanie na odpowiedz z watku transformation
	return sem_wait(&(trans_t_to_master_sem));
}

// oczekiwanie na semafor statusu polecenia z trans_t
int master_trans_t_buffer::trans_t_to_master_order_status_ready()
{

	// odwieszenie watku new master
	sem_trywait(&(trans_t_to_master_sem));
	return sem_post(&(trans_t_to_master_sem));// odwieszenie watku edp_master
}

// oczekiwanie na semafor statusu polecenia z trans_t
int master_trans_t_buffer::trans_t_wait_for_master_order()
{

	// oczekiwanie na rozkaz z watku master
	return sem_wait(&(master_to_trans_t_sem));
}

// -------------------------------------------------------------------------
// Plik:			srlib.cc
// Opis:		metody klas SR
// -------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <sys/utsname.h>

#include <boost/thread/mutex.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

namespace mrrocpp {
namespace lib {

/*
 sr_package::sr_package()
 : process_type(UNKNOWN_PROCESS_TYPE),
 message_type(-1)
 {
 memset(process_name, 0, sizeof(process_name));
 memset(host_name, 0, sizeof(host_name));
 memset(description, 0, sizeof(description));
 ts.tv_sec = ts.tv_nsec = 0;
 }
 */

#if !defined(USE_MESSIP_SRR)

sr::sr(process_type_t process_type, const std::string & process_name,
		const std::string & sr_name, bool _multi_thread) :
	has_command(false),
	multi_thread(_multi_thread),
	cb(SR_BUFFER_LENGHT)
{
	// kilka sekund  (~1) na otworzenie urzadzenia
	int tmp = 0;
	while ((fd = name_open(sr_name.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0) {
		if ((tmp++) < CONNECT_RETRY) {
			delay(CONNECT_DELAY);
		} else {
			// TODO: throw
			perror("SR cannot be located ");
			throw ;
		}
	}

	struct utsname sysinfo;
	if( uname( &sysinfo ) == -1 ) {
		// TODO: throw
		perror( "uname" );
	}
	strcpy(sr_message.host_name, sysinfo.nodename);

	sr_message.hdr.type=0;

	sr_message.process_type = process_type;
	sr_message.message_type = NEW_MESSAGE;
	strcpy(sr_message.process_name, process_name.c_str());
	for (int i=0; i < ERROR_TAB_SIZE; i++) {
		error_tab[i] = 0;
	}

	if (multi_thread) {
		thread_id = new boost::thread(boost::bind(&sr::operator(), this));
	}
} // end:  sr::sr()

// Destruktor
sr::~sr(void) {
	if (multi_thread)
	{
		delete thread_id;
	}
	name_close(fd);
}

int sr::send_package_to_sr(const sr_package_t & sr_mess)
{
	int16_t status;

	return MsgSend(fd, &sr_mess, sizeof(sr_mess), &status, sizeof(status));
}
#else /* USE_MESSIP_SRR */
// Konstruktor
sr::sr(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
multi_thread(_multi_thread), cb(SR_BUFFER_LENGHT)
{
	int tmp = 0;
	while ((ch = messip::port_connect(sr_name)) == NULL) {
		if (tmp++ < 50) {
			delay(50);
		} else {
			fprintf(stderr, "messip::port_connect(\"%s\") @ %s:%d: %s\n",
					sr_name.c_str(), __FILE__, __LINE__, strerror(errno));
			// TODO: throw
			assert(0);
		}
	}

	assert(ch);

	memset(&sr_message, 0, sizeof(sr_message));

	struct utsname sysinfo;

	if( uname( &sysinfo ) == -1 ) {
		perror( "uname" );
	}
	strcpy(sr_message.host_name, sysinfo.nodename);
	sr_message.process_type = process_type;
	sr_message.message_type = NEW_MESSAGE;
	strcpy(sr_message.process_name, process_name.c_str());
	for (int i=0; i < ERROR_TAB_SIZE; i++) {
		error_tab[i] = 0;
	}

	if (multi_thread) {
		thread_id = new boost::thread(boost::bind(&sr::operator(), this));
	}
} // end:  sr::sr()

// Destruktor
sr::~sr(void) {
	if (multi_thread)
	{
		delete thread_id;
	}

	fprintf(stderr, "~sr: messip::port_disconnect(%s)\n", sr_message.process_name);
	if(messip::port_disconnect(ch) == -1) {
		perror("messip::port_disconnect()");
	}
}

int sr::send_package_to_sr(const sr_package_t & sr_mess)
{
	return messip::port_send_sync(ch, 0, 0, sr_mess);
}
#endif /* !USE_MESSIP_SRR */

int sr::send_package(void) {
	clock_gettime(CLOCK_REALTIME, &sr_message.ts);
	if (!multi_thread)
	{
		return send_package_to_sr(sr_message);
	} else
	{
		put_one_msg(sr_message);

		return 1;
	}
}

void sr::put_one_msg(const lib::sr_package_t& new_msg)
{
	boost::unique_lock<boost::mutex> lock(mtx);
	cb.push_back(new_msg);

	has_command = true;

	cond.notify_one();

	return;
}

void sr::get_one_msg(lib::sr_package_t& new_msg) {
	boost::unique_lock<boost::mutex> lock(mtx);
	new_msg = cb.front();
	cb.pop_front();
	has_command = false;
	return;
}

bool sr::buffer_empty() // sprawdza czy bufor jest pusty
{
	boost::unique_lock<boost::mutex> lock(mtx);
	return cb.empty();
}

void sr::wait_for_new_msg() // oczekiwanie na semafor
{
	boost::unique_lock<boost::mutex> lock(mtx);

	while (!has_command) {
		cond.wait(lock);
	}
}

/* -------------------------------------------------------------------- */
/* Wysylka wiadomosci do procesu SR                                     */
/* -------------------------------------------------------------------- */

int sr::message(const std::string & text) {
	boost::mutex::scoped_lock lock(srMutex);

	sr_message.message_type = NEW_MESSAGE;
	if(text.length()) {
		strncpy(sr_message.description, text.c_str(), TEXT_LENGTH);
		sr_message.description[TEXT_LENGTH-1] = '\0';
	} else {
		sr_message.description[0] = '\0';
	}
	return send_package();
} // end: sr::message()


int sr::message(error_class_t message_type, const std::string & text) {
	boost::mutex::scoped_lock lock(srMutex);

	sr_message.message_type = message_type;
	if(text.length()) {
		strncpy(sr_message.description, text.c_str(), TEXT_LENGTH);
		sr_message.description[TEXT_LENGTH-1] = '\0';
	} else {
		sr_message.description[0] = '\0';
	}
	return send_package();
} // end: sr::message()


int sr::message(error_class_t message_type, uint64_t error_code, const std::string & text) {
	boost::mutex::scoped_lock lock(srMutex);

	sr_message.message_type = message_type;
	error_tab[0] = error_code;
	interpret();
	strcat(sr_message.description, text.c_str());
	return send_package();
} // end: sr::message()

int sr::message(error_class_t message_type, uint64_t error_code) {
	boost::mutex::scoped_lock lock(srMutex);

	sr_message.message_type = message_type;
	error_tab[0] = error_code;
	interpret();
	return send_package();
} // end: sr::message()

int sr::message(error_class_t message_type, uint64_t error_code0, uint64_t error_code1) {
	boost::mutex::scoped_lock lock(srMutex);

	sr_message.message_type = message_type;
	error_tab[0] = error_code0;
	error_tab[1] = error_code1;
	interpret();
	return send_package();
} // end: sr::message()
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// interpretacja bledu dla EDP robota irp6_on_track

sr_edp::sr_edp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
sr(process_type, process_name, sr_name, _multi_thread)
{
}

void sr_edp::interpret(void) {
	uint64_t s_error; // zmienna pomocnicza
	char tbuf[1+1]; // bufor tymczasowy na liczby

	sr_message.description[0] = '\0';
	switch (sr_message.message_type) {
		case SYSTEM_ERROR: // interpretacja do funkcji:
		// message(int16_t message_type, uint64_t error_code, char *text)

		break;
		case FATAL_ERROR:
		case NON_FATAL_ERROR: // interpretacja do funkcji:
		// message(int16_t message_type, uint64_t error_code0, uint64_t error_code1)
		s_error = error_tab[0];
		for (int j = 0; j < MAX_SERVOS_NR; j++) {
			if ( s_error & 0x00000001 ) {
				sprintf(tbuf, "%1d", j+1);
				strcpy(sr_message.description,"SERVO_");
				strcat(sr_message.description, tbuf);
				strcat(sr_message.description, "_LOWER_LIMIT_SWITCH ");
			}
			s_error >>= 1;
			if ( s_error & 0x00000001 ) {
				sprintf(tbuf, "%1d", j+1);
				strcat(sr_message.description,"SERVO_");
				strcat(sr_message.description, tbuf);
				strcat(sr_message.description, "_UPPER_LIMIT_SWITCH ");
			}
			s_error >>= 1;
			if ( s_error & 0x00000001 ) {
				sprintf(tbuf, "%1d", j+1);
				strcat(sr_message.description,"SERVO_");
				strcat(sr_message.description, tbuf);
				strcat(sr_message.description, "_OVER_CURRENT ");
			}
			s_error >>= 1;
		} // end: for

		s_error = error_tab[0];

		switch ( s_error & 0x003C000000000000ULL ) {
			case OK: break; // OK
			case SERVO_ERROR_IN_PASSIVE_LOOP: strcat (sr_message.description, "SERVO_ERROR_IN_PASSIVE_LOOP "); break;
			case UNIDENTIFIED_SERVO_COMMAND: strcat (sr_message.description, "UNIDENTIFIED_SERVO_COMMAND "); break;
			case SERVO_ERROR_IN_PHASE_1: strcat (sr_message.description, "SERVO_ERROR_IN_PHASE_1 "); break;
			case SERVO_ERROR_IN_PHASE_2: strcat (sr_message.description, "SERVO_ERROR_IN_PHASE_2 "); break;
			case SYNCHRO_SWITCH_EXPECTED: strcat (sr_message.description, "SYNCHRO_SWITCH_EXPECTED "); break;
			case SYNCHRO_ERROR: strcat (sr_message.description, "SYNCHRO_ERROR "); break;
			case SYNCHRO_DELAY_ERROR: strcat (sr_message.description, "SYNCHRO_DELAY_ERROR "); break;
			default:
			strcat (sr_message.description, "UNIDENTIFIED_SERVO_ERROR "); break;
		}

		switch ( s_error & 0xFF00000000000000ULL ) {
			case OK: break;// OK
			case INVALID_INSTRUCTION_TYPE: strcat (sr_message.description, "INVALID_INSTRUCTION_TYPE"); break;
			case INVALID_REPLY_TYPE: strcat (sr_message.description, "INVALID_REPLY_TYPE"); break;
			case INVALID_SET_ROBOT_MODEL_TYPE: strcat (sr_message.description, "INVALID_SET_ROBOT_MODEL_TYPE"); break;
			case INVALID_GET_ROBOT_MODEL_TYPE: strcat (sr_message.description, "INVALID_GET_ROBOT_MODEL_TYPE"); break;
			case ERROR_IN_ROBOT_MODEL_REQUEST: strcat (sr_message.description, "ERROR_IN_ROBOT_MODEL_REQUEST"); break;
			case INVALID_HOMOGENEOUS_MATRIX: strcat (sr_message.description, "INVALID_HOMOGENEOUS_MATRIX"); break;
			case QUERY_EXPECTED: strcat (sr_message.description, "QUERY_EXPECTED"); break;
			case QUERY_NOT_EXPECTED: strcat (sr_message.description, "QUERY_NOT_EXPECTED"); break;
			case NO_VALID_END_EFFECTOR_POSE: strcat (sr_message.description, "NO_VALID_END_EFFECTOR_POSE"); break;
			case INVALID_MOTION_TYPE: strcat (sr_message.description, "INVALID_MOTION_TYPE"); break;
			case INVALID_MOTION_PARAMETERS: strcat (sr_message.description, "INVALID_MOTION_PARAMETERS"); break;
			case INVALID_SET_END_EFFECTOR_TYPE: strcat (sr_message.description, "INVALID_SET_END_EFFECTOR_TYPE"); break;
			case INVALID_GET_END_EFFECTOR_TYPE: strcat (sr_message.description, "INVALID_GET_END_EFFECTOR_TYPE"); break;
			case STRANGE_GET_ARM_REQUEST: strcat (sr_message.description, "STRANGE_GET_ARM_REQUEST"); break;
			case BEYOND_UPPER_LIMIT_AXIS_0: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_0"); break;
			case BEYOND_UPPER_LIMIT_AXIS_1: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_1"); break;
			case BEYOND_UPPER_LIMIT_AXIS_2: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_2"); break;
			case BEYOND_UPPER_LIMIT_AXIS_3: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_3"); break;
			case BEYOND_UPPER_LIMIT_AXIS_4: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_4"); break;
			case BEYOND_UPPER_LIMIT_AXIS_5: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_5"); break;
			case BEYOND_UPPER_LIMIT_AXIS_6: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_6"); break;
			case BEYOND_UPPER_LIMIT_AXIS_7: strcat (sr_message.description, "BEYOND_UPPER_LIMIT_AXIS_7"); break;
			case BEYOND_LOWER_LIMIT_AXIS_0: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_0"); break;
			case BEYOND_LOWER_LIMIT_AXIS_1: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_1"); break;
			case BEYOND_LOWER_LIMIT_AXIS_2: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_2"); break;
			case BEYOND_LOWER_LIMIT_AXIS_3: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_3"); break;
			case BEYOND_LOWER_LIMIT_AXIS_4: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_4"); break;
			case BEYOND_LOWER_LIMIT_AXIS_5: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_5"); break;
			case BEYOND_LOWER_LIMIT_AXIS_6: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_6"); break;
			case BEYOND_LOWER_LIMIT_AXIS_7: strcat (sr_message.description, "BEYOND_LOWER_LIMIT_AXIS_7"); break;
			case BEYOND_UPPER_D0_LIMIT: strcat (sr_message.description, "BEYOND_UPPER_Q1_LIMIT"); break;
			case BEYOND_UPPER_THETA1_LIMIT: strcat (sr_message.description, "BEYOND_UPPER_LEFT_LIMIT"); break;
			case BEYOND_UPPER_THETA2_LIMIT: strcat (sr_message.description, "BEYOND_UPPER_RIGHT_LIMIT"); break;
			case BEYOND_UPPER_THETA3_LIMIT: strcat (sr_message.description, "BEYOND_Q4_LIMIT_RANGE"); break;
			case BEYOND_UPPER_THETA4_LIMIT: strcat (sr_message.description, "BEYOND_UPPER_Q5_LIMIT"); break;
			case BEYOND_UPPER_THETA5_LIMIT: strcat (sr_message.description, "BEYOND_UPPER_Q6_LIMIT"); break;
			case BEYOND_UPPER_THETA6_LIMIT: strcat (sr_message.description, "BEYOND_UPPER_Q7_LIMIT"); break;
			case BEYOND_UPPER_THETA7_LIMIT: strcat (sr_message.description, "BEYOND_UPPER_Q8_LIMIT"); break;
			case BEYOND_LOWER_D0_LIMIT: strcat (sr_message.description, "BEYOND_LOWER_Q1_LIMIT"); break;
			case BEYOND_LOWER_THETA1_LIMIT: strcat (sr_message.description, "BEYOND_LOWER_LEFT_LIMIT"); break;
			case BEYOND_LOWER_THETA2_LIMIT: strcat (sr_message.description, "BEYOND_LOWER_RIGHT_LIMIT"); break;
			case BEYOND_LOWER_THETA3_LIMIT: strcat (sr_message.description, "BEYOND_Q4_LIMIT_RANGE"); break;
			case BEYOND_LOWER_THETA4_LIMIT: strcat (sr_message.description, "BEYOND_LOWER_Q5_LIMIT"); break;
			case BEYOND_LOWER_THETA5_LIMIT: strcat (sr_message.description, "BEYOND_LOWER_Q6_LIMIT"); break;
			case BEYOND_LOWER_THETA6_LIMIT: strcat (sr_message.description, "BEYOND_LOWER_Q7_LIMIT"); break;
			case BEYOND_LOWER_THETA7_LIMIT: strcat (sr_message.description, "BEYOND_LOWER_Q8_LIMIT"); break;
			case OUT_OF_WORKSPACE: strcat (sr_message.description, "OUT_OF_WORKSPACE"); break;
			case SINGULAR_POSE: strcat (sr_message.description, "SINGULAR_POSE"); break;
			case ACOS_DOMAIN_ERROR: strcat (sr_message.description, "ACOS_DOMAIN_ERROR"); break;
			case ASIN_DOMAIN_ERROR: strcat (sr_message.description, "ASIN_DOMAIN_ERROR"); break;
			case ATAN2_DOMAIN_ERROR: strcat (sr_message.description, "ATAN2_DOMAIN_ERROR"); break;
			case SQRT_DOMAIN_ERROR: strcat (sr_message.description, "SQRT_DOMAIN_ERROR"); break;
			case UNKNOWN_MATH_ERROR: strcat (sr_message.description, "UNKNOWN_MATH_ERROR"); break;
			case UNKNOWN_INSTRUCTION: strcat (sr_message.description, "UNKNOWN_INSTRUCTION"); break;
			case NOT_IMPLEMENTED_YET: strcat (sr_message.description, "NOT_IMPLEMENTED_YET"); break;
			case NOT_YET_SYNCHRONISED: strcat (sr_message.description, "NOT_YET_SYNCHRONISED"); break;
			case ALREADY_SYNCHRONISED: strcat (sr_message.description, "ALREADY_SYNCHRONISED"); break;
			case UNKNOWN_SYNCHRO_ERROR: strcat (sr_message.description, "UNKNOWN_SYNCHRO_ERROR"); break;
			case INVALID_KINEMATIC_MODEL_NO: strcat (sr_message.description, "INVALID_KINEMATIC_MODEL_NO"); break;
			case EDP_UNIDENTIFIED_ERROR: strcat (sr_message.description, "EDP_UNIDENTIFIED_ERROR"); break;
			case NOT_A_NUMBER_JOINT_VALUE_D0: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_D0"); break;
			case NOT_A_NUMBER_JOINT_VALUE_THETA1: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_THETA1"); break;
			case NOT_A_NUMBER_JOINT_VALUE_THETA2: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_THETA2"); break;
			case NOT_A_NUMBER_JOINT_VALUE_THETA3: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_THETA3"); break;
			case NOT_A_NUMBER_JOINT_VALUE_THETA4: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_THETA4"); break;
			case NOT_A_NUMBER_JOINT_VALUE_THETA5: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_THETA5"); break;
			case NOT_A_NUMBER_JOINT_VALUE_THETA6: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_THETA6"); break;
			case NOT_A_NUMBER_JOINT_VALUE_THETA7: strcat (sr_message.description, "NOT_A_NUMBER_JOINT_VALUE_THETA7"); break;
			default:
			strcat (sr_message.description, "UNIDENTIFIED_ERROR"); break;
		}

		// analiza informacji zawartej w error_tab[1]
		s_error = error_tab[1];
		for (int j = 0; j < MAX_SERVOS_NR; j++) {
			if ( s_error & 0x00000001 ) {
				sprintf(tbuf, "%1d", j+1);
				strcat(sr_message.description,"SERVO ");
				strcat(sr_message.description, tbuf);
				strcat(sr_message.description, " : UNIDENTIFIED_ALGORITHM_NO");
			}
			s_error >>= 1;
			if ( s_error & 0x00000001 ) {
				sprintf(tbuf, "%1d", j+1);
				strcat(sr_message.description,"SERVO ");
				strcat(sr_message.description, tbuf);
				strcat(sr_message.description, " : UNIDENTIFIED_ALGORITHM_PARAMETERS_NO");
			}
			s_error >>= 1;
		}
		break;
		default:
		strcat (sr_message.description, "EDP UNIDENTIFIED ERROR");
	}
} // end: sr_edp::interpret()
// ---------------------------------------------------------------------

sr_ecp::sr_ecp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
sr(process_type, process_name, sr_name, _multi_thread)
{
}

// ---------------------------------------------------------------------
// Interpretacja bledow generowanych w ECP i MP
void sr_ecp::interpret(void) {

	switch (sr_message.message_type) {
		case SYSTEM_ERROR:
		switch (error_tab[0]) {
			case CANNOT_SPAWN_VSP:
			sprintf(sr_message.description, "CANNOT SPAWN VSP PROCESS "); break;
			case CANNOT_LOCATE_DEVICE:
			sprintf(sr_message.description, "CANNOT LOCATE DEVICE "); break;
			case CANNOT_READ_FROM_DEVICE:
			sprintf(sr_message.description, "CANNOT READ FROM DEVICE "); break;
			case CANNOT_WRITE_TO_DEVICE:
			sprintf(sr_message.description, "CANNOT WRITE TO DEVICE "); break;
			case DEVICE_ALREADY_EXISTS:
			sprintf(sr_message.description, "SENSOR DEVICE ALREADY EXISTS "); break;
			case NAME_ATTACH_ERROR:
			sprintf(sr_message.description, "NAME ATTACH ERROR"); break;
			default:
			sprintf(sr_message.description, "%s",strerror(error_tab[0]));
		}
		break; // SYSTEM_ERROR
		case FATAL_ERROR:
		switch (error_tab[0]) {
			case SAVE_FILE_ERROR:
			sprintf(sr_message.description, "SAVE FILE ERROR"); break;
			default:
			sprintf(sr_message.description, "%s",strerror(error_tab[0]));
		}
		break; // FATAL_ERROR
		case NON_FATAL_ERROR: // interpretacja do funkcji:
		// message(int16_t message_type, uint64_t error_code)
		// message(int16_t message_type, uint64_t error_code, char *text)
		switch (error_tab[0]) {
			case EDP_ERROR:
			sprintf (sr_message.description, "ERROR IN EDP"); break;
			case INVALID_MP_COMMAND:
			sprintf(sr_message.description, "INVALID MP COMMAND"); break;
			case INVALID_POSE_SPECIFICATION:
			sprintf(sr_message.description, "INVALID POSE SPECIFICATION"); break;
			case INVALID_ROBOT_MODEL_TYPE:
			sprintf(sr_message.description, "INVALID ROBOT_MODEL TYPE"); break;
			case INVALID_ECP_COMMAND:
			sprintf(sr_message.description, "INVALID ECP COMMAND"); break;
			case INVALID_EDP_REPLY:
			sprintf(sr_message.description, "INVALID EDP REPLY"); break;
			case ECP_ERRORS:
			sprintf(sr_message.description, "ECP ERRORS"); break;
			case INVALID_COMMAND_TO_EDP:
			sprintf(sr_message.description, "INVALID COMMAND TO EDP"); break;
			case ECP_UNIDENTIFIED_ERROR:
			sprintf(sr_message.description, "ECP UNIDENTIFIED ERROR"); break;
			case MP_UNIDENTIFIED_ERROR:
			sprintf(sr_message.description, "MP UNIDENTIFIED ERROR"); break;
			case NON_EXISTENT_DIRECTORY:
			sprintf(sr_message.description, "NON-EXISTENT DIRECTORY"); break;
			case NON_EXISTENT_FILE:
			sprintf(sr_message.description, "NON-EXISTENT FILE"); break;
			case READ_FILE_ERROR:
			sprintf(sr_message.description, "READ FILE ERROR"); break;
			case NON_TRAJECTORY_FILE:
			sprintf(sr_message.description, "NON-TRAJECTORY FILE"); break;
			case NON_COMPATIBLE_LISTS:
			sprintf(sr_message.description, "NON-COMPATIBLE LISTS"); break;
			case MAX_ACCELERATION_EXCEEDED:
			sprintf(sr_message.description, "MAX ACCELERATION EXCEEDED"); break;
			case MAX_VELOCITY_EXCEEDED:
			sprintf(sr_message.description, "MAX VELOCITY EXCEEDED"); break;
			case NOT_ENOUGH_MEMORY:
			sprintf(sr_message.description, "NOT ENOUGH MEMORY"); break;
			case INVALID_VSP_REPLY:
			sprintf(sr_message.description, "INVALID VSP REPLY"); break;
			case DANGEROUS_FORCE_DETECTED:
			sprintf(sr_message.description, "DANGEROUS FORCE DETECTED"); break;
			case BAD_VSP_REPLY:
			sprintf(sr_message.description, "BAD VSP REPLY"); break;
			case INVALID_ECP_PULSE_IN_MP_START_ALL:
			sprintf(sr_message.description, "INVALID ECP PULSE IN MP START ALL"); break;
			case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
			sprintf(sr_message.description, "INVALID ECP PULSE IN MP EXECUTE ALL"); break;
			case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
			sprintf(sr_message.description, "INVALID ECP PULSE IN MP TERMINATE ALL"); break;
			default:
			sprintf(sr_message.description, "UNIDENTIFIED ECP or MP ERROR");
		} // end: switch (sr_message.error_tab[0])
		break;
		default:
		sprintf(sr_message.description, "UNIDENTIFIED ECP or MP ERROR");
	} // end: switch (sr_message.message_type)
} // end: sr_ecp::interpret()

// ---------------------------------------------------------------------
sr_ui::sr_ui(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
sr(process_type, process_name, sr_name, _multi_thread)
{
}

// Interpretacja bledow generowanych w UI // by Y - UWAGA UZUPELNIC
void sr_ui::interpret(void) {
	switch (sr_message.message_type) {
		case NEW_MESSAGE:
		sprintf(sr_message.description, "%s", strerror(error_tab[0]));
		break; // NEW_MESSAGE
		default:
		sprintf(sr_message.description, "UNIDENTIFIED UI ERROR");
	}
}

sr_vsp::sr_vsp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
sr(process_type, process_name, sr_name, _multi_thread)
{
}

// Interpretacja bledow generowanych w VSP
void sr_vsp::interpret(void) {
	switch (sr_message.message_type) {
		case SYSTEM_ERROR:
		switch (error_tab[0]) {
			case DISPATCH_ALLOCATION_ERROR:
			sprintf(sr_message.description, "SENSOR DISPATCH ALLOCATION ERROR "); break;
			case DEVICE_EXISTS:
			sprintf(sr_message.description, "SENSOR DEVICE ALREADY EXISTS "); break;
			default:
			sprintf(sr_message.description, "UNIDENTIFIED VSP ERROR ");
		}
		break; // SYSTEM_ERROR
		case FATAL_ERROR:
		switch (error_tab[0]) {
			default:
			sprintf(sr_message.description, "UNIDENTIFIED VSP ERROR ");
		}
		break; // FATAL_ERROR
		case NON_FATAL_ERROR:
		switch (error_tab[0]) {
			case SENSOR_NOT_CONFIGURED:
			sprintf(sr_message.description, "SENSOR NOT CONFIGURED "); break;
			case READING_NOT_READY:
			sprintf(sr_message.description, "SENSOR READING NOT READY "); break;
			case INVALID_COMMAND_TO_VSP:
			sprintf(sr_message.description, "INVALID COMMAND TO VSP "); break;
			default:
			sprintf(sr_message.description, "UNIDENTIFIED VSP ERROR ");
		} // end: switch (sr_message.error_tab[0])
		break;
		case NEW_MESSAGE:
		sprintf(sr_message.description, "%s", strerror(error_tab[0]));
		break; // NEW_MESSAGE
		default:
		sprintf(sr_message.description, "UNIDENTIFIED VSP ERROR");
	}
}

void sr::operator()()

{	sr_package_t local_message;
	while (1) {

		wait_for_new_msg();

		while (!buffer_empty())
		{

			{
				get_one_msg(local_message);
			}
			try
			{
				send_package_to_sr(local_message);
			}
			catch (...)
			{
				printf("send_package_to_sr error multi_thread variant\n");
			}
		}

	}
}

} // namespace lib
} // namespace mrrocpp

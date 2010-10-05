/*
 * sr_ecp.cc
 *
 *  Created on: Sep 9, 2010
 *      Author: ptroja
 */

#include <cstdio>
#include <cstring>

#include "base/lib/sr/sr_ecp.h"

namespace mrrocpp {
namespace lib {

sr_ecp::sr_ecp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
	sr(process_type, process_name, sr_name, _multi_thread)
{
}

void sr_ecp::interpret(void)
{

	switch (sr_message.message_type)
	{
		case SYSTEM_ERROR:
			switch (error_tab[0])
			{
				case CANNOT_SPAWN_VSP:
					sprintf(sr_message.description, "CANNOT SPAWN VSP PROCESS ");
					break;
				case CANNOT_LOCATE_DEVICE:
					sprintf(sr_message.description, "CANNOT LOCATE DEVICE ");
					break;
				case CANNOT_READ_FROM_DEVICE:
					sprintf(sr_message.description, "CANNOT READ FROM DEVICE ");
					break;
				case CANNOT_WRITE_TO_DEVICE:
					sprintf(sr_message.description, "CANNOT WRITE TO DEVICE ");
					break;
				case DEVICE_ALREADY_EXISTS:
					sprintf(sr_message.description, "SENSOR DEVICE ALREADY EXISTS ");
					break;
				case NAME_ATTACH_ERROR:
					sprintf(sr_message.description, "NAME ATTACH ERROR");
					break;
				default:
					sprintf(sr_message.description, "%s", strerror(error_tab[0]));
			}
			break; // SYSTEM_ERROR
		case FATAL_ERROR:
			switch (error_tab[0])
			{
				case SAVE_FILE_ERROR:
					sprintf(sr_message.description, "SAVE FILE ERROR");
					break;
				default:
					sprintf(sr_message.description, "%s", strerror(error_tab[0]));
			}
			break; // FATAL_ERROR
		case NON_FATAL_ERROR: // interpretacja do funkcji:
			// message(int16_t message_type, uint64_t error_code)
			// message(int16_t message_type, uint64_t error_code, char *text)
			switch (error_tab[0])
			{
				case EDP_ERROR:
					sprintf(sr_message.description, "ERROR IN EDP");
					break;
				case INVALID_MP_COMMAND:
					sprintf(sr_message.description, "INVALID MP COMMAND");
					break;
				case INVALID_POSE_SPECIFICATION:
					sprintf(sr_message.description, "INVALID POSE SPECIFICATION");
					break;
				case INVALID_ROBOT_MODEL_TYPE:
					sprintf(sr_message.description, "INVALID ROBOT_MODEL TYPE");
					break;
				case ECP_ERRORS:
					sprintf(sr_message.description, "ecp ERRORS");
					break;
				case INVALID_COMMAND_TO_EDP:
					sprintf(sr_message.description, "INVALID COMMAND TO EDP");
					break;
				case ECP_UNIDENTIFIED_ERROR:
					sprintf(sr_message.description, "ecp UNIDENTIFIED ERROR");
					break;
				case MP_UNIDENTIFIED_ERROR:
					sprintf(sr_message.description, "mp UNIDENTIFIED ERROR");
					break;
				case NON_EXISTENT_DIRECTORY:
					sprintf(sr_message.description, "NON-EXISTENT DIRECTORY");
					break;
				case NON_EXISTENT_FILE:
					sprintf(sr_message.description, "NON-EXISTENT FILE");
					break;
				case READ_FILE_ERROR:
					sprintf(sr_message.description, "READ FILE ERROR");
					break;
				case NON_TRAJECTORY_FILE:
					sprintf(sr_message.description, "NON-TRAJECTORY FILE");
					break;
				case NON_COMPATIBLE_LISTS:
					sprintf(sr_message.description, "NON-COMPATIBLE LISTS");
					break;
				case MAX_ACCELERATION_EXCEEDED:
					sprintf(sr_message.description, "MAX ACCELERATION EXCEEDED");
					break;
				case MAX_VELOCITY_EXCEEDED:
					sprintf(sr_message.description, "MAX VELOCITY EXCEEDED");
					break;
				case DANGEROUS_FORCE_DETECTED:
					sprintf(sr_message.description, "DANGEROUS FORCE DETECTED");
					break;
				case INVALID_ECP_PULSE_IN_MP_START_ALL:
					sprintf(sr_message.description, "INVALID ECP PULSE IN MP START ALL");
					break;
				case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
					sprintf(sr_message.description, "INVALID ECP PULSE IN MP EXECUTE ALL");
					break;
				case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
					sprintf(sr_message.description, "INVALID ECP PULSE IN MP TERMINATE ALL");
					break;
				default:
					sprintf(sr_message.description, "UNIDENTIFIED ECP or MP ERROR");
			} // end: switch (sr_message.error_tab[0])
			break;
		default:
			sprintf(sr_message.description, "UNIDENTIFIED ECP or MP ERROR");
	} // end: switch (sr_message.message_type)
}

} // namespace lib
} // namespace mrrocpp

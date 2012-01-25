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

sr_ecp::sr_ecp(process_type_t process_type, const std::string & process_name, const std::string & sr_name) :
	sr(process_type, process_name, sr_name)
{
}

void sr_ecp::interpret(char * description, error_class_t message_type, uint64_t error_code0, uint64_t error_code1)
{
	switch (message_type)
	{
		case SYSTEM_ERROR:
			switch (error_code0)
			{
				case CANNOT_SPAWN_VSP:
					sprintf(description, "CANNOT SPAWN VSP PROCESS ");
					break;
				case CANNOT_LOCATE_DEVICE:
					sprintf(description, "CANNOT LOCATE DEVICE ");
					break;
				case CANNOT_READ_FROM_DEVICE:
					sprintf(description, "CANNOT READ FROM DEVICE ");
					break;
				case CANNOT_WRITE_TO_DEVICE:
					sprintf(description, "CANNOT WRITE TO DEVICE ");
					break;
				case DEVICE_ALREADY_EXISTS:
					sprintf(description, "SENSOR DEVICE ALREADY EXISTS ");
					break;
				case NAME_ATTACH_ERROR:
					sprintf(description, "NAME ATTACH ERROR");
					break;
				default:
					sprintf(description, "%s", strerror(error_code0));
					break;
			}
			break; // SYSTEM_ERROR
		case FATAL_ERROR:
			switch (error_code0)
			{
				case SAVE_FILE_ERROR:
					sprintf(description, "SAVE FILE ERROR");
					break;
				default:
					sprintf(description, "%s", strerror(error_code0));
					break;
			}
			break; // FATAL_ERROR
		case NON_FATAL_ERROR: // interpretacja do funkcji:
			// message(int16_t message_type, uint64_t error_code)
			// message(int16_t message_type, uint64_t error_code, char *text)
			switch (error_code0)
			{
				case EDP_ERROR:
					sprintf(description, "ERROR IN EDP");
					break;
				case INVALID_MP_COMMAND:
					sprintf(description, "INVALID MP COMMAND");
					break;
				case INVALID_POSE_SPECIFICATION:
					sprintf(description, "INVALID POSE SPECIFICATION");
					break;
				case INVALID_ROBOT_MODEL_TYPE:
					sprintf(description, "INVALID ROBOT_MODEL TYPE");
					break;
				case ECP_ERRORS:
					sprintf(description, "ecp ERRORS");
					break;
				case INVALID_COMMAND_TO_EDP:
					sprintf(description, "INVALID COMMAND TO EDP");
					break;
				case ECP_UNIDENTIFIED_ERROR:
					sprintf(description, "ecp UNIDENTIFIED ERROR");
					break;
				case MP_UNIDENTIFIED_ERROR:
					sprintf(description, "mp UNIDENTIFIED ERROR");
					break;
				case NON_EXISTENT_DIRECTORY:
					sprintf(description, "NON-EXISTENT DIRECTORY");
					break;
				case NON_EXISTENT_FILE:
					sprintf(description, "NON-EXISTENT FILE");
					break;
				case READ_FILE_ERROR:
					sprintf(description, "READ FILE ERROR");
					break;
				case NON_TRAJECTORY_FILE:
					sprintf(description, "NON-TRAJECTORY FILE");
					break;
				case NON_COMPATIBLE_LISTS:
					sprintf(description, "NON-COMPATIBLE LISTS");
					break;
				case MAX_ACCELERATION_EXCEEDED:
					sprintf(description, "MAX ACCELERATION EXCEEDED");
					break;
				case MAX_VELOCITY_EXCEEDED:
					sprintf(description, "MAX VELOCITY EXCEEDED");
					break;
				case DANGEROUS_FORCE_DETECTED:
					sprintf(description, "DANGEROUS FORCE DETECTED");
					break;
				case INVALID_ECP_PULSE_IN_MP_START_ALL:
					sprintf(description, "INVALID ECP PULSE IN MP START ALL");
					break;
				case INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL:
					sprintf(description, "INVALID ECP PULSE IN MP EXECUTE ALL");
					break;
				case INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL:
					sprintf(description, "INVALID ECP PULSE IN MP TERMINATE ALL");
					break;
				default:
					sprintf(description, "UNIDENTIFIED ECP or MP ERROR");
					break;
			} // end: switch (error_code0)
			break;
		default:
			sprintf(description, "UNIDENTIFIED ECP or MP ERROR");
			break;
	} // end: switch (message_type)
}


} // namespace lib
} // namespace mrrocpp

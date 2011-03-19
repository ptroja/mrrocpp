/*
 * sr_vsp.cc
 *
 *  Created on: Sep 9, 2010
 *      Author: ptroja
 */

#include <cstdio>
#include <cstring>

#include "base/lib/sr/sr_vsp.h"

namespace mrrocpp {
namespace lib {

sr_vsp::sr_vsp(process_type_t process_type, const std::string & process_name, const std::string & sr_name) :
	sr(process_type, process_name, sr_name)
{
}

// Interpretacja bledow generowanych w VSP
void sr_vsp::interpret(char * description, error_class_t message_type, uint64_t error_code0, uint64_t error_code1)
{
	switch (message_type)
	{
		case SYSTEM_ERROR:
			switch (error_code0)
			{
				case DISPATCH_ALLOCATION_ERROR:
					sprintf(description, "SENSOR DISPATCH ALLOCATION ERROR ");
					break;
				case DEVICE_EXISTS:
					sprintf(description, "SENSOR DEVICE ALREADY EXISTS ");
					break;
				default:
					sprintf(description, "UNIDENTIFIED VSP ERROR ");
			}
			break; // SYSTEM_ERROR
		case FATAL_ERROR:
			switch (error_code0)
			{
				default:
					sprintf(description, "UNIDENTIFIED VSP ERROR ");
			}
			break; // FATAL_ERROR
		case NON_FATAL_ERROR:
			switch (error_code0)
			{
				case SENSOR_NOT_CONFIGURED:
					sprintf(description, "SENSOR NOT CONFIGURED ");
					break;
				case READING_NOT_READY:
					sprintf(description, "SENSOR READING NOT READY ");
					break;
				case INVALID_COMMAND_TO_VSP:
					sprintf(description, "INVALID COMMAND TO VSP ");
					break;
				default:
					sprintf(description, "UNIDENTIFIED VSP ERROR ");
			} // end: switch (sr_message.error_code0)
			break;
		case NEW_MESSAGE:
			sprintf(description, "%s", strerror(error_code0));
			break; // NEW_MESSAGE
		default:
			sprintf(description, "UNIDENTIFIED VSP ERROR");
	}
}

} // namespace lib
} // namespace mrrocpp

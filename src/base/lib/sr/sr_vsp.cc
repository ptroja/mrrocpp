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

sr_vsp::sr_vsp(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
	sr(process_type, process_name, sr_name, _multi_thread)
{
}

// Interpretacja bledow generowanych w VSP
void sr_vsp::interpret(void)
{
	switch (sr_message.message_type)
	{
		case SYSTEM_ERROR:
			switch (error_tab[0])
			{
				case DISPATCH_ALLOCATION_ERROR:
					sprintf(sr_message.description, "SENSOR DISPATCH ALLOCATION ERROR ");
					break;
				case DEVICE_EXISTS:
					sprintf(sr_message.description, "SENSOR DEVICE ALREADY EXISTS ");
					break;
				default:
					sprintf(sr_message.description, "UNIDENTIFIED VSP ERROR ");
			}
			break; // SYSTEM_ERROR
		case FATAL_ERROR:
			switch (error_tab[0])
			{
				default:
					sprintf(sr_message.description, "UNIDENTIFIED VSP ERROR ");
			}
			break; // FATAL_ERROR
		case NON_FATAL_ERROR:
			switch (error_tab[0])
			{
				case SENSOR_NOT_CONFIGURED:
					sprintf(sr_message.description, "SENSOR NOT CONFIGURED ");
					break;
				case READING_NOT_READY:
					sprintf(sr_message.description, "SENSOR READING NOT READY ");
					break;
				case INVALID_COMMAND_TO_VSP:
					sprintf(sr_message.description, "INVALID COMMAND TO VSP ");
					break;
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

} // namespace lib
} // namespace mrrocpp

/*
 * sr_ui.cc
 *
 *  Created on: Sep 9, 2010
 *      Author: ptroja
 */

#include <cstdio>
#include <cstring>

#include "base/lib/sr/sr_ui.h"

namespace mrrocpp {
namespace lib {

sr_ui::sr_ui(process_type_t process_type, const std::string & process_name, const std::string & sr_name) :
	sr(process_type, process_name, sr_name)
{
}

// Interpretacja bledow generowanych w UI // by Y - UWAGA UZUPELNIC
void sr_ui::interpret(char * description, error_class_t message_type, uint64_t error_code0, uint64_t error_code1)
{
	switch (message_type)
	{
		case NEW_MESSAGE:
			sprintf(description, "%s", strerror(error_code0));
			break; // NEW_MESSAGE
		default:
			sprintf(description, "UNIDENTIFIED UI ERROR");
	}
}

} // namespace lib
} // namespace mrrocpp

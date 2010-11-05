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

sr_ui::sr_ui(process_type_t process_type, const std::string & process_name, const std::string & sr_name, bool _multi_thread) :
	sr(process_type, process_name, sr_name, _multi_thread)
{
}

// Interpretacja bledow generowanych w UI // by Y - UWAGA UZUPELNIC
void sr_ui::interpret(void)
{
	switch (sr_message.message_type)
	{
		case NEW_MESSAGE:
			sprintf(sr_message.description, "%s", strerror(error_tab[0]));
			break; // NEW_MESSAGE
		default:
			sprintf(sr_message.description, "UNIDENTIFIED UI ERROR");
	}
}

} // namespace lib
} // namespace mrrocpp

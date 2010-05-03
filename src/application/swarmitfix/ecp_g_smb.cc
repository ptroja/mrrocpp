/*
 * generator/ecp_g_sleep.cc
 *
 *Author: Tomasz Bem
 */

#include "ecp_g_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

//constructor with parameters: task and time to sleep [s]
pin_lock::pin_lock(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool pin_lock::first_step() {

	return true;
}

bool pin_lock::next_step() {
	return true;
}



void pin_lock::create_ecp_mp_reply() {

}

void pin_lock::get_mp_ecp_command() {

}



//constructor with parameters: task and time to sleep [s]
pin_unlock::pin_unlock(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool pin_unlock::first_step() {

	return true;
}

bool pin_unlock::next_step() {
	return true;
}


void pin_unlock::create_ecp_mp_reply() {

}

void pin_unlock::get_mp_ecp_command() {

}




//constructor with parameters: task and time to sleep [s]
pin_rise::pin_rise(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool pin_rise::first_step() {

	return true;
}

bool pin_rise::next_step() {
	return true;
}



void pin_rise::create_ecp_mp_reply() {

}

void pin_rise::get_mp_ecp_command() {

}




//constructor with parameters: task and time to sleep [s]
pin_lower::pin_lower(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool pin_lower::first_step() {

	return true;
}

bool pin_lower::next_step() {
	return true;
}

void pin_lower::create_ecp_mp_reply() {

}

void pin_lower::get_mp_ecp_command() {

}


} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp


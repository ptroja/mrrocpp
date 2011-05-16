// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <cerrno>
#include <sys/wait.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include "base/lib/messip/messip_dataport.h"

#include "base/lib/mis_fun.h"
#include "base/edp/edp_effector.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
effector::effector(shell &_shell, lib::robot_name_t l_robot_name) :
	edp_shell(_shell), robot_name(l_robot_name), config(_shell.config), msg(_shell.msg), robot_test_mode(true)
{

	if (config.exists(lib::ROBOT_TEST_MODE.c_str())) {
		robot_test_mode = config.exists_and_true(lib::ROBOT_TEST_MODE.c_str());
	}

	if (robot_test_mode) {
		msg->message("Robot test mode activated");
	}

}

effector::~effector()
{
}

/*--------------------------------------------------------------------------*/
bool effector::initialize_communication()
{

	const std::string
			server_attach_point(config.get_edp_resourceman_attach_point());

	// nawiazywanie komunikacji

	std::string full_path_to_server_attach_point("/dev/name/global/");
	full_path_to_server_attach_point += server_attach_point;

	// sprawdzenie czy nie jest juz zarejestrowany server EDP
	if (access(full_path_to_server_attach_point.c_str(), R_OK) == 0) {
		fprintf(stderr, "edp already exists() failed: %s\n", strerror(errno));
		return false;
	}

	/* Ustawienie priorytetu procesu */

	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 2);

	server_attach = messip::port_create(server_attach_point);

	if (server_attach == NULL) {
		msg->message(lib::SYSTEM_ERROR, errno, "edp: resmg failed to attach");
		fprintf(stderr, "name_attach() failed: %s\n", strerror(errno));
		return false;
	}

	msg->message("edp loaded");

	return true;
}

void effector::establish_error(lib::r_buffer_base & reply, uint64_t err0, uint64_t err1)
{
	reply.reply_type = lib::ERROR;
	reply.error_no.error0 = err0;
	reply.error_no.error1 = err1;
}

void effector::instruction_deserialization()
{
}

void effector::reply_serialization()
{
}

} // namespace common
} // namespace edp
}
// namespace mrrocpp


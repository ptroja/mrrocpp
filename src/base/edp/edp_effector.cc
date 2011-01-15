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
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <sys/wait.h>
#include <sys/types.h>
#if !defined(USE_MESSIP_SRR)
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>

#else
#include "base/lib/messip/messip_dataport.h"
#endif /* !USE_MESSIP_SRR */
#include <pthread.h>
#include <cerrno>

#include "base/lib/mis_fun.h"
#include "base/edp/edp_effector.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
effector::effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	robot_name(l_robot_name), config(_config), robot_test_mode(true)
{

	/* Lokalizacja procesu wywietlania komunikatow SR */
	msg
			= new lib::sr_edp(lib::EDP, config.value <std::string> ("resourceman_attach_point").c_str(), config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION).c_str(), true);

	sh_msg
			= new lib::sr_edp(lib::EDP, config.value <std::string> ("resourceman_attach_point").c_str(), config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION).c_str(), false);

	if (config.exists(lib::ROBOT_TEST_MODE.c_str())) {
		robot_test_mode = config.value <int> (lib::ROBOT_TEST_MODE);
	}

	if (robot_test_mode) {
		msg->message("Robot test mode activated");
	}
}

effector::~effector()
{
	delete msg;
	delete sh_msg;
}

/*--------------------------------------------------------------------------*/
bool effector::initialize_communication()
{
	const std::string
			server_attach_point(config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point"));

#if !defined(USE_MESSIP_SRR)
	// obsluga mechanizmu sygnalizacji zajetosci sprzetu
	if (!(robot_test_mode)) {

		const std::string hardware_busy_attach_point = config.value <std::string> ("hardware_busy_attach_point");

		std::string full_path_to_hardware_busy_attach_point("/dev/name/global/");
		full_path_to_hardware_busy_attach_point += hardware_busy_attach_point;

		// sprawdzenie czy nie jakis proces EDP nie zajmuje juz sprzetu
		if (access(full_path_to_hardware_busy_attach_point.c_str(), R_OK) == 0) {
			fprintf(stderr, "edp: hardware busy\n");
			return false;
		}

		name_attach_t * tmp_attach = name_attach(NULL, hardware_busy_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL);

		if (tmp_attach == NULL) {
			msg->message(lib::SYSTEM_ERROR, errno, "edp: hardware_busy_attach_point failed to attach");
			fprintf(stderr, "hardware_busy_attach_point name_attach() to %s failed: %s\n", hardware_busy_attach_point.c_str(), strerror(errno));
			// TODO: throw
			return false;
		}
	}
#endif /* !defined(USE_MESSIP_SRR */

	std::string full_path_to_server_attach_point("/dev/name/global/");
	full_path_to_server_attach_point += server_attach_point;

	// sprawdzenie czy nie jest juz zarejestrowany server EDP
	if (access(full_path_to_server_attach_point.c_str(), R_OK) == 0) {
		fprintf(stderr, "edp already exists() failed: %s\n", strerror(errno));
		return false;
	}

	/* Ustawienie priorytetu procesu */

	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 2);

	server_attach =
#if !defined(USE_MESSIP_SRR)
			name_attach(NULL, server_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL);
#else /* USE_MESSIP_SRR */
	messip::port_create(server_attach_point);
#endif /* USE_MESSIP_SRR */

	if (server_attach == NULL) {
		msg->message(lib::SYSTEM_ERROR, errno, "edp: resmg failed to attach");
		fprintf(stderr, "name_attach() failed: %s\n", strerror(errno));
		return false;
	}

	msg->message("edp loaded");

	return true;
}

void effector::establish_error(uint64_t err0, uint64_t err1)
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

